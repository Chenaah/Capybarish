"""
Interface module for robot communication and control.

This module provides the main Interface class for communicating with robot hardware,
managing data flow, and coordinating various subsystems including plugins, services,
and dashboard functionality.

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import copy
import json
import os
import signal
import sys
import time
from collections import defaultdict
from datetime import datetime
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
from rich.live import Live
from rich.table import Table

from .communication import CommunicationManager, ConnectionStatus, UDPProtocol
from .dashboard_server import DashboardServer
from .data_struct import SentDataStruct  # Legacy support
from .interpreter import interpret_motor_error, interpret_motor_mode
from .kbhit import KBHit
from .natnet.NatNetClient import NatNetClient
from .plugin_system import PluginManager, PluginType
from .service_registry import RobotModuleHealthChecker, ServiceRegistry, ServiceType
from .utils import convert_np_arrays_to_lists, load_cached_pings

# Try to import generated message types, fall back to legacy if not available
try:
    from .generated import ReceivedData  # New generated message type
    USE_GENERATED_MESSAGES = True
except ImportError:
    ReceivedData = None
    USE_GENERATED_MESSAGES = False
    raise ImportError("Generated message types not found")


# Constants
DEFAULT_PORT = 6666
DEFAULT_BUFFER_SIZE = 4096
DEFAULT_CONNECTION_TIMEOUT = 5.0
DEFAULT_HEARTBEAT_TIMEOUT = 30.0
DEFAULT_HEALTH_CHECK_INTERVAL = 10.0
MAX_VELOCITY_LIMIT = 20.0
MAX_KP_LIMIT = 100.0
MAX_KD_LIMIT = 100.0
MAX_POSITION_DELTA = 3.14
COMMAND_RESET_TIMEOUT = 0.5
SAFETY_STOP_ITERATIONS = 10
SAFETY_STOP_DELAY = 0.01
MAX_PENDING_THRESHOLD = 5
REFRESH_RATE = 20


def sanitize_list(data_list: List[Any]) -> List[Any]:
    """Sanitize a list by applying appropriate validation to each element."""
    return [validation_map[type(element)](element) for element in data_list]


def sanitize_dict(to_sanitize: Dict[str, Any]) -> Dict[str, Any]:
    """Sanitize a dictionary by applying appropriate validation to each value."""
    for key, value in to_sanitize.items():
        if type(value) not in validation_map:
            print(f"[ERROR] Unrecognized type: {type(value)}")
            print(f"[ERROR] Key: {key}, Value: {value}")
        to_sanitize[key] = validation_map[type(value)](value)
    return to_sanitize


# Validation mapping for data sanitization
validation_map = {
    dict: sanitize_dict,
    list: sanitize_list,
    np.ndarray: lambda x: x.tolist(),
    np.float64: lambda x: float(x),
    int: lambda x: x,
    float: lambda x: x,
    str: lambda x: x,
    bool: lambda x: x,
    type(None): lambda x: "None",
}


class Interface:
    """Main interface for robot hardware communication and control.

    This class manages communication with robot modules, handles data flow,
    coordinates plugins and services, and provides a unified interface for
    robot control and monitoring.

    Attributes:
        cfg: Configuration object containing system settings
        module_ids: List of module IDs to communicate with
        data: Dictionary storing received data from modules
        switch_on: Global switch state for motor control
        ready_to_go: System readiness status
    """

    def __init__(self, cfg) -> None:
        """Initialize the Interface with the given configuration.

        Args:
            cfg: Configuration object containing all system settings
        """

        # Configuration and core state
        self.cfg = cfg
        self.motor_commands: Dict[int, Dict[str, Any]] = defaultdict(dict)
        self.pending_counter: Dict[int, int] = {}
        self.update_config(cfg)
        # Simulation rendering (should be moved to configuration in future)
        self.enable_sim_render = True
        
        # Control whether Interface sets up its own signal handler
        # Set to False if the calling code will handle signals
        self.setup_signal_handler = getattr(cfg.interface, 'setup_signal_handler', True)

        # Timing and data tracking
        self.start_time = time.time()
        self.optitrack_time = -1
        self.optitrack_data: Dict[int, List] = {}
        self.module_address_book: Dict[int, Tuple[str, int]] = {}
        self.pending_modules: Set[int] = set()
        self.pings = load_cached_pings()
        self.data: Dict[int, Dict[str, Any]] = {}  # Module data storage

        # Control state
        # Global motor switch (future: individual module control)
        self.switch_on = 0
        self.send_dt = 0.0
        self.compute_time = 0.0
        self.all_motor_on = False
        self.overwrite_actions: Optional[np.ndarray] = None  # For debugging
        self.ready_to_go = False

        # Setup signal handling and UI
        if self.setup_signal_handler:
            signal.signal(signal.SIGINT, self.signal_handler)
        self.kb = KBHit()
        print("[e] enable; [d] disable")
        self.live = Live(self._generate_table(), refresh_per_second=REFRESH_RATE)
        self.live.__enter__()

        # Initialize core subsystems
        self._setup_communication(check_connection=True)
        self._setup_service_registry()
        self._setup_plugin_system()

        # Setup dashboard if enabled
        if self.enable_dashboard or self.enable_sim_render:
            self.dashboard_server = DashboardServer()

        # Setup OptiTrack if enabled
        if "optitrack" in self.sources:
            self._setup_optitrack()

        # Setup logging
        self._setup_logging()

        # Initialize tracking variables
        self._initialize_tracking_variables()

    def _setup_optitrack(self) -> None:
        """Setup OptiTrack motion capture system."""
        print("[Server] Connecting to OptiTrack...")
        self.streaming_client = NatNetClient()
        self.streaming_client.set_client_address("0.0.0.0")
        self.streaming_client.set_server_address("129.105.73.172")
        self.streaming_client.set_use_multicast(False)
        self.streaming_client.set_print_level(0)

        self.streaming_client.new_frame_listener = self._receive_new_frame
        self.streaming_client.rigid_body_listener = self._receive_rigid_body_frame

        is_running = self.streaming_client.run()
        if not is_running:
            print("[WARNING] OptiTrack connection failed")

    def _setup_logging(self) -> None:
        """Setup data logging if enabled."""
        if self.log_dir is not None:
            os.makedirs(self.log_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            log_filename = os.path.join(self.log_dir, f"{timestamp}_raw.txt")
            self.log_file = open(log_filename, "w")

    def _initialize_tracking_variables(self) -> None:
        """Initialize all tracking and monitoring variables."""
        # Performance tracking
        self.received_dt = 0.0
        self.max_received_dt = 0.0
        self.latency = 0.0
        self.max_latency = 0.0
        self.step_counter = 0

        # Timing variables
        self.last_motor_com_time = time.time()
        self.last_rcv_timestamp = time.time()

        # Data tracking
        self.last_log_info = ""
        self.module_info_dict: Dict[int, str] = {}
        self.module_lastinfo_dict: Dict[int, str] = {}
        self.publish_log_info = ""
        self.abnormal_modules: Set[int] = set()
        self.calibration_command_buffer = {i: [] for i in self.module_ids}

        # OptiTrack position tracking
        self.pos_world_opti_last = np.zeros(3)
        self._last_optitrack_time = time.time()
        self.vel_world_opti = np.zeros(3)
        self.pos_world_opti = np.zeros(3)

    def update_config(self, cfg) -> None:
        """Update configuration settings.

        Args:
            cfg: Configuration object with updated settings
        """
        # Core module configuration
        self.module_ids = cfg.interface.module_ids
        self.torso_module_id = cfg.interface.torso_module_id
        self.sources = cfg.interface.sources
        self.struct_format = cfg.interface.struct_format
        self.protocol = cfg.interface.protocol

        # Control configuration
        self.filter_action = cfg.agent.filter_action
        self.enable_firmware_filter = cfg.interface.enable_filter
        self.optitrack_rigibody = cfg.interface.optitrack_rigibody

        # Validate protocol
        if self.protocol not in ["UDP", "USB"]:
            raise ValueError(
                f"Unsupported protocol: {self.protocol}. Only UDP and USB are supported."
            )

        # Robot configuration
        self.dt = cfg.robot.dt
        self.motor_range = np.array(cfg.robot.motor_range)
        self.kp_ratio = cfg.interface.kp_ratio
        self.kd_ratio = cfg.interface.kd_ratio
        self.calibration_modes = cfg.interface.calibration_modes
        self.broken_motors = cfg.interface.broken_motors

        # Validate calibration modes
        if self.calibration_modes is not None:
            if len(self.calibration_modes) != len(self.module_ids):
                raise ValueError(
                    f"Length of calibration_modes ({len(self.calibration_modes)}) "
                    f"must equal number of modules ({len(self.module_ids)})"
                )

        # Interface configuration
        self.enable_dashboard = cfg.interface.dashboard
        self.check_action_safety = cfg.interface.check_action_safety
        self.log_dir = cfg.logging.robot_data_dir

        # Reset state
        self._reset_motor_commands()
        for module_id in self.module_ids:
            self.pending_counter[module_id] = 0

    def _reset_motor_commands(self) -> None:
        """Reset all motor commands to default state."""
        for module_id in self.module_ids:
            self.motor_commands[module_id]["calibration"] = 0
            self.motor_commands[module_id]["restart"] = 0

    def _generate_table(self) -> Table:
        """Generate status table for display.

        Returns:
            Rich Table object with current system status
        """
        border_style = "green" if self.ready_to_go else "yellow"
        table = Table(border_style=border_style)

        # Add columns
        columns = [
            "Module",
            "Connection",
            "Address",
            "Latency",
            "Mode",
            "Voltage",
            "Current",
            "Energy",
            "Torque",
            "Switch",
            "Error",
        ]
        for column in columns:
            table.add_column(column, justify="center")

        table.add_section()

        # Add row for each module
        for module_id in self.module_ids:
            row_data = self._get_module_status_row(module_id)
            table.add_row(*row_data, end_section=True)

        # Add system performance row
        table.add_row(
            "[bold]dt",
            f"{self.send_dt:.3f}",
            "[bold]Comp Time",
            f"{self.compute_time:.3f}",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            end_section=True,
        )
        return table

    def _get_module_status_row(self, module_id: int) -> List[str]:
        """Get status row data for a specific module.

        Args:
            module_id: ID of the module to get status for

        Returns:
            List of strings representing the status row
        """
        # Initialize default values
        addr = "..."
        conn = "[red]Disconnected"
        mode = "Unknown"
        voltage = "Unknown"
        current = "Unknown"
        energy = "Unk"
        torque = "Unknown"
        switch = "[green]On" if self.switch_on else "[red]Off"
        error = "Unknown"
        ping = "Unknown"

        # Update connection info
        if module_id in self.module_address_book:
            conn = "[green]Connected"
            addrs = self.module_address_book[module_id]
            addr = f"{addrs[0]}:{addrs[1]}"

        if module_id in self.pending_modules:
            conn = "[yellow]Lost"

        # Update data from module
        if module_id in self.data:
            module_data = self.data[module_id]

            # Motor mode
            motor_mode = module_data["motor_mode"]
            mode = (
                "[green]Running" if motor_mode == 2 else f"[red]{interpret_motor_mode(motor_mode)}"
            )

            # Error status
            error_id = module_data["motor_error"]
            error = interpret_motor_error(error_id)
            if not error and module_data["esp_errors"][0] != 1:
                error = f"ESP32 Rebooted ({module_data['esp_errors'][0]})"
            error = f"[red]{error}" if error else "[green]None"

            # Sensor readings
            voltage = (
                f"{module_data['voltage']:.2f} V" if module_data["voltage"] != 0 else "Unknown"
            )
            current = f"{module_data['current']:10.6f} A"
            energy = f"{module_data['energy']:.2f} J"
            torque = f"{module_data['motor_torque']:.2f} Nm"

        # Update ping info
        if module_id in self.pings:
            ping = f"{self.pings[module_id]:.2f} ms"

        return [
            f"[bold]{module_id}",
            conn,
            addr,
            ping,
            mode,
            voltage,
            current,
            energy,
            torque,
            switch,
            error,
        ]

    def _update_motor_commands(self) -> None:
        """Update motor commands from dashboard input."""
        enable, disable, calibrate, reset, debug_pos_list = self.dashboard_server.get_commands()

        if enable:
            self.switch_on = 1
        if disable:
            self.switch_on = 0
        if calibrate:
            for module_id in self.module_ids:
                self.motor_commands[module_id]["calibration"] = 1
            self.last_motor_com_time = time.time()
        if reset:
            for module_id in self.module_ids:
                self.motor_commands[module_id]["restart"] = 1
            self.last_motor_com_time = time.time()
        if debug_pos_list is not None:
            self.overwrite_actions = np.array(debug_pos_list)

        # Auto-reset commands after timeout
        if time.time() - self.last_motor_com_time > COMMAND_RESET_TIMEOUT:
            self._reset_motor_commands()

    def _setup_communication(self, check_connection: bool = True) -> None:
        """Setup modern communication manager.

        Args:
            check_connection: Whether to check initial connections

        Raises:
            NotImplementedError: If protocol is not supported
            RuntimeError: If communication setup fails
        """
        if self.protocol != "UDP":
            raise NotImplementedError("Only UDP protocol is currently supported")

        print("[Server] Setting up modern communication layer...")

        # Create communication protocol and manager
        udp_protocol = UDPProtocol()
        self.comm_manager = CommunicationManager(
            protocol=udp_protocol,
            struct_format=self.struct_format,
            expected_modules=self.module_ids,
            connection_timeout=DEFAULT_CONNECTION_TIMEOUT,
            max_pending_count=int(MAX_PENDING_THRESHOLD / self.dt),
        )

        # Setup callbacks for module events
        self.comm_manager.on_module_connected = self._on_module_connected
        self.comm_manager.on_module_disconnected = self._on_module_disconnected

        # Setup communication
        success = self.comm_manager.setup(
            check_initial_connections=check_connection,
            port=DEFAULT_PORT,
            buffer_size=DEFAULT_BUFFER_SIZE,
        )

        if not success:
            raise RuntimeError("Failed to setup communication")

        # Maintain backward compatibility
        self._sync_legacy_structures()

    def _on_module_connected(self, module_id: int, module_info) -> None:
        """Callback for when a module connects."""
        print(f"[Server] Module {module_id} connected from {module_info.address}")
        self.live.update(self._generate_table())

    def _on_module_disconnected(self, module_id: int, module_info) -> None:
        """Callback for when a module disconnects."""
        print(f"[Server] Module {module_id} disconnected")
        self.live.update(self._generate_table())

    def _sync_legacy_structures(self):
        """Sync new communication manager state with legacy data structures."""
        # Update module address book and pings for backward compatibility
        all_modules = self.comm_manager.get_all_modules_info()

        for module_id, module_info in all_modules.items():
            self.module_address_book[module_id] = module_info.address
            if module_info.ping_time:
                self.pings[module_id] = module_info.ping_time

        # Update pending modules
        self.pending_modules.clear()
        for module_id, module_info in all_modules.items():
            if module_info.status == ConnectionStatus.PENDING:
                self.pending_modules.add(module_id)
            self.pending_counter[module_id] = module_info.pending_count

    def _setup_service_registry(self) -> None:
        """Setup service registry for managing robot services."""
        print("[Server] Setting up service registry...")

        # Create service registry
        self.service_registry = ServiceRegistry(
            heartbeat_timeout=DEFAULT_HEARTBEAT_TIMEOUT,
            health_check_interval=DEFAULT_HEALTH_CHECK_INTERVAL,
        )

        # Add health checker for robot modules
        robot_health_checker = RobotModuleHealthChecker(self.comm_manager)
        self.service_registry.add_health_checker(ServiceType.ROBOT_MODULE, robot_health_checker)

        # Register callbacks for service events
        self.service_registry.on_service_registered.append(self._on_service_registered)
        self.service_registry.on_service_deregistered.append(self._on_service_deregistered)
        self.service_registry.on_service_status_changed.append(self._on_service_status_changed)

        # Register robot modules as services
        for module_id in self.module_ids:
            service_id = self.service_registry.register_service(
                name=f"RobotModule-{module_id}",
                service_type=ServiceType.ROBOT_MODULE,
                metadata={
                    "module_id": module_id,
                    "struct_format": self.struct_format,
                    "protocol": self.protocol,
                },
                tags={"robot", "motor", f"module_{module_id}"},
            )
            print(f"[Server] Registered robot module {module_id} as service {service_id}")

        # Register communication service
        self.service_registry.register_service(
            name="CommunicationManager",
            service_type=ServiceType.COMMUNICATION,
            metadata={
                "protocol": self.protocol,
                "port": DEFAULT_PORT,
                "expected_modules": self.module_ids,
            },
            tags={"communication", "udp"},
        )

        # Register dashboard service if enabled
        if self.enable_dashboard:
            self.service_registry.register_service(
                name="Dashboard",
                service_type=ServiceType.DASHBOARD,
                metadata={"enabled": self.enable_dashboard},
                tags={"dashboard", "ui"},
            )

        # Start background tasks
        self.service_registry.start_background_tasks()
        print("[Server] Service registry setup complete")

    def _on_service_registered(self, service):
        """Callback when a service is registered."""
        print(
            f"[ServiceRegistry] Service registered: {service.name} ({service.service_type.value})"
        )

    def _on_service_deregistered(self, service):
        """Callback when a service is deregistered."""
        print(
            f"[ServiceRegistry] Service deregistered: {service.name} ({service.service_type.value})"
        )

    def _on_service_status_changed(self, service, old_status, new_status):
        """Callback when a service status changes."""
        print(
            f"[ServiceRegistry] Service {service.name} status changed: {old_status.value} -> {new_status.value}"
        )

    def _setup_plugin_system(self) -> None:
        """Setup plugin system for extensible data processing."""
        print("[Server] Setting up plugin system...")

        # Create plugin manager
        self.plugin_manager = PluginManager(plugin_directories=["plugins"])

        # Setup plugin event callbacks
        self.plugin_manager.on_plugin_loaded.append(self._on_plugin_loaded)
        self.plugin_manager.on_plugin_started.append(self._on_plugin_started)
        self.plugin_manager.on_plugin_error.append(self._on_plugin_error)

        # Discover and load plugins
        discovered_plugins = self.plugin_manager.discover_plugins()
        print(f"[Server] Discovered {len(discovered_plugins)} plugins")

        # Load useful plugins based on configuration
        plugins_to_load = []

        # Load IMU processor if we have IMU sources
        if "imu" in self.sources:
            plugins_to_load.append(
                ("plugins.imu_processor", {"filter_alpha": 0.1, "calibration_samples": 100})
            )

        # Load OptiTrack source if we have optitrack sources
        if "optitrack" in self.sources:
            plugins_to_load.append(
                (
                    "plugins.optitrack_source",
                    {
                        "server_address": "129.105.73.172",
                        "client_address": "0.0.0.0",
                        "use_multicast": False,
                        "rigid_body_id": self.optitrack_rigibody,
                    },
                )
            )

        # Load plugins
        loaded_count = 0
        for module_name, config in plugins_to_load:
            try:
                if self.plugin_manager.load_plugin(module_name, config):
                    plugin_name = module_name.split(".")[-1]
                    if self.plugin_manager.initialize_plugin(plugin_name.title().replace("_", "")):
                        if self.plugin_manager.start_plugin(plugin_name.title().replace("_", "")):
                            loaded_count += 1
            except Exception as e:
                print(f"[Server] Failed to load plugin {module_name}: {e}")

        print(f"[Server] Plugin system setup complete - {loaded_count} plugins loaded")

    def _on_plugin_loaded(self, plugin_name: str, plugin_info):
        """Callback when a plugin is loaded."""
        print(f"[PluginSystem] Plugin loaded: {plugin_name} v{plugin_info.metadata.version}")

    def _on_plugin_started(self, plugin_name: str, plugin_info):
        """Callback when a plugin is started."""
        print(f"[PluginSystem] Plugin started: {plugin_name}")

        # Register plugin as a service
        if hasattr(self, "service_registry"):
            service_type = (
                ServiceType.DATA_SOURCE
                if plugin_info.metadata.plugin_type == PluginType.DATA_SOURCE
                else ServiceType.DATA_PROCESSOR
            )
            self.service_registry.register_service(
                name=f"Plugin-{plugin_name}",
                service_type=service_type,
                metadata={
                    "plugin_name": plugin_name,
                    "plugin_version": plugin_info.metadata.version,
                    "plugin_type": plugin_info.metadata.plugin_type.value,
                },
                tags=plugin_info.metadata.tags | {"plugin"},
            )

    def _on_plugin_error(self, plugin_name: str, plugin_info, error_msg: str):
        """Callback when a plugin encounters an error."""
        print(f"[PluginSystem] Plugin error in {plugin_name}: {error_msg}")

    def receive_module_data(self) -> None:
        """Receive data from all modules using modern communication manager."""
        # Use the new communication manager to receive data
        received_data = self.comm_manager.receive_data_batch(max_messages=100)

        # Process received data
        for module_id, data_dict in received_data.items():
            # Calculate the latency (maintaining compatibility with existing logic)
            curr_timestamp = float(time.time()) - self.start_time
            data_dict["latency"] = (curr_timestamp - data_dict["last_rcv_timestamp"] - self.dt) / 2

            # Store the data
            self.data[module_id] = data_dict
            self.module_info_dict[module_id] = data_dict["log_info"]

            # Handle log info messages
            if module_id in self.module_lastinfo_dict:
                if (
                    data_dict["log_info"] != self.module_lastinfo_dict[module_id]
                    and data_dict["log_info"] != ""
                ):
                    print(f"[ESP32 MESSAGE] [Module {module_id}] ", data_dict["log_info"])
                    self.module_lastinfo_dict[module_id] = data_dict["log_info"]
            else:
                self.module_lastinfo_dict[module_id] = data_dict["log_info"]
            self.publish_log_info = data_dict["log_info"]

            # Handle switch off requests
            if data_dict["switch_off_request"]:
                print(f"[Server] Receive switch off request from Module {module_id}. Switch off!")
                self._disable_motor()

            self.latency = data_dict["latency"]

        # Sync legacy data structures for backward compatibility
        self._sync_legacy_structures()

        # Log data if logging is enabled
        if self.log_dir is not None:
            self.log_file.write(json.dumps(sanitize_dict(copy.deepcopy(self.data))) + "\n")
            self.log_file.flush()

        # Check if we received data from all expected modules
        connected_modules = self.comm_manager.get_connected_modules()
        missing_modules = set(self.module_ids) - set(received_data.keys())

        if missing_modules:
            # Some modules didn't send data this cycle - this is handled by the communication manager
            # but we need to update the pending_modules set for compatibility
            pass

        # Update motor status
        on_modules = set(self.data.keys()) & set(self.module_ids)
        self.all_motor_on = all([self.data[module_id]["motor_on"] for module_id in on_modules])

        # Check system health
        self._check_health()

    def _check_health(self) -> None:
        """Check system health and identify abnormal modules."""
        # Check for inconsistent motor states
        if not self.all_motor_on and self.switch_on:
            motors_off = [
                mid
                for mid in self.module_ids
                if mid in self.data and not self.data[mid]["motor_on"]
            ]
            if motors_off and len(motors_off) != len(self.module_ids):
                # Some modules are off when they should be on
                for module_id in motors_off:
                    self.abnormal_modules.add(module_id)
                    print(f"[DEBUG] Module {module_id} current: {self.data[module_id]['current']}")
                print(
                    f"[ERROR][Server] Not all modules are on! Abnormal modules: {self.abnormal_modules}"
                )

        # Check for disconnected modules
        max_pending = MAX_PENDING_THRESHOLD / self.dt
        for module_id in self.module_ids:
            if self.pending_counter[module_id] > max_pending:
                print(f"[ERROR][Server] Module {module_id} is not connected!")
                self.abnormal_modules.add(module_id)
            elif module_id in self.data and module_id in self.abnormal_modules:
                # Module recovered
                if self.data[module_id]["motor_on"]:
                    self.abnormal_modules.remove(module_id)

        if self.abnormal_modules:
            print(f"[Server] Abnormal modules: {self.abnormal_modules}")

    def _action_safety_check(self, target: float, module_id: int) -> Optional[float]:
        """Check if action is safe and apply safety limits.

        Args:
            target: Target position
            module_id: Module ID to check

        Returns:
            Safe target position or None if module not available
        """
        if module_id not in self.data:
            print(f"[WARN][Server] Module {module_id} is not connected!")
            return None

        curr_pos = self.data[module_id]["motor_pos"]
        position_delta = abs(target - curr_pos)

        if position_delta > MAX_POSITION_DELTA:
            print(
                f"[WARN][Server] Module {module_id} target {target:.3f} is too far "
                f"from current position {curr_pos:.3f} (delta: {position_delta:.3f})!"
            )
            return curr_pos + 0.1 * np.tanh(target - curr_pos)

        return target

    def send_action(
        self,
        pos_actions: np.ndarray,
        vel_actions: Optional[np.ndarray] = None,
        kps: Optional[np.ndarray] = None,
        kds: Optional[np.ndarray] = None,
    ) -> None:
        """Send control actions to robot modules.

        Args:
            pos_actions: Position commands for each module
            vel_actions: Velocity commands for each module (optional)
            kps: Proportional gains for each module (optional)
            kds: Derivative gains for each module (optional)
        """
        # Update motor commands from dashboard
        if self.enable_dashboard:
            self._update_motor_commands()

        # Set default values
        if vel_actions is None:
            vel_actions = np.zeros_like(pos_actions)
        if kps is None:
            kps = np.full(len(pos_actions), 8.0)
        if kds is None:
            kds = np.full(len(pos_actions), 0.2)

        # Handle action override for debugging
        if self.overwrite_actions is not None:
            pos_actions = self.overwrite_actions
            print("[Server] Overwrite actions: ", pos_actions)

        # Validate input dimensions
        self._validate_action_dimensions(pos_actions, vel_actions, kps, kds)

        # Store actions for logging
        self.actions = pos_actions

        # Apply safety limits
        vel_actions = np.clip(vel_actions, -MAX_VELOCITY_LIMIT, MAX_VELOCITY_LIMIT)
        kps_real = np.clip(kps * self.kp_ratio, 0, MAX_KP_LIMIT)
        kds_real = np.clip(kds * self.kd_ratio, 0, MAX_KD_LIMIT)

        # Handle broken motors
        if self.broken_motors is not None:
            kps_real[self.broken_motors] = 0
            kds_real[self.broken_motors] = 0
            print("Applied broken motor compensation - Kp: ", kps_real)

        # Send commands to each module
        self.curr_timestamp = time.time() - self.start_time
        for target_pos, target_vel, module_id, kp, kd in zip(
            pos_actions, vel_actions, self.module_ids, kps_real, kds_real
        ):
            # Apply safety check if enabled
            if self.check_action_safety:
                safe_pos = self._action_safety_check(target_pos, module_id)
                if safe_pos is None:
                    continue
                target_pos = safe_pos

            # Prepare command data
            calibration_cmd = (
                self.calibration_command_buffer[module_id].pop(0)
                if self.calibration_command_buffer[module_id]
                else 0
            )

            data_to_send = [
                target_pos,
                target_vel,
                kp,
                kd,
                int(self.enable_firmware_filter),
                self.switch_on,
                calibration_cmd,
                self.motor_commands[module_id]["restart"],
                self.curr_timestamp,
            ]

            # Log restart command
            if self.motor_commands[module_id]["restart"] == 1:
                print(f"[Server] Restarting Module {module_id}...")

            # Send command
            if module_id in self.module_address_book:
                self._send_msg(data_to_send, self.module_address_book[module_id])
                self.step_counter += 1
            else:
                print(f"[WARN] Module {module_id} address not found")

    def _validate_action_dimensions(
        self, pos_actions: np.ndarray, vel_actions: np.ndarray, kps: np.ndarray, kds: np.ndarray
    ) -> None:
        """Validate that action arrays have correct dimensions.

        Args:
            pos_actions: Position actions array
            vel_actions: Velocity actions array
            kps: Proportional gains array
            kds: Derivative gains array

        Raises:
            ValueError: If any array has incorrect dimensions
        """
        expected_len = len(self.module_ids)

        if len(pos_actions) != expected_len:
            raise ValueError(
                f"pos_actions length ({len(pos_actions)}) != module count ({expected_len})"
            )
        if len(vel_actions) != expected_len:
            raise ValueError(
                f"vel_actions length ({len(vel_actions)}) != module count ({expected_len})"
            )
        if len(kps) != expected_len:
            raise ValueError(f"kps length ({len(kps)}) != module count ({expected_len})")
        if len(kds) != expected_len:
            raise ValueError(f"kds length ({len(kds)}) != module count ({expected_len})")

    def _send_msg(self, data: List[Any], address: Tuple[str, int]) -> None:
        """Send message using modern communication manager.

        Args:
            data: Command data to send (list of values)
            address: Target address (host, port)
        """
        # Extract module_id from address
        module_id = None
        for mid, addr in self.module_address_book.items():
            if addr == address:
                module_id = mid
                break

        if module_id is None:
            print(f"[ERROR] Could not find module_id for address {address}")
            return

        try:
            # Create command using generated message type if available
            if USE_GENERATED_MESSAGES:
                # Data format: [target_pos, target_vel, kp, kd, enable_filter, switch, calibrate, restart, timestamp]
                command_data = ReceivedData(
                    target=data[0],
                    target_vel=data[1],
                    kp=data[2],
                    kd=data[3],
                    enable_filter=data[4],
                    switch_=data[5],
                    calibrate=data[6],
                    restart=data[7],
                    timestamp=data[8]
                )
            else:
                # Fallback to old SentDataStruct
                command_data = SentDataStruct(*data)
            
            success = self.comm_manager.send_command(module_id, command_data)

            if not success:
                print(f"[ERROR] Failed to send command to module {module_id}")
        except Exception as e:
            print(f"[ERROR] Exception sending command to module {module_id}: {e}")

    def _restart_motor(self, module_id: Optional[int] = None) -> None:
        """Restart motor modules.

        Args:
            module_id: Specific module to restart, None for all modules,
                      'auto' to restart only abnormal modules
        """
        self._disable_motor()
        print("[Server] Initiating motor restart...")

        if module_id is None:
            # Restart all modules
            for mid in self.module_ids:
                self.motor_commands[mid]["restart"] = 1
        elif module_id == "auto":
            # Restart abnormal modules, or all if none abnormal
            target_modules = (
                self.abnormal_modules if self.abnormal_modules else set(self.module_ids)
            )
            for mid in target_modules:
                self.motor_commands[mid]["restart"] = 1
        else:
            # Restart specific module
            if module_id in self.module_ids:
                self.motor_commands[module_id]["restart"] = 1
            else:
                print(f"[ERROR] Invalid module_id: {module_id}")
                return

        self._reset()

    def _fix_motor(self, module_id: Optional[str] = None) -> None:
        """Attempt to fix motor issues through calibration.

        Args:
            module_id: 'auto' to fix abnormal modules, None for manual specification
        """
        self._disable_motor()
        print("[Server] Initiating motor fix procedure...")

        if module_id == "auto":
            for mid in self.abnormal_modules:
                self.motor_commands[mid]["calibration"] = 2  # Auto-calibration
            print(
                f"[Server] Applied auto-calibration to {len(self.abnormal_modules)} abnormal modules"
            )

        self._reset()

    def _reset(self) -> None:
        """Reset performance tracking metrics."""
        self.max_received_dt = 0.0
        self.max_latency = 0.0

    def signal_handler(self, signum: int, frame) -> None:
        """Handle shutdown signal gracefully.

        Args:
            signum: Signal number
            frame: Current stack frame
        """
        print("[Server] Shutdown signal received, stopping motors safely...")

        try:
            # Emergency stop - send zero commands multiple times
            self.switch_on = 0
            zeros = np.zeros(len(self.module_ids))

            for _ in range(SAFETY_STOP_ITERATIONS):
                try:
                    self.send_action(zeros, kps=zeros, kds=zeros)
                    time.sleep(SAFETY_STOP_DELAY)
                except Exception as e:
                    print(f"[ERROR] Failed to send stop command: {e}")
                    break

            # Graceful shutdown of subsystems
            self._shutdown_subsystems()

        except Exception as e:
            print(f"[ERROR] Error during shutdown: {e}")
        finally:
            # Ensure UI is closed and cursor is restored
            self._cleanup_display()
            print("[Server] Shutdown complete")
            sys.exit(0)

    def _cleanup_display(self) -> None:
        """Cleanup Rich Live display and restore terminal state."""
        if hasattr(self, "live"):
            try:
                self.live.__exit__(None, None, None)
            except Exception as e:
                print(f"[ERROR] Failed to exit Live display: {e}")
        
        # Explicitly show cursor as a safety measure
        try:
            sys.stdout.write('\033[?25h')  # ANSI escape code to show cursor
            sys.stdout.flush()
        except Exception as e:
            print(f"[ERROR] Failed to restore cursor: {e}")

    def _shutdown_subsystems(self) -> None:
        """Shutdown all subsystems gracefully."""
        # Close OptiTrack streaming
        if hasattr(self, "streaming_client"):
            try:
                self.streaming_client.shutdown()
            except Exception as e:
                print(f"[ERROR] Failed to shutdown OptiTrack: {e}")

        # Close plugin system
        if hasattr(self, "plugin_manager"):
            try:
                self.plugin_manager.shutdown()
            except Exception as e:
                print(f"[ERROR] Failed to shutdown plugin manager: {e}")

        # Close service registry
        if hasattr(self, "service_registry"):
            try:
                self.service_registry.stop_background_tasks()
            except Exception as e:
                print(f"[ERROR] Failed to shutdown service registry: {e}")

        # Close communication manager
        if hasattr(self, "comm_manager"):
            try:
                self.comm_manager.close()
            except Exception as e:
                print(f"[ERROR] Failed to shutdown communication manager: {e}")

        # Close log file
        if hasattr(self, "log_file"):
            try:
                self.log_file.close()
            except Exception as e:
                print(f"[ERROR] Failed to close log file: {e}")

    def __del__(self) -> None:
        """Destructor to ensure terminal state is restored."""
        self._cleanup_display()

    def ready(self) -> bool:
        """Check if system is ready for operation.

        Returns:
            True if system is ready, False otherwise
        """
        self.ready_to_go = self.all_motor_on and self.switch_on
        self.live.update(self._generate_table())
        return self.ready_to_go

    def _receive_new_frame(self, data_frame: Dict[str, Any]) -> None:
        """Handle new OptiTrack frame data.

        Args:
            data_frame: Frame data from OptiTrack
        """
        self.optitrack_time = data_frame["frame_number"]

    def _receive_rigid_body_frame(self, new_id: int, pos: List[float], rot: List[float]) -> None:
        """Handle new OptiTrack rigid body frame.

        Args:
            new_id: Rigid body ID
            pos: Position data [x, y, z]
            rot: Rotation data [qx, qy, qz, qw]
        """
        self.optitrack_data[new_id] = [list(pos), list(rot)]

        if self.optitrack_rigibody is not None and new_id == self.optitrack_rigibody:
            current_time = time.time()
            self.pos_world_opti = np.array(pos)

            # Calculate velocity
            opti_dt = current_time - self._last_optitrack_time
            if opti_dt > 0:
                self.vel_world_opti = (self.pos_world_opti - self.pos_world_opti_last) / opti_dt

            self._last_optitrack_time = current_time
            self.pos_world_opti_last = self.pos_world_opti.copy()

    def get_observable_data(self) -> Dict[str, Any]:
        """Get observable data from all configured sources.

        Returns:
            Dictionary containing all observable data
        """
        self.observable_data: Dict[str, Any] = {}
        self.data_source: Dict[str, str] = {}

        # Process each data source
        for source in self.sources:
            # Process data sources in priority order (later sources override earlier ones)
            if source == "imu" and self.torso_module_id in self.data:
                imu_data = self.data[self.torso_module_id]
                self.observable_data["acc_body"] = imu_data["acc_body_imu"]
                self.observable_data["ang_vel_body"] = imu_data["body_omega_imu"]
                self.observable_data["quat"] = imu_data["body_rot_imu"]
                self.data_source.update({k: "IMU" for k in ["acc_body", "ang_vel_body", "quat"]})
            elif source == "optitrack":
                self.observable_data["pos_world"] = self.pos_world_opti
                self.observable_data["vel_world"] = self.vel_world_opti
                self.data_source.update({k: "Optitrack" for k in ["pos_world", "vel_world"]})

            elif source == "uwb" and self.torso_module_id in self.data:
                uwb_data = self.data[self.torso_module_id]
                self.observable_data["pos_world"] = uwb_data["pos_world_uwb"]
                self.observable_data["vel_world"] = uwb_data["vel_world_uwb"]
                self.data_source.update({k: "UWB" for k in ["pos_world", "vel_world"]})

            elif source == "gps":
                raise NotImplementedError("GPS data source not yet implemented")
            else:
                raise ValueError(f"Unknown data source: {source}")

        # Get module data in consistent order (matching action order)
        sorted_data = [
            self.data[module_id] for module_id in self.module_ids if module_id in self.data
        ]

        # Extract motor and sensor data
        if sorted_data:
            self.observable_data["dof_pos"] = np.array([data["motor_pos"] for data in sorted_data])
            self.observable_data["dof_vel"] = np.array([data["motor_vel"] for data in sorted_data])
            self.observable_data["energy"] = np.array([data["energy"] for data in sorted_data])

            # Multi-module sensor data
            self.observable_data["quats"] = np.array([data["body_rot_imu"] for data in sorted_data])
            self.observable_data["gyros"] = np.array(
                [data["body_omega_imu"] for data in sorted_data]
            )
            self.observable_data["accs"] = np.array([data["acc_body_imu"] for data in sorted_data])
        else:
            # No data available - set empty arrays
            num_modules = len(self.module_ids)
            self.observable_data["dof_pos"] = np.zeros(num_modules)
            self.observable_data["dof_vel"] = np.zeros(num_modules)
            self.observable_data["energy"] = np.zeros(num_modules)
            self.observable_data["quats"] = np.zeros((num_modules, 4))
            self.observable_data["gyros"] = np.zeros((num_modules, 3))
            self.observable_data["accs"] = np.zeros((num_modules, 3))

        # Robot system state
        self.observable_data["robot_switch_on"] = self.switch_on
        self.observable_data["robot_send_dt"] = self.send_dt
        self.observable_data["robot_received_dt"] = np.array(
            [self.received_dt, self.max_received_dt]
        )
        self.observable_data["robot_latency"] = np.array([self.latency, self.max_latency])
        self.observable_data["robot_motor_commands"] = list(self.motor_commands.values())
        self.observable_data["robot_motor_message"] = [self.publish_log_info]

        # Module-specific robot state
        if sorted_data:
            self.observable_data["robot_motor_torque"] = [
                data["motor_torque"] for data in sorted_data
            ]
            self.observable_data["robot_temperature"] = [
                data["temperature"] for data in sorted_data
            ]
            self.observable_data["robot_voltage"] = [data["voltage"] for data in sorted_data]
            self.observable_data["robot_current"] = [data["current"] for data in sorted_data]
            self.observable_data["robot_motor_error"] = [
                data["motor_error"] for data in sorted_data
            ]
            self.observable_data["robot_motor_mode"] = [data["motor_mode"] for data in sorted_data]
            self.observable_data["robot_esp_errors"] = [data["esp_errors"] for data in sorted_data]
        else:
            # No data available
            num_modules = len(self.module_ids)
            self.observable_data["robot_motor_torque"] = [0.0] * num_modules
            self.observable_data["robot_temperature"] = [0.0] * num_modules
            self.observable_data["robot_voltage"] = [0.0] * num_modules
            self.observable_data["robot_current"] = [0.0] * num_modules
            self.observable_data["robot_motor_error"] = [0] * num_modules
            self.observable_data["robot_motor_mode"] = [0] * num_modules
            self.observable_data["robot_esp_errors"] = [[0]] * num_modules

        # OptiTrack data
        self.observable_data["optitrack_time"] = self.optitrack_time
        for rigid_body_id, data in self.optitrack_data.items():
            self.observable_data[f"optitrack_rigibody{rigid_body_id}"] = data

        # Send data to dashboard if enabled
        if self.enable_dashboard and hasattr(self, "dashboard_server"):
            try:
                self.dashboard_server.send_data(convert_np_arrays_to_lists(self.observable_data))
            except Exception as e:
                print(f"[ERROR] Failed to send data to dashboard: {e}")

        # Update display
        self.live.update(self._generate_table())

        return self.observable_data

    def _enable_motor(self) -> None:
        """Enable motor control for all modules."""
        self.switch_on = 1
        # Motor enabled timestamp for tracking
        self.motor_enabled_time = time.time()
        self.step_counter = 0

        # Setup calibration commands if configured
        if self.calibration_modes is not None:
            for i, module_id in enumerate(self.module_ids):
                # Send calibration command multiple times for reliability
                self.calibration_command_buffer[module_id] = [self.calibration_modes[i]] * 20

    def _disable_motor(self) -> None:
        """Disable motor control for all modules."""
        self.switch_on = 0

    def log_raw_data(self) -> Dict[str, Any]:
        """Prepare data for logging with source information.

        Returns:
            Dictionary of data ready for logging
        """
        data_to_log = {}

        # Add source prefixes to observable data
        for key, value in self.observable_data.items():
            if key in self.data_source:
                log_key = f"{self.data_source[key]}/{key}"
            else:
                log_key = key
            data_to_log[log_key] = value

        # Add action data
        if hasattr(self, "actions"):
            data_to_log["action"] = self.actions

        return data_to_log
