"""
Rich Dashboard Module for Capybarish.

This module provides a modular, reusable dashboard component for real-time
monitoring of robot status, motor data, and system performance. It uses
the Rich library for beautiful terminal-based visualization.

Features:
- Real-time status table with auto-refresh
- Customizable columns and data fields
- Support for multiple robots/modules
- Color-coded status indicators
- Thread-safe updates
- Clean shutdown handling

Example Usage:
    ```python
    from capybarish.dashboard import RichDashboard, DashboardConfig
    
    # Create dashboard with custom config
    config = DashboardConfig(
        title="Motor Controller Dashboard",
        refresh_rate=20,
        show_performance=True,
    )
    dashboard = RichDashboard(config)
    
    # Start the dashboard
    dashboard.start()
    
    # Update data in your control loop
    dashboard.update_device("192.168.1.100", {
        "position": 1.5,
        "velocity": 0.2,
        "torque": 0.5,
        "status": "running",
    })
    
    # Stop when done
    dashboard.stop()
    ```

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>
Licensed under the Apache License, Version 2.0
"""

import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import numpy as np

from rich import box
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.layout import Layout
from rich.style import Style

# Import error decoder (with lazy import to avoid circular deps)
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .error_decoder import ErrorDecoder


def _force_restore_terminal() -> None:
    """Force restore terminal to normal state.
    
    This is a module-level function that can be called from signal handlers
    and atexit handlers to ensure the cursor is always restored.
    """
    import sys
    import os
    import subprocess
    
    # Method 1: Direct ANSI escape codes to stdout
    try:
        sys.stdout.write('\033[?25h')  # Show cursor
        sys.stdout.write('\033[0m')    # Reset attributes
        sys.stdout.flush()
    except Exception:
        pass
    
    # Method 2: Direct ANSI escape codes to stderr (backup)
    try:
        sys.stderr.write('\033[?25h')
        sys.stderr.write('\033[0m')
        sys.stderr.flush()
    except Exception:
        pass
    
    # Method 3: Write directly to terminal device (most reliable)
    try:
        with open('/dev/tty', 'w') as tty:
            tty.write('\033[?25h')
            tty.write('\033[0m')
            tty.flush()
    except Exception:
        pass
    
    # Method 4: Use tput cnorm (show cursor)
    try:
        subprocess.run(['tput', 'cnorm'], stderr=subprocess.DEVNULL, timeout=1)
    except Exception:
        pass
    
    # Method 5: Use stty sane to reset terminal (last resort)
    try:
        subprocess.run(['stty', 'sane'], stderr=subprocess.DEVNULL, timeout=1)
    except Exception:
        pass


# Register module-level cleanup that runs on ANY exit
import atexit as _atexit
import sys as _sys

_atexit.register(_force_restore_terminal)

# Also install a custom excepthook to restore terminal on unhandled exceptions
_original_excepthook = _sys.excepthook

def _terminal_safe_excepthook(exc_type, exc_value, exc_tb):
    """Restore terminal before showing exception."""
    _force_restore_terminal()
    _original_excepthook(exc_type, exc_value, exc_tb)

_sys.excepthook = _terminal_safe_excepthook


class DeviceStatus(Enum):
    """Status enum for connected devices."""
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    PENDING = "pending"
    ERROR = "error"
    UNKNOWN = "unknown"


@dataclass
class DeviceInfo:
    """Information about a connected device."""
    address: str
    port: int = 0
    name: str = ""
    status: DeviceStatus = DeviceStatus.UNKNOWN
    last_seen: float = 0.0
    recv_count: int = 0
    send_count: int = 0
    
    # Motor/sensor data
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    voltage: float = 0.0
    current: float = 0.0
    temperature: float = 0.0
    mode: str = "unknown"
    error: str = ""
    distance: float = -1.0  # Goal distance (-1 = not available)
    
    # Motor status (new fields)
    motor_error: int = 0      # Motor error flags (6 bits)
    motor_mode: int = 0       # Motor mode (0=Reset/Off, 1=Calibration, 2=Active/On)
    driver_error: int = 0     # Driver chip error/fault state
    
    # Custom data fields
    custom_data: Dict[str, Any] = field(default_factory=dict)
    
    def update(self, **kwargs) -> None:
        """Update device info with provided values."""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
            else:
                self.custom_data[key] = value
        self.last_seen = time.time()


@dataclass
class ColumnConfig:
    """Configuration for a dashboard column."""
    name: str
    key: str
    width: Optional[int] = None
    justify: str = "center"
    format_func: Optional[Callable[[Any], str]] = None
    style_func: Optional[Callable[[Any, DeviceInfo], str]] = None


@dataclass
class DashboardConfig:
    """Configuration for the Rich Dashboard."""
    title: str = "Capybarish Dashboard"
    refresh_rate: int = 20  # Hz
    show_performance: bool = False  # Performance now shown in header
    show_header: bool = True
    show_footer: bool = True
    border_style: str = "blue"
    active_border_style: str = "green"
    inactive_border_style: str = "yellow"
    timeout_sec: float = 2.0  # Device timeout
    
    # Column configuration
    columns: Optional[List[ColumnConfig]] = None
    
    def __post_init__(self):
        if self.columns is None:
            self.columns = self._default_columns()
    
    @staticmethod
    def _default_columns() -> List[ColumnConfig]:
        """Get default column configuration."""
        return [
            ColumnConfig("Device", "name", justify="left"),
            ColumnConfig("Address", "address"),
            ColumnConfig("Status", "status", style_func=DashboardConfig._status_style),
            ColumnConfig("Position", "position", format_func=lambda x: f"{x:+.3f}"),
            ColumnConfig("Velocity", "velocity", format_func=lambda x: f"{x:+.3f}"),
            ColumnConfig("Torque", "torque", format_func=lambda x: f"{x:+.3f}"),
            ColumnConfig("RX/TX", "_rx_tx"),
            ColumnConfig("Last Seen", "last_seen", format_func=DashboardConfig._format_last_seen),
        ]
    
    @staticmethod
    def _status_style(value: Any, device: DeviceInfo) -> str:
        """Get style for status column."""
        if isinstance(value, DeviceStatus):
            styles = {
                DeviceStatus.CONNECTED: "green",
                DeviceStatus.DISCONNECTED: "red",
                DeviceStatus.PENDING: "yellow",
                DeviceStatus.ERROR: "red bold",
                DeviceStatus.UNKNOWN: "dim",
            }
            return styles.get(value, "")
        return ""
    
    @staticmethod
    def _format_last_seen(value: float) -> str:
        """Format last seen timestamp."""
        if value <= 0:
            return "N/A"
        elapsed = time.time() - value
        if elapsed < 1:
            return f"{elapsed*1000:.0f}ms"
        return f"{elapsed:.1f}s"


class RichDashboard:
    """Rich-based terminal dashboard for real-time monitoring.
    
    This class provides a modular, thread-safe dashboard for monitoring
    robot status, motor data, and system performance in real-time.
    """
    
    def __init__(self, config: Optional[DashboardConfig] = None):
        """Initialize the dashboard.
        
        Args:
            config: Dashboard configuration. Uses defaults if None.
        """
        self.config = config or DashboardConfig()
        self.console = Console()
        
        # Device tracking
        self._devices: Dict[str, DeviceInfo] = {}
        self._devices_lock = threading.Lock()
        
        # Performance metrics
        self._start_time = time.time()
        self._update_count = 0
        self._last_update_time = time.time()
        self._loop_dt = 0.0
        self._compute_time = 0.0
        
        # Control state
        self._switch_on = False
        self._mission_type = "default"
        self._custom_status: Dict[str, str] = {}
        
        # Rich Live display
        self._live: Optional[Live] = None
        self._running = False
        self._update_thread: Optional[threading.Thread] = None
    
    def start(self) -> None:
        """Start the dashboard display."""
        if self._running:
            return
        
        self._running = True
        self._start_time = time.time()
        
        # Register signal handlers for clean shutdown
        import signal
        import atexit
        
        self._original_sigint = signal.getsignal(signal.SIGINT)
        self._original_sigterm = signal.getsignal(signal.SIGTERM)
        
        def signal_handler(signum, frame):
            """Handle Ctrl+C and SIGTERM gracefully."""
            # IMMEDIATELY restore terminal before anything else
            _force_restore_terminal()
            self.stop()
            # Re-raise the signal to allow the program to exit
            if signum == signal.SIGINT:
                raise KeyboardInterrupt
            elif signum == signal.SIGTERM:
                import sys
                sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Register atexit handler for extra safety
        atexit.register(_force_restore_terminal)
        
        self._live = Live(
            self._generate_display(),
            refresh_per_second=self.config.refresh_rate,
            console=self.console,
        )
        self._live.__enter__()
    
    def stop(self) -> None:
        """Stop the dashboard display."""
        if not self._running:
            return
        
        self._running = False
        if self._live:
            try:
                self._live.__exit__(None, None, None)
            except Exception:
                pass
            self._live = None
        
        # Restore terminal state (cursor, etc.)
        _force_restore_terminal()
        
        # Restore original signal handlers
        import signal
        if hasattr(self, '_original_sigint') and self._original_sigint:
            try:
                signal.signal(signal.SIGINT, self._original_sigint)
            except Exception:
                pass
        if hasattr(self, '_original_sigterm') and self._original_sigterm:
            try:
                signal.signal(signal.SIGTERM, self._original_sigterm)
            except Exception:
                pass
    
    def _restore_terminal(self) -> None:
        """Restore terminal to normal state (show cursor, reset attributes)."""
        _force_restore_terminal()
    
    def update(self) -> None:
        """Update the dashboard display."""
        if self._live and self._running:
            try:
                self._live.update(self._generate_display())
                self._update_count += 1
            except Exception:
                pass
    
    def update_device(
        self,
        address: str,
        data: Optional[Dict[str, Any]] = None,
        increment_recv: bool = True,
        **kwargs
    ) -> None:
        """Update device information.
        
        Args:
            address: Device IP address or identifier
            data: Dictionary of data to update
            increment_recv: Whether to increment receive counter (default True)
            **kwargs: Additional fields to update
        """
        with self._devices_lock:
            if address not in self._devices:
                self._devices[address] = DeviceInfo(
                    address=address,
                    name=kwargs.get('name', f"Device_{len(self._devices)+1}"),
                )
            
            device = self._devices[address]
            
            # Update from dict
            if data:
                device.update(**data)
            
            # Update from kwargs
            if kwargs:
                device.update(**kwargs)
            
            # Auto-set status based on last_seen
            if device.status == DeviceStatus.UNKNOWN:
                device.status = DeviceStatus.CONNECTED
            
            # Increment receive counter
            if increment_recv:
                device.recv_count += 1
    
    def remove_device(self, address: str) -> None:
        """Remove a device from tracking."""
        with self._devices_lock:
            self._devices.pop(address, None)
    
    def get_device(self, address: str) -> Optional[DeviceInfo]:
        """Get device information."""
        with self._devices_lock:
            return self._devices.get(address)
    
    def get_all_devices(self) -> Dict[str, DeviceInfo]:
        """Get all tracked devices."""
        with self._devices_lock:
            return dict(self._devices)
    
    def get_active_devices(self) -> Dict[str, DeviceInfo]:
        """Get only active devices (seen within timeout)."""
        now = time.time()
        with self._devices_lock:
            return {
                addr: dev for addr, dev in self._devices.items()
                if now - dev.last_seen < self.config.timeout_sec
            }
    
    def set_switch(self, on: bool) -> None:
        """Set the global switch state."""
        self._switch_on = on
    
    def set_mission(self, mission_type: str) -> None:
        """Set the current mission type."""
        self._mission_type = mission_type
    
    def set_status(self, key: str, value: str) -> None:
        """Set a custom status field."""
        self._custom_status[key] = value
    
    def set_performance(self, loop_dt: float = 0.0, compute_time: float = 0.0) -> None:
        """Update performance metrics."""
        self._loop_dt = loop_dt
        self._compute_time = compute_time
        self._last_update_time = time.time()
    
    def _generate_display(self) -> Panel:
        """Generate the full dashboard display."""
        # Create main table
        table = self._generate_table()
        
        # Create header text
        runtime = time.time() - self._start_time
        active_count = len(self.get_active_devices())
        total_count = len(self._devices)
        
        switch_str = "[green]ON[/green]" if self._switch_on else "[red]OFF[/red]"
        
        header = Text()
        # header.append(f"🤖 {self.config.title}", style="bold blue")
        header.append(f"Runtime: {runtime:.1f}s", style="dim")
        header.append(f" | Switch: {switch_str}")
        header.append(f" | Mission: {self._mission_type}", style="cyan")
        header.append(f" | Devices: {active_count}/{total_count}", style="yellow")
        
        # Add performance metrics to header
        if self._loop_dt > 0:
            freq = 1.0 / self._loop_dt
            header.append(f" | dt: {self._loop_dt*1000:.1f}ms @ {freq:.0f}Hz", style="dim")
        
        # Add custom status
        for key, value in self._custom_status.items():
            header.append(f" | {key}: {value}", style="magenta")
        
        # Determine border style
        if active_count > 0 and self._switch_on:
            border_style = self.config.active_border_style
        elif active_count > 0:
            border_style = self.config.border_style
        else:
            border_style = self.config.inactive_border_style
        
        # Create panel
        panel = Panel(
            table,
            title=str(header),
            border_style=border_style,
            padding=(0, 1),
        )
        
        return panel
    
    def _generate_table(self) -> Table:
        """Generate the status table."""
        table = Table(show_header=self.config.show_header, expand=True)
        
        # Add columns
        for col in self.config.columns:
            table.add_column(
                col.name,
                justify=col.justify,
                width=col.width,
            )
        
        # Add device rows
        devices = self.get_all_devices()
        now = time.time()
        
        if not devices:
            # Show placeholder when no devices
            table.add_row(
                *["⏳ Waiting for devices..." if i == 0 else "" 
                  for i in range(len(self.config.columns))]
            )
        else:
            for address, device in sorted(devices.items()):
                row = self._generate_row(device, now)
                table.add_row(*row)
        
        # Add performance row if enabled
        if self.config.show_performance:
            table.add_section()
            perf_row = self._generate_performance_row()
            table.add_row(*perf_row)
        
        return table
    
    def _generate_row(self, device: DeviceInfo, now: float) -> List[str]:
        """Generate a table row for a device."""
        row = []
        
        # Check if device is inactive
        is_inactive = (now - device.last_seen) > self.config.timeout_sec
        
        for col in self.config.columns:
            # Handle special keys
            if col.key == "_rx_tx":
                value = f"{device.recv_count}/{device.send_count}"
            elif col.key == "status":
                if is_inactive and device.status == DeviceStatus.CONNECTED:
                    value = DeviceStatus.PENDING
                else:
                    value = device.status
            elif hasattr(device, col.key):
                value = getattr(device, col.key)
            elif col.key in device.custom_data:
                value = device.custom_data[col.key]
            else:
                value = "N/A"
            
            # Format value
            if col.format_func:
                try:
                    formatted = col.format_func(value)
                except (TypeError, ValueError):
                    formatted = str(value)
            elif isinstance(value, DeviceStatus):
                formatted = value.value.upper()
            elif isinstance(value, float):
                formatted = f"{value:.3f}"
            else:
                formatted = str(value)
            
            # Apply style
            if col.style_func:
                style = col.style_func(value, device)
                if style:
                    formatted = f"[{style}]{formatted}[/{style}]"
            
            # Dim if inactive
            if is_inactive:
                formatted = f"[dim]{formatted}[/dim]"
            
            row.append(formatted)
        
        return row
    
    def _generate_performance_row(self) -> List[str]:
        """Generate the performance metrics row."""
        cols = len(self.config.columns)
        row = [""] * cols
        
        if cols >= 2:
            row[0] = "[bold]Loop dt"
            row[1] = f"{self._loop_dt*1000:.1f}ms"
        if cols >= 4:
            row[2] = "[bold]Compute"
            row[3] = f"{self._compute_time*1000:.1f}ms"
        if cols >= 6:
            freq = 1.0 / self._loop_dt if self._loop_dt > 0 else 0
            row[4] = "[bold]Freq"
            row[5] = f"{freq:.1f}Hz"
        
        return row
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        return False


class MotorDashboard(RichDashboard):
    """Specialized dashboard for motor control applications.
    
    This extends RichDashboard with motor-specific columns and formatting.
    """
    
    def __init__(self, config: Optional[DashboardConfig] = None):
        """Initialize motor dashboard with motor-specific columns."""
        if config is None:
            config = DashboardConfig(
                title="Motor Controller Dashboard",
                columns=self._motor_columns(),
            )
        else:
            # Override columns with motor-specific columns
            config.columns = self._motor_columns()
        super().__init__(config)
    
    @staticmethod
    def _motor_columns() -> List[ColumnConfig]:
        """Get motor-specific column configuration."""
        return [
            ColumnConfig("Module", "name", justify="left"),
            ColumnConfig("Address", "address"),
            ColumnConfig("Status", "status", style_func=DashboardConfig._status_style),
            ColumnConfig("Mode", "mode", style_func=MotorDashboard._mode_style),
            ColumnConfig("Position", "position", format_func=lambda x: f"{x:+.3f}"),
            ColumnConfig("Velocity", "velocity", format_func=lambda x: f"{x:+.3f}"),
            ColumnConfig("Torque", "torque", format_func=lambda x: f"{x:+.3f}"),
            ColumnConfig("Voltage", "voltage", format_func=lambda x: f"{x:.1f}V"),
            ColumnConfig("Current", "current", format_func=lambda x: f"{x:.2f}A"),
            ColumnConfig("Distance", "distance", format_func=lambda x: f"{x:.3f}m" if x >= 0 else "-"),
            ColumnConfig("Switch", "switch", style_func=MotorDashboard._switch_style),
            ColumnConfig("Error", "error", style_func=MotorDashboard._error_style),
        ]
    
    @staticmethod
    def _mode_style(value: Any, device: DeviceInfo) -> str:
        """Style for motor mode column."""
        if isinstance(value, str):
            if value.lower() in ("running", "enabled", "on"):
                return "green"
            elif value.lower() in ("disabled", "off", "idle"):
                return "yellow"
            elif value.lower() in ("error", "fault"):
                return "red bold"
        return ""
    
    @staticmethod
    def _switch_style(value: Any, device: DeviceInfo) -> str:
        """Style for switch column."""
        if value in (True, 1, "on", "ON"):
            return "green"
        return "red"
    
    @staticmethod
    def _error_style(value: Any, device: DeviceInfo) -> str:
        """Style for error column."""
        if value and value not in ("None", "none", "", "OK", "ok"):
            return "red bold"
        return "green"
    
    def update_motor(
        self,
        address: str,
        position: float = 0.0,
        velocity: float = 0.0,
        torque: float = 0.0,
        voltage: float = 0.0,
        current: float = 0.0,
        mode: str = "unknown",
        switch: bool = False,
        error: str = "",
        distance: float = -1.0,
        motor_error: int = 0,
        motor_mode: int = 0,
        driver_error: int = 0,
        **kwargs
    ) -> None:
        """Update motor-specific data for a device.
        
        Args:
            address: Motor/device address
            position: Motor position
            velocity: Motor velocity
            torque: Motor torque
            voltage: Supply voltage
            current: Motor current
            mode: Motor mode string
            switch: Switch state
            error: Error message if any
            distance: Goal distance in meters (-1 = not available)
            motor_error: Motor error flags (6 bits)
            motor_mode: Motor mode (0=Reset/Off, 1=Calibration, 2=Active/On)
            driver_error: Driver chip error/fault state
            **kwargs: Additional custom data
        """
        self.update_device(
            address,
            position=position,
            velocity=velocity,
            torque=torque,
            voltage=voltage,
            current=current,
            mode=mode,
            switch=switch,
            error=error,
            distance=distance,
            motor_error=motor_error,
            motor_mode=motor_mode,
            driver_error=driver_error,
            **kwargs
        )


# =============================================================================
# Enhanced RL Dashboard for Real Robot Debugging
# =============================================================================

@dataclass
class RLDashboardConfig:
    """Configuration for the RL-enhanced dashboard."""
    title: str = "RL Robot Dashboard"
    refresh_rate: int = 20
    timeout_sec: float = 2.0
    show_observations: bool = True
    show_actions: bool = True
    show_rewards: bool = True
    max_obs_components: int = 12
    max_log_lines: int = 8
    history_steps: int = 3
    
    # Visual themes
    theme: str = "cyber"  # cyber, matrix, minimal, retro
    
    # Display mode options
    fullscreen: bool = False # True       # If False, dashboard doesn't take over terminal
    capture_prints: bool = False # True   # If True, captures print() calls to log panel
    dashboard_height: int = 30    # Height in lines when fullscreen=False


class RLDashboard:
    """Enhanced dashboard for RL real-robot debugging.
    
    Features a multi-panel layout with:
    - Motor status with animated indicators
    - Observation component breakdown with history
    - Action visualization with bar graphs
    - System performance metrics
    - Log/messages panel for debug output
    - Cool cyberpunk-inspired aesthetics
    """
    
    # Color themes
    THEMES = {
        "cyber": {
            "primary": "cyan",
            "secondary": "magenta", 
            "accent": "bright_green",
            "warning": "yellow",
            "error": "red",
            "dim": "bright_black",
            "bg": "on dark_blue",
            "border": "cyan",
            "title": "bold bright_cyan",
        },
        "matrix": {
            "primary": "green",
            "secondary": "bright_green",
            "accent": "white",
            "warning": "yellow",
            "error": "red",
            "dim": "dark_green",
            "bg": "",
            "border": "green",
            "title": "bold bright_green",
        },
        "minimal": {
            "primary": "white",
            "secondary": "bright_white",
            "accent": "cyan",
            "warning": "yellow",
            "error": "red",
            "dim": "bright_black",
            "bg": "",
            "border": "white",
            "title": "bold white",
        },
        "retro": {
            "primary": "bright_yellow",
            "secondary": "bright_magenta",
            "accent": "bright_cyan",
            "warning": "yellow",
            "error": "red",
            "dim": "bright_black",
            "bg": "",
            "border": "bright_yellow",
            "title": "bold bright_yellow",
        },
    }
    
    def __init__(self, config: Optional[RLDashboardConfig] = None, error_decoder: Optional["ErrorDecoder"] = None):
        """Initialize the RL dashboard.
        
        Args:
            config: Dashboard configuration. Uses defaults if None.
            error_decoder: Optional error decoder for motor/driver errors.
                          If None, uses default decoder that shows hex codes.
                          Use `capybarish.devices.CybergearErrorDecoder()` for
                          Cybergear motors, or implement your own.
        """
        self.config = config or RLDashboardConfig()
        self.console = Console()
        self.theme = self.THEMES.get(self.config.theme, self.THEMES["cyber"])
        
        # Error decoder for motor/driver errors
        if error_decoder is None:
            from .error_decoder import default_decoder
            self._error_decoder = default_decoder
        else:
            self._error_decoder = error_decoder
        
        # Motor/device data
        self._devices: Dict[str, DeviceInfo] = {}
        self._devices_lock = threading.Lock()
        
        # RL State tracking
        self._observation_components: Dict[str, np.ndarray] = {}
        self._observation_history: Dict[str, List[np.ndarray]] = {}
        self._used_obs_components: set = set()  # Components actually used in policy obs
        self._command_obs_components: set = set()  # Components that are command-related
        self._current_action: Optional[np.ndarray] = None
        self._action_history: List[np.ndarray] = []
        self._last_reward: float = 0.0
        self._episode_reward: float = 0.0
        self._reward_history: List[float] = []
        
        # Performance metrics
        self._start_time = time.time()
        self._step_count = 0
        self._loop_dt = 0.0
        self._compute_time = 0.0
        self._cmd_count = 0
        self._fb_count = 0
        
        # Control state
        self._switch_on = False
        self._motor_enabled = False
        self._episode_count = 0
        self._custom_status: Dict[str, str] = {}
        
        # Module connection tracking
        self._expected_modules: List[int] = []
        self._connected_modules: set = set()
        self._waiting_for_modules: bool = False
        self._env_ready: bool = False  # Environment ready status
        
        # Command tracking for RL
        self._commands: Optional[np.ndarray] = None
        self._command_names: List[str] = []
        self._selected_command_idx: int = 0
        self._onehot_mode: bool = False
        self._keyboard_command_mode: bool = False
        
        # Model tracking for multi-model mode
        self._model_names: List[str] = []
        self._model_obs_dims: List[int] = []
        self._current_model_idx: int = 0
        self._num_models: int = 0
        
        # Log/message buffer for debug output
        self._log_messages: List[Tuple[float, str, str]] = []  # (timestamp, level, message)
        self._max_log_lines = config.max_log_lines if config else 8
        
        # Training progress tracking (for RL training on real robot)
        self._training_enabled: bool = False
        self._training_timesteps: int = 0
        self._training_total_timesteps: int = 0
        self._training_episodes: int = 0
        self._training_avg_reward: float = 0.0
        self._training_last_reward: float = 0.0
        self._training_recent_rewards: List[float] = []
        
        # Custom gauge/indicator panels (general purpose)
        # Each gauge is a dict with: value, min_val, max_val, center_val, unit, title,
        # left_label, right_label, direction_threshold, history, ready, extra_info
        self._gauges: Dict[str, Dict[str, Any]] = {}
        
        # Display state
        self._live: Optional[Live] = None
        self._running = False
        
        # Print capture state
        self._original_print = None
        self._capturing_prints = False
        
        # Animation state
        self._frame = 0
        self._spinner_chars = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"
    
    def start(self) -> None:
        """Start the dashboard display."""
        if self._running:
            return
        self._running = True
        self._start_time = time.time()
        
        # Setup print capturing if enabled
        if self.config.capture_prints:
            self._start_print_capture()
        
        # Register signal handlers for clean shutdown
        import signal
        import atexit
        
        self._original_sigint = signal.getsignal(signal.SIGINT)
        self._original_sigterm = signal.getsignal(signal.SIGTERM)
        
        def signal_handler(signum, frame):
            """Handle Ctrl+C and SIGTERM gracefully."""
            # IMMEDIATELY restore terminal before anything else
            _force_restore_terminal()
            self.stop()
            # Re-raise the signal to allow the program to exit
            if signum == signal.SIGINT:
                raise KeyboardInterrupt
            elif signum == signal.SIGTERM:
                import sys
                sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Register atexit handler for extra safety
        atexit.register(_force_restore_terminal)
        
        # Create Live display with appropriate settings
        if self.config.fullscreen:
            # Fullscreen mode - takes over terminal
            self._live = Live(
                self._generate_display(),
                refresh_per_second=self.config.refresh_rate,
                console=self.console,
                screen=False,  # Don't use alternate screen buffer
                transient=False,
            )
        else:
            # Non-fullscreen mode - dashboard updates in place, prints go below
            self._live = Live(
                self._generate_compact_display(),
                refresh_per_second=self.config.refresh_rate,
                console=self.console,
                screen=False,
                transient=False,
                vertical_overflow="visible",
            )
        
        self._live.__enter__()
    
    def stop(self) -> None:
        """Stop the dashboard display."""
        if not self._running:
            return
        
        # Restore print function first
        if self._capturing_prints:
            self._stop_print_capture()
        
        self._running = False
        if self._live:
            try:
                self._live.__exit__(None, None, None)
            except Exception:
                pass
        
        # Restore terminal state (cursor, etc.)
        _force_restore_terminal()
        
        # Restore original signal handlers
        import signal
        if hasattr(self, '_original_sigint') and self._original_sigint:
            try:
                signal.signal(signal.SIGINT, self._original_sigint)
            except Exception:
                pass
        if hasattr(self, '_original_sigterm') and self._original_sigterm:
            try:
                signal.signal(signal.SIGTERM, self._original_sigterm)
            except Exception:
                pass
    
    def _restore_terminal(self) -> None:
        """Restore terminal to normal state (show cursor, reset attributes)."""
        _force_restore_terminal()
    
    def _start_print_capture(self) -> None:
        """Start capturing print() calls to the dashboard log."""
        import builtins
        self._original_print = builtins.print
        self._capturing_prints = True
        
        dashboard = self  # Capture reference for closure
        
        def captured_print(*args, **kwargs):
            # Build message from args
            message = " ".join(str(arg) for arg in args)
            
            # Log to dashboard
            dashboard.log(message, "info")
            
            # If not fullscreen, also print to terminal
            if not dashboard.config.fullscreen and dashboard._original_print:
                dashboard._original_print(*args, **kwargs)
        
        builtins.print = captured_print
    
    def _stop_print_capture(self) -> None:
        """Stop capturing print() calls."""
        import builtins
        if self._original_print is not None:
            builtins.print = self._original_print
            self._original_print = None
        self._capturing_prints = False
        self._live = None
    
    def update(self) -> None:
        """Update the dashboard display."""
        if self._live and self._running:
            try:
                self._frame += 1
                if self.config.fullscreen:
                    self._live.update(self._generate_display())
                else:
                    self._live.update(self._generate_compact_display())
            except Exception:
                pass
    
    def _generate_compact_display(self) -> Panel:
        """Generate a compact single-panel display for non-fullscreen mode.
        
        This is used when fullscreen=False, allowing print statements to appear
        below the dashboard.
        """
        from rich.columns import Columns
        
        # Build a compact status line
        runtime = time.time() - self._start_time
        spinner = self._spinner_chars[self._frame % len(self._spinner_chars)]
        device_count = len(self._devices)
        active_count = sum(1 for d in self._devices.values() if time.time() - d.last_seen < self.config.timeout_sec)
        
        # Header line
        header = Text()
        header.append(f"{spinner} ", style=self.theme['accent'])
        header.append(self.config.title, style=self.theme['title'])
        header.append(" │ ", style=self.theme['dim'])
        if self._motor_enabled:
            header.append("● ", style=self.theme['accent'])
            header.append("ON", style=self.theme['accent'])
        else:
            header.append("○ ", style=self.theme['error'])
            header.append("OFF", style=self.theme['error'])
        header.append(f" │ Dev:{active_count}/{device_count}", style=self.theme['dim'])
        header.append(f" │ T:{runtime:.0f}s", style=self.theme['dim'])
        header.append(f" │ Ep:{self._episode_count} St:{self._step_count}", style=self.theme['secondary'])
        
        # Show module connection progress
        if self._expected_modules:
            connected = len(self._connected_modules)
            total = len(self._expected_modules)
            header.append(f" │ ", style=self.theme['dim'])
            if connected < total:
                header.append("⏳ ", style=self.theme['warning'])
                header.append(f"{connected}/{total}", style=self.theme['warning'])
            else:
                header.append("✓ ", style=self.theme['accent'])
                header.append(f"{connected}/{total}", style=self.theme['accent'])
        
        # Show env ready status in compact mode
        header.append(f" │ ", style=self.theme['dim'])
        if self._env_ready:
            header.append("✓ ", style=self.theme['accent'])
            header.append("Rdy", style=self.theme['accent'])
        else:
            header.append("⏳ ", style=self.theme['warning'])
            header.append("Wait", style=self.theme['warning'])
        
        # Show model info in multi-model mode
        if self._num_models > 1:
            header.append(f" │ ", style=self.theme['dim'])
            header.append("🤖", style=self.theme['accent'])
            current_name = self._model_names[self._current_model_idx] if self._current_model_idx < len(self._model_names) else "?"
            # Truncate name for compact display
            display_name = current_name[:12] if len(current_name) > 12 else current_name
            header.append(f"[{self._current_model_idx + 1}/{self._num_models}]", style=self.theme['secondary'])
            header.append(f" {display_name}", style=self.theme['accent'])
        
        # Motor status line with mode indicators
        motor_line = Text()
        if self._devices:
            mode_chars = {0: "○", 1: "◐", 2: "●"}  # OFF, CAL, ON
            for addr, dev in list(self._devices.items())[:4]:  # Show max 4 motors
                is_active = (time.time() - dev.last_seen) < self.config.timeout_sec
                mode_char = mode_chars.get(dev.motor_mode, "?")
                if dev.motor_mode == 2:
                    mode_style = self.theme['accent']
                elif dev.motor_mode == 1:
                    mode_style = self.theme['warning']
                else:
                    mode_style = self.theme['error']
                motor_line.append(mode_char, style=mode_style if is_active else self.theme['dim'])
                motor_line.append(" ", style=self.theme['dim'])  # Add space after mode char
                motor_line.append(f"{dev.name}:", style=self.theme['dim'])
                style = self.theme['accent'] if is_active else self.theme['dim']
                motor_line.append(f"{dev.position:+.2f} ", style=style)
        else:
            motor_line.append("Waiting for motors...", style=self.theme['dim'])
        
        # Action line
        action_line = Text()
        action_line.append("Act: ", style=self.theme['dim'])
        if self._current_action is not None:
            for i, val in enumerate(self._current_action[:6]):
                action_line.append(f"{val:+.2f} ", style=self.theme['secondary'])
        else:
            action_line.append("—", style=self.theme['dim'])
        
        # Command line
        cmd_line = Text()
        cmd_line.append("Cmd", style=self.theme['dim'])
        if self._keyboard_command_mode:
            cmd_line.append("[K]", style=self.theme['warning'])
        cmd_line.append(": ", style=self.theme['dim'])
        if self._commands is not None and len(self._commands) > 0:
            if self._onehot_mode:
                active_idx = int(np.argmax(self._commands))
                name = self._command_names[active_idx] if active_idx < len(self._command_names) else f"cmd_{active_idx}"
                cmd_line.append(f"[{active_idx}]", style=self.theme['accent'])
                cmd_line.append(f" {name}", style=self.theme['secondary'])
            else:
                for i, val in enumerate(self._commands[:4]):  # Show max 4
                    if i == self._selected_command_idx:
                        cmd_line.append(f"►{val:+.1f} ", style=self.theme['accent'])
                    else:
                        cmd_line.append(f"{val:+.1f} ", style=self.theme['secondary'])
        else:
            cmd_line.append("—", style=self.theme['dim'])
        
        # Reward line
        reward_line = Text()
        reward_line.append(f"R:{self._last_reward:+.3f} ", style=self.theme['accent'])
        reward_line.append(f"Σ:{self._episode_reward:+.2f} ", style=self.theme['secondary'])
        reward_line.append(f"dt:{self._loop_dt*1000:.1f}ms", style=self.theme['dim'])
        
        # Gauge lines (if any gauges defined)
        gauge_line = Text()
        if self._gauges:
            for i, (name, gauge) in enumerate(list(self._gauges.items())[:2]):  # Max 2 in compact
                if i > 0:
                    gauge_line.append(" │ ", style=self.theme['dim'])
                
                value = gauge['value']
                center_val = gauge['center_val']
                min_val = gauge['min_val']
                max_val = gauge['max_val']
                threshold = gauge['direction_threshold']
                unit = gauge['unit']
                
                # Title emoji if present
                title = gauge['title']
                if title and len(title) > 0 and ord(title[0]) > 127:
                    gauge_line.append(f"{title[0]} ", style=self.theme['accent'])
                
                if not gauge['ready']:
                    gauge_line.append("...", style=self.theme['dim'])
                else:
                    # Mini gauge bar
                    bar_width = 12
                    range_val = max_val - min_val
                    if range_val > 0:
                        normalized = (value - min_val) / range_val
                        pos = int(np.clip(normalized, 0, 1) * (bar_width - 1))
                        center_normalized = (center_val - min_val) / range_val
                        center_pos = int(np.clip(center_normalized, 0, 1) * (bar_width - 1))
                    else:
                        pos = bar_width // 2
                        center_pos = bar_width // 2
                    
                    gauge_line.append("[", style=self.theme['dim'])
                    for j in range(bar_width):
                        if j == center_pos:
                            gauge_line.append("│", style=self.theme['dim'])
                        elif j == pos:
                            gauge_line.append("●", style=self.theme['accent'])
                        elif (j < center_pos and j >= pos and value < center_val) or \
                             (j > center_pos and j <= pos and value > center_val):
                            gauge_line.append("─", style=self.theme['secondary'])
                        else:
                            gauge_line.append("·", style=self.theme['dim'])
                    gauge_line.append("] ", style=self.theme['dim'])
                    
                    # Direction
                    if value > center_val + threshold:
                        direction = gauge['right_direction'].split()[0] if ' ' in gauge['right_direction'] else gauge['right_direction']
                        dir_style = self.theme['warning']
                    elif value < center_val - threshold:
                        direction = gauge['left_direction'].split()[0] if ' ' in gauge['left_direction'] else gauge['left_direction']
                        dir_style = self.theme['warning']
                    else:
                        direction = gauge['center_direction'].split()[0] if ' ' in gauge['center_direction'] else gauge['center_direction']
                        dir_style = self.theme['accent']
                    
                    gauge_line.append(f"{value:+.1f}{unit} ", style=self.theme['accent'])
                    gauge_line.append(direction, style=dir_style)
                    
                    # First extra info
                    for key, val in list(gauge['extra_info'].items())[:1]:
                        gauge_line.append(f" {val}", style=self.theme['dim'])
        
        # Recent log messages (last 2-3 depending on training mode)
        log_line = Text()
        max_logs = 2 if self._training_enabled else 3
        recent_logs = self._log_messages[-max_logs:]
        if recent_logs:
            for _, level, msg in recent_logs:
                if level == "error":
                    log_line.append("✗ ", style=self.theme['error'])
                elif level == "warn":
                    log_line.append("⚠ ", style=self.theme['warning'])
                elif level == "success":
                    log_line.append("✓ ", style=self.theme['accent'])
                else:
                    log_line.append("• ", style=self.theme['dim'])
                log_line.append(msg[:28] + " ", style=self.theme['secondary'])
        
        # Training progress line (if enabled)
        training_line = Text()
        if self._training_enabled:
            # Progress percentage and bar
            progress = self._training_timesteps / max(self._training_total_timesteps, 1)
            progress_pct = progress * 100
            bar_width = 20
            filled = int(progress * bar_width)
            
            training_line.append("🎓 ", style=self.theme['accent'])
            training_line.append(f"{progress_pct:5.1f}% ", style=self.theme['accent'])
            
            # Progress bar
            training_line.append("[", style=self.theme['dim'])
            training_line.append("█" * filled, style=self.theme['accent'])
            training_line.append("░" * (bar_width - filled), style=self.theme['dim'])
            training_line.append("] ", style=self.theme['dim'])
            
            # Stats
            training_line.append(f"{self._training_timesteps:,}/{self._training_total_timesteps:,}", style=self.theme['secondary'])
            training_line.append(f" │ Ep:{self._training_episodes}", style=self.theme['dim'])
            training_line.append(f" │ R̄:{self._training_avg_reward:+.2f}", style=self.theme['accent'])
            if self._training_last_reward != 0:
                training_line.append(f" (last:{self._training_last_reward:+.1f})", style=self.theme['dim'])
        
        # Combine all into a compact panel
        content = Text()
        content.append_text(header)
        content.append("\n")
        
        # Add training progress line prominently if enabled
        if self._training_enabled:
            content.append_text(training_line)
            content.append("\n")
        
        # Add gauge line if any gauges defined
        if self._gauges:
            content.append_text(gauge_line)
            content.append("\n")
        
        content.append_text(motor_line)
        content.append("\n")
        content.append_text(action_line)
        content.append(" │ ", style=self.theme['dim'])
        content.append_text(cmd_line)
        content.append("\n")
        content.append_text(reward_line)
        content.append(" │ ", style=self.theme['dim'])
        content.append_text(log_line)
        
        # Adjust height based on active features
        panel_height = 6
        if self._training_enabled:
            panel_height += 1
        if self._gauges:
            panel_height += 1
        
        return Panel(
            content,
            border_style=self.theme['accent'] if self._training_enabled else self.theme['border'],
            box=box.ROUNDED,
            height=panel_height,
        )
    
    # =========================================================================
    # Motor/Device Updates
    # =========================================================================
    
    def update_motor(
        self,
        address: str,
        name: str = "",
        position: float = 0.0,
        velocity: float = 0.0,
        torque: float = 0.0,
        voltage: float = 0.0,
        current: float = 0.0,
        mode: str = "unknown",
        switch: bool = False,
        error: str = "",
        distance: float = -1.0,
        motor_error: int = 0,
        motor_mode: int = 0,
        driver_error: int = 0,
        **kwargs
    ) -> None:
        """Update motor data."""
        with self._devices_lock:
            if address not in self._devices:
                self._devices[address] = DeviceInfo(address=address, name=name or f"M{len(self._devices)}")
            
            device = self._devices[address]
            device.name = name or device.name
            device.position = position
            device.velocity = velocity
            device.torque = torque
            device.voltage = voltage
            device.current = current
            device.mode = mode
            device.custom_data["switch"] = switch
            device.error = error
            device.distance = distance
            device.motor_error = motor_error
            device.motor_mode = motor_mode
            device.driver_error = driver_error
            device.last_seen = time.time()
            device.recv_count += 1
    
    def set_switch(self, on: bool) -> None:
        """Set global switch state."""
        self._switch_on = on
        self._motor_enabled = on
    
    # =========================================================================
    # RL State Updates
    # =========================================================================
    
    def update_observation(self, components: Dict[str, np.ndarray], used_in_policy: bool = False, is_command: bool = False) -> None:
        """Update observation component data.
        
        Args:
            components: Dict mapping component names to their values
            used_in_policy: If True, marks these components as used in policy observation
            is_command: If True, marks these as command-related observation components
        """
        for name, value in components.items():
            arr = np.atleast_1d(np.asarray(value)).flatten()
            self._observation_components[name] = arr
            
            # Mark as used in policy if specified
            if used_in_policy:
                self._used_obs_components.add(name)
            
            # Mark as command component
            if is_command:
                self._command_obs_components.add(name)
            
            # Track history
            if name not in self._observation_history:
                self._observation_history[name] = []
            self._observation_history[name].append(arr.copy())
            # Keep limited history
            if len(self._observation_history[name]) > self.config.history_steps:
                self._observation_history[name].pop(0)
    
    def mark_obs_used(self, component_names: List[str]) -> None:
        """Mark observation components as used in the policy.
        
        Args:
            component_names: List of component names used in policy observation
        """
        self._used_obs_components.update(component_names)
    
    def update_action(self, action: np.ndarray) -> None:
        """Update current action."""
        self._current_action = np.atleast_1d(np.asarray(action)).flatten()
        self._action_history.append(self._current_action.copy())
        if len(self._action_history) > 10:
            self._action_history.pop(0)
    
    def update_reward(self, reward: float, episode_reward: float = None) -> None:
        """Update reward data."""
        self._last_reward = reward
        if episode_reward is not None:
            self._episode_reward = episode_reward
        self._reward_history.append(reward)
        if len(self._reward_history) > 50:
            self._reward_history.pop(0)
    
    def update_commands(
        self,
        commands: np.ndarray,
        names: List[str],
        selected_idx: int = 0,
        onehot_mode: bool = False,
        keyboard_mode: bool = False,
    ) -> None:
        """Update RL command state for display.
        
        Args:
            commands: Current command values
            names: Command dimension names
            selected_idx: Currently selected command index (for adjustment)
            onehot_mode: Whether commands are in one-hot mode
            keyboard_mode: Whether keyboard command control is active
        """
        self._commands = np.atleast_1d(np.asarray(commands)).flatten()
        self._command_names = list(names)
        self._selected_command_idx = selected_idx
        self._onehot_mode = onehot_mode
        self._keyboard_command_mode = keyboard_mode
    
    def update_models(
        self,
        model_names: List[str],
        current_idx: int = 0,
        obs_dims: List[int] = None,
    ) -> None:
        """Update loaded models state for display.
        
        Args:
            model_names: List of model display names
            current_idx: Index of the currently active model
            obs_dims: Optional list of observation dimensions for each model
        """
        self._model_names = list(model_names) if model_names else []
        self._current_model_idx = current_idx
        self._num_models = len(self._model_names)
        self._model_obs_dims = list(obs_dims) if obs_dims else []
    
    def update_performance(
        self,
        loop_dt: float = 0.0,
        compute_time: float = 0.0,
        cmd_count: int = 0,
        fb_count: int = 0
    ) -> None:
        """Update performance metrics."""
        self._loop_dt = loop_dt
        self._compute_time = compute_time
        self._cmd_count = cmd_count
        self._fb_count = fb_count
    
    def increment_step(self) -> None:
        """Increment step counter."""
        self._step_count += 1
    
    def new_episode(self) -> None:
        """Signal new episode."""
        self._episode_count += 1
        self._episode_reward = 0.0
        self._step_count = 0
    
    def set_status(self, key: str, value: str) -> None:
        """Set custom status field."""
        self._custom_status[key] = value
    
    def set_env_ready(self, ready: bool) -> None:
        """Set the environment ready status."""
        self._env_ready = ready
    
    # =========================================================================
    # Training Progress Tracking
    # =========================================================================
    
    def enable_training_progress(self, total_timesteps: int) -> None:
        """Enable training progress display.
        
        Args:
            total_timesteps: Total timesteps for training
        """
        self._training_enabled = True
        self._training_total_timesteps = total_timesteps
        self._training_timesteps = 0
        self._training_episodes = 0
        self._training_avg_reward = 0.0
        self._training_recent_rewards = []
        self.log_info(f"Training started: {total_timesteps:,} timesteps")
    
    def update_training_progress(
        self,
        timesteps: int,
        episodes: int = None,
        episode_reward: float = None,
        avg_reward: float = None,
    ) -> None:
        """Update training progress display.
        
        Args:
            timesteps: Current total timesteps
            episodes: Current episode count (optional)
            episode_reward: Reward from last completed episode (optional)
            avg_reward: Moving average reward (optional)
        """
        self._training_timesteps = timesteps
        
        if episodes is not None:
            self._training_episodes = episodes
        
        if episode_reward is not None:
            self._training_last_reward = episode_reward
            self._training_recent_rewards.append(episode_reward)
            # Keep last 20 for averaging
            if len(self._training_recent_rewards) > 20:
                self._training_recent_rewards.pop(0)
        
        if avg_reward is not None:
            self._training_avg_reward = avg_reward
        elif self._training_recent_rewards:
            self._training_avg_reward = np.mean(self._training_recent_rewards)
    
    def disable_training_progress(self) -> None:
        """Disable training progress display."""
        self._training_enabled = False
        if self._training_recent_rewards:
            final_avg = np.mean(self._training_recent_rewards)
            self.log_success(f"Training done: {self._training_episodes} eps, avg={final_avg:.2f}")
    
    # =========================================================================
    # Custom Gauge/Indicator Panel (General Purpose)
    # =========================================================================
    
    def add_gauge(
        self,
        name: str,
        title: str = None,
        min_val: float = -90.0,
        max_val: float = 90.0,
        center_val: float = 0.0,
        unit: str = "°",
        left_label: str = "MIN",
        right_label: str = "MAX",
        direction_threshold: float = 5.0,
        left_direction: str = "←",
        right_direction: str = "→",
        center_direction: str = "↑",
        track_history: bool = True,
    ) -> None:
        """Add a custom gauge/indicator panel.
        
        This creates a general-purpose visual gauge that shows:
        - A large numeric value display
        - A visual bar showing the value's position in range
        - Optional direction indicators
        - Optional history sparkline
        - Optional extra info (like distance)
        
        Args:
            name: Unique identifier for this gauge
            title: Panel title (default: name)
            min_val: Minimum value for the gauge range
            max_val: Maximum value for the gauge range
            center_val: Center value for bipolar display (where the bar center is)
            unit: Unit string (e.g., "°", "m", "%")
            left_label: Label for left side of bar
            right_label: Label for right side of bar
            direction_threshold: Threshold from center to show direction change
            left_direction: Direction indicator when value < center - threshold
            right_direction: Direction indicator when value > center + threshold
            center_direction: Direction indicator when value is near center
            track_history: Whether to track and display history sparkline
            
        Example:
            # Bearing gauge (bipolar, centered at 0)
            dashboard.add_gauge(
                "bearing",
                title="🧭 BEARING",
                min_val=-90, max_val=90, center_val=0,
                unit="°",
                left_label="RIGHT", right_label="LEFT",
                left_direction="→", right_direction="←", center_direction="↑",
            )
            
            # Speed gauge (unipolar)
            dashboard.add_gauge(
                "speed",
                title="🚀 SPEED",
                min_val=0, max_val=10, center_val=0,
                unit="m/s",
                left_label="STOP", right_label="MAX",
            )
        """
        self._gauges[name] = {
            'title': title or name,
            'value': center_val,
            'min_val': min_val,
            'max_val': max_val,
            'center_val': center_val,
            'unit': unit,
            'left_label': left_label,
            'right_label': right_label,
            'direction_threshold': direction_threshold,
            'left_direction': left_direction,
            'right_direction': right_direction,
            'center_direction': center_direction,
            'track_history': track_history,
            'history': [],
            'ready': False,
            'extra_info': {},
        }
    
    def update_gauge(
        self,
        name: str,
        value: float,
        ready: bool = True,
        extra_info: Dict[str, Any] = None,
    ) -> None:
        """Update a gauge value.
        
        Args:
            name: Gauge identifier
            value: New value to display
            ready: Whether the gauge is ready/valid (False shows "collecting..." state)
            extra_info: Optional dict of extra info to display (e.g., {"distance": 3.5})
        """
        if name not in self._gauges:
            # Auto-create with defaults if not exists
            self.add_gauge(name)
        
        gauge = self._gauges[name]
        gauge['value'] = value
        gauge['ready'] = ready
        
        if extra_info:
            gauge['extra_info'].update(extra_info)
        
        # Track history
        if gauge['track_history']:
            gauge['history'].append(value)
            if len(gauge['history']) > 100:
                gauge['history'] = gauge['history'][-50:]
    
    def remove_gauge(self, name: str) -> None:
        """Remove a gauge."""
        self._gauges.pop(name, None)
    
    def get_gauge_value(self, name: str) -> Optional[float]:
        """Get the current value of a gauge."""
        if name in self._gauges:
            return self._gauges[name]['value']
        return None
    
    def has_gauges(self) -> bool:
        """Check if any gauges are defined."""
        return len(self._gauges) > 0
    
    # Convenience method for bearing (backward compatibility)
    def update_bearing(
        self,
        bearing_rad: float,
        distance: float = -1.0,
        history_filled: bool = True,
    ) -> None:
        """Update bearing estimation display (convenience wrapper for gauge).
        
        Args:
            bearing_rad: Bearing in radians (positive = left, negative = right)
            distance: Current distance to target (meters)
            history_filled: Whether enough history has been collected
        """
        # Auto-create bearing gauge if not exists
        if "bearing" not in self._gauges:
            self.add_gauge(
                "bearing",
                title="🧭 BEARING",
                min_val=-180, max_val=180, center_val=0,
                unit="°",
                left_label="RIGHT", right_label="LEFT",
                direction_threshold=5.0,
                left_direction="→ RIGHT",
                right_direction="← LEFT",
                center_direction="↑ FWD",
            )
        
        bearing_deg = np.degrees(bearing_rad)
        extra = {}
        if distance > 0:
            extra['distance'] = f"{distance:.2f}m"
        
        self.update_gauge("bearing", bearing_deg, ready=history_filled, extra_info=extra)
    
    def disable_bearing(self) -> None:
        """Disable bearing gauge (convenience wrapper)."""
        self.remove_gauge("bearing")
    
    # =========================================================================
    # Logging and Module Tracking
    # =========================================================================
    
    def log(self, message: str, level: str = "info") -> None:
        """Add a log message to the dashboard.
        
        Args:
            message: The message to log
            level: Log level - "info", "warn", "error", "success"
        """
        self._log_messages.append((time.time(), level, message))
        # Keep only recent messages
        if len(self._log_messages) > self._max_log_lines * 2:
            self._log_messages = self._log_messages[-self._max_log_lines:]
    
    def log_info(self, message: str) -> None:
        """Log an info message."""
        self.log(message, "info")
    
    def log_warn(self, message: str) -> None:
        """Log a warning message."""
        self.log(message, "warn")
    
    def log_error(self, message: str) -> None:
        """Log an error message."""
        self.log(message, "error")
    
    def log_success(self, message: str) -> None:
        """Log a success message."""
        self.log(message, "success")
    
    def set_expected_modules(self, module_ids: List[int], sensor_ids: List[int] = None) -> None:
        """Set the expected module IDs for connection tracking.
        
        Args:
            module_ids: List of expected active module IDs
            sensor_ids: List of expected sensor-only module IDs (optional)
        """
        self._expected_modules = list(module_ids)
        if sensor_ids:
            self._expected_modules.extend(sensor_ids)
        self._waiting_for_modules = True
        self.log_info(f"Expecting modules: {self._expected_modules}")
    
    def module_connected(self, module_id: int, ip: str = "", is_sensor: bool = False) -> None:
        """Notify that a module has connected.
        
        Args:
            module_id: The module ID that connected
            ip: IP address of the module (optional)
            is_sensor: Whether this is a sensor-only module
        """
        self._connected_modules.add(module_id)
        module_type = "sensor" if is_sensor else "active"
        ip_str = f" @ {ip}" if ip else ""
        self.log_success(f"Module {module_id} ({module_type}) connected{ip_str}")
        
        # Check if all modules connected
        if self._expected_modules:
            missing = set(self._expected_modules) - self._connected_modules
            if not missing:
                self._waiting_for_modules = False
                self.log_success(f"All {len(self._expected_modules)} modules connected!")
    
    def get_missing_modules(self) -> List[int]:
        """Get list of modules that haven't connected yet."""
        if not self._expected_modules:
            return []
        return [m for m in self._expected_modules if m not in self._connected_modules]
    
    # =========================================================================
    # Display Generation
    # =========================================================================
    
    def _generate_display(self) -> Layout:
        """Generate the full multi-panel display."""
        layout = Layout()
        
        # Create vertical layout (taller header when training)
        header_size = 5 if self._training_enabled else 3
        layout.split_column(
            Layout(name="header", size=header_size),
            Layout(name="main", ratio=1),
            Layout(name="footer", size=3),
        )
        
        # Split main into left and right
        layout["main"].split_row(
            Layout(name="left", ratio=2),
            Layout(name="right", ratio=3),
        )
        
        # Left side layout depends on whether models are loaded
        if self._num_models > 1:
            # Multi-model mode: Motors + Models + Commands + Log
            layout["left"].split_column(
                Layout(name="motors", ratio=2),
                Layout(name="models", ratio=1),
                Layout(name="commands", ratio=1),
                Layout(name="log", ratio=1),
            )
        else:
            # Single model mode: Motors + Commands + Log
            layout["left"].split_column(
                Layout(name="motors", ratio=2),
                Layout(name="commands", ratio=1),
                Layout(name="log", ratio=1),
            )
        
        # Right side: Observations + Actions/System (+ Gauges if any)
        if self._gauges:
            # Include gauge panel(s)
            layout["right"].split_column(
                Layout(name="observations", ratio=2),
                Layout(name="gauges", ratio=1),
                Layout(name="bottom_right", ratio=1),
            )
        else:
            layout["right"].split_column(
                Layout(name="observations", ratio=2),
                Layout(name="bottom_right", ratio=1),
            )
        
        # Bottom right: Actions + System side by side
        layout["bottom_right"].split_row(
            Layout(name="actions", ratio=1),
            Layout(name="system", ratio=1),
        )
        
        # Populate panels
        layout["header"].update(self._generate_header())
        layout["motors"].update(self._generate_motor_panel())
        if self._num_models > 1:
            layout["models"].update(self._generate_models_panel())
        layout["commands"].update(self._generate_command_panel())
        layout["log"].update(self._generate_log_panel())
        layout["observations"].update(self._generate_observation_panel())
        if self._gauges:
            layout["gauges"].update(self._generate_gauges_panel())
        layout["actions"].update(self._generate_action_panel())
        layout["system"].update(self._generate_system_panel())
        layout["footer"].update(self._generate_footer())
        
        return layout
    
    def _generate_header(self) -> Panel:
        """Generate the header panel."""
        runtime = time.time() - self._start_time
        spinner = self._spinner_chars[self._frame % len(self._spinner_chars)]
        
        # Status indicators
        device_count = len(self._devices)
        active_count = sum(1 for d in self._devices.values() if time.time() - d.last_seen < self.config.timeout_sec)
        
        # Build header text with proper Text styling (not markup strings)
        header = Text()
        header.append(f" {spinner} ", style=self.theme['accent'])
        header.append(self.config.title, style=self.theme['title'])
        header.append("  │  Motors: ", style=self.theme['primary'])
        if self._motor_enabled:
            header.append("● ", style=self.theme['accent'])
            header.append("ON", style=self.theme['accent'])
        else:
            header.append("○ ", style=self.theme['error'])
            header.append("OFF", style=self.theme['error'])
        header.append("  │  Devices: ", style=self.theme['primary'])
        header.append(f"{active_count}", style=self.theme['accent'])
        header.append(f"/{device_count}", style=self.theme['dim'])
        header.append(f"  │  Runtime: ", style=self.theme['primary'])
        header.append(f"{runtime:.1f}s", style=self.theme['secondary'])
        header.append(f"  │  Ep: ", style=self.theme['primary'])
        header.append(f"{self._episode_count}", style=self.theme['accent'])
        header.append(f"  Step: ", style=self.theme['primary'])
        header.append(f"{self._step_count}", style=self.theme['accent'])
        
        # Show module connection status
        if self._expected_modules:
            connected = len(self._connected_modules)
            total = len(self._expected_modules)
            header.append(f"  │  ", style=self.theme['dim'])
            if connected < total:
                # Still waiting - show progress
                header.append("⏳ ", style=self.theme['warning'])
                header.append(f"{connected}/{total}", style=self.theme['warning'])
            else:
                # All connected
                header.append("✓ ", style=self.theme['accent'])
                header.append(f"{connected}/{total}", style=self.theme['accent'])
        
        # Show environment ready status
        header.append(f"  │  ", style=self.theme['dim'])
        if self._env_ready:
            header.append("✓ ", style=self.theme['accent'])
            header.append("Ready", style=self.theme['accent'])
        else:
            header.append("⏳ ", style=self.theme['warning'])
            header.append("Not Ready", style=self.theme['warning'])
        
        # Show gauges summary in header if any
        for gauge_name, gauge in list(self._gauges.items())[:2]:  # Show max 2 in header
            header.append(f"  │  ", style=self.theme['dim'])
            # Extract emoji from title if present
            title = gauge['title']
            if title and len(title) > 0 and ord(title[0]) > 127:
                header.append(f"{title[0]} ", style=self.theme['accent'])
            
            if gauge['ready']:
                value = gauge['value']
                center = gauge['center_val']
                threshold = gauge['direction_threshold']
                
                if value > center + threshold:
                    direction = gauge['right_direction'].split()[0] if ' ' in gauge['right_direction'] else gauge['right_direction']
                    dir_style = self.theme['warning']
                elif value < center - threshold:
                    direction = gauge['left_direction'].split()[0] if ' ' in gauge['left_direction'] else gauge['left_direction']
                    dir_style = self.theme['warning']
                else:
                    direction = gauge['center_direction'].split()[0] if ' ' in gauge['center_direction'] else gauge['center_direction']
                    dir_style = self.theme['accent']
                
                header.append(f"{value:+.0f}{gauge['unit']} ", style=self.theme['accent'])
                header.append(direction, style=dir_style)
            else:
                header.append("...", style=self.theme['dim'])
        
        # Add training progress on second line if enabled
        if self._training_enabled:
            header.append("\n")
            
            # Progress percentage and bar
            progress = self._training_timesteps / max(self._training_total_timesteps, 1)
            progress_pct = progress * 100
            bar_width = 30
            filled = int(progress * bar_width)
            
            header.append("  🎓 TRAINING ", style=self.theme['title'])
            header.append(f"{progress_pct:5.1f}% ", style=self.theme['accent'])
            
            # Progress bar
            header.append("[", style=self.theme['dim'])
            header.append("█" * filled, style=self.theme['accent'])
            header.append("░" * (bar_width - filled), style=self.theme['dim'])
            header.append("] ", style=self.theme['dim'])
            
            # Stats
            header.append(f"{self._training_timesteps:,}/{self._training_total_timesteps:,}", style=self.theme['secondary'])
            header.append(f"  │  Episodes: ", style=self.theme['dim'])
            header.append(f"{self._training_episodes}", style=self.theme['accent'])
            header.append(f"  │  Avg Reward: ", style=self.theme['dim'])
            header.append(f"{self._training_avg_reward:+.2f}", style=self.theme['accent'])
            if self._training_last_reward != 0:
                header.append(f" (last: {self._training_last_reward:+.1f})", style=self.theme['secondary'])
        
        # Use larger height when training is enabled
        border_style = self.theme['accent'] if self._training_enabled else self.theme['border']
        return Panel(header, style=border_style, box=box.DOUBLE)
    
    def _generate_motor_panel(self) -> Panel:
        """Generate the motor status panel with motor error/mode/driver error info."""
        table = Table(show_header=True, expand=True, box=box.SIMPLE_HEAD)
        
        table.add_column("Module", style=self.theme['primary'], width=14)
        table.add_column("Pos", justify="right", style=self.theme['secondary'], width=7)
        table.add_column("Vel", justify="right", style=self.theme['secondary'], width=6)
        table.add_column("V", justify="right", style=self.theme['dim'], width=4)
        table.add_column("Mode", justify="center", width=6)  # Motor mode indicator
        table.add_column("Error", justify="left", width=14)   # Motor + Driver errors (decoded)
        table.add_column("Status", justify="center", width=8)
        
        devices = dict(self._devices)
        if not devices:
            table.add_row("⏳ Waiting...", "", "", "", "", "", "")
        else:
            now = time.time()
            for addr, dev in sorted(devices.items()):
                is_active = (now - dev.last_seen) < self.config.timeout_sec
                
                # Format values
                pos_str = f"{dev.position:+.2f}"
                vel_str = f"{dev.velocity:+.1f}"
                volt_str = f"{dev.voltage:.0f}"
                
                # Motor mode indicator (0=Reset/Off, 1=Calibration, 2=Active/On)
                mode_names = {0: "OFF", 1: "CAL", 2: "ON"}
                mode_name = mode_names.get(dev.motor_mode, "?")
                if dev.motor_mode == 2:
                    mode_str = f"[{self.theme['accent']}]● {mode_name}[/]"
                elif dev.motor_mode == 1:
                    mode_str = f"[{self.theme['warning']}]◐ {mode_name}[/]"
                else:
                    mode_str = f"[{self.theme['error']}]○ {mode_name}[/]"
                
                # Decode and display motor_error + driver_error using error decoder
                err_parts = []
                motor_err_str = self._error_decoder.decode_motor_error(dev.motor_error)
                driver_err_str = self._error_decoder.decode_driver_error(dev.driver_error)
                if motor_err_str:
                    err_parts.append(motor_err_str)
                if driver_err_str:
                    err_parts.append(driver_err_str)
                if err_parts:
                    err_str = f"[{self.theme['error']}]{' '.join(err_parts)}[/]"
                else:
                    err_str = f"[{self.theme['accent']}]OK[/]"
                
                # Status with visual indicator (based on connection status)
                if dev.error:  # Reset reason errors
                    status = f"[{self.theme['error']}]⚠ RST[/]"
                elif is_active:
                    switch = dev.custom_data.get("switch", False)
                    if switch:
                        status = f"[{self.theme['accent']}]● LIVE[/]"
                    else:
                        status = f"[{self.theme['warning']}]◐ IDLE[/]"
                else:
                    status = f"[{self.theme['dim']}]○ LOST[/]"
                
                # Dim if inactive
                style = "" if is_active else self.theme['dim']
                table.add_row(
                    Text(dev.name, style=style),
                    Text(pos_str, style=style),
                    Text(vel_str, style=style),
                    Text(volt_str, style=style),
                    mode_str,
                    err_str,
                    status
                )
        
        return Panel(
            table,
            title=f"[{self.theme['title']}]🔧 MOTORS[/]",
            border_style=self.theme['border'],
            box=box.ROUNDED
        )
    
    def _generate_action_panel(self) -> Panel:
        """Generate the action visualization panel."""
        if self._current_action is None or len(self._current_action) == 0:
            content = Text("Waiting for actions...", style=self.theme['dim'])
        else:
            # Create bar visualization for each action using Text objects
            content = Text()
            max_width = 16
            
            for i, val in enumerate(self._current_action[:6]):  # Limit to 6 actions for compact view
                # Normalize to [-1, 1] for display
                normalized = np.clip(val, -1, 1)
                
                # Create bar
                bar_pos = int((normalized + 1) / 2 * max_width)
                center = max_width // 2
                
                # Determine color based on magnitude
                if abs(normalized) > 0.7:
                    color = self.theme['warning']
                elif abs(normalized) > 0.3:
                    color = self.theme['accent']
                else:
                    color = self.theme['dim']
                
                # Build bar character by character
                content.append(f"a{i} ", style=self.theme['dim'])
                for j in range(max_width):
                    if j == center:
                        content.append("│", style=self.theme['dim'])
                    elif (j < center and j >= bar_pos and normalized < 0) or \
                         (j > center and j <= bar_pos and normalized > 0) or \
                         j == bar_pos:
                        content.append("█", style=color)
                    else:
                        content.append("·", style=self.theme['dim'])
                
                content.append(f" {val:+.2f}\n", style=self.theme['secondary'])
        
        title = Text()
        title.append("🎮 ACTIONS", style=self.theme['title'])
        
        return Panel(
            content,
            title=title,
            border_style=self.theme['border'],
            box=box.ROUNDED
        )
    
    def _generate_command_panel(self) -> Panel:
        """Generate the RL command visualization panel."""
        if self._commands is None or len(self._commands) == 0:
            content = Text("No commands configured", style=self.theme['dim'])
        else:
            content = Text()
            
            # Show mode indicator
            mode_text = "ONE-HOT" if self._onehot_mode else "CONTINUOUS"
            kb_text = " [K]" if self._keyboard_command_mode else ""
            content.append(f"Mode: {mode_text}{kb_text}\n", style=self.theme['dim'])
            content.append("\n")
            
            for i, val in enumerate(self._commands):
                name = self._command_names[i] if i < len(self._command_names) else f"cmd_{i}"
                
                # Truncate name for display
                display_name = name[:12] if len(name) > 12 else name
                
                if self._onehot_mode:
                    # One-hot mode: show filled/empty circle
                    is_active = float(val) > 0.5
                    marker = "●" if is_active else "○"
                    marker_style = self.theme['accent'] if is_active else self.theme['dim']
                    
                    content.append(f"[{i}] ", style=self.theme['dim'])
                    content.append(marker, style=marker_style)
                    content.append(f" {display_name}", style=self.theme['primary'] if is_active else self.theme['dim'])
                    content.append("\n")
                else:
                    # Continuous mode: show bar
                    is_selected = i == self._selected_command_idx
                    selector = "►" if is_selected else " "
                    
                    # Normalize for bar display
                    normalized = np.clip(float(val), -1, 1)
                    bar_width = 10
                    bar_pos = int((normalized + 1) / 2 * bar_width)
                    center = bar_width // 2
                    
                    content.append(selector, style=self.theme['accent'] if is_selected else self.theme['dim'])
                    content.append(f"[{i}]", style=self.theme['dim'])
                    
                    # Build bar
                    for j in range(bar_width):
                        if j == center:
                            content.append("│", style=self.theme['dim'])
                        elif (j < center and j >= bar_pos and normalized < 0) or \
                             (j > center and j <= bar_pos and normalized > 0) or \
                             j == bar_pos:
                            content.append("█", style=self.theme['accent'] if is_selected else self.theme['secondary'])
                        else:
                            content.append("·", style=self.theme['dim'])
                    
                    content.append(f" {float(val):+.2f}", style=self.theme['secondary'])
                    content.append(f" {display_name}\n", style=self.theme['primary'] if is_selected else self.theme['dim'])
            
            # Add key hints
            content.append("\n")
            if self._onehot_mode:
                content.append("Keys: 0-9=activate, +/-=cycle", style=self.theme['dim'])
            else:
                content.append("Keys: []=select, +/-=adjust", style=self.theme['dim'])
        
        title = Text()
        title.append("🎯 COMMANDS", style=self.theme['title'])
        if self._keyboard_command_mode:
            title.append(" ", style="default")
            title.append("[KB]", style=self.theme['warning'])
        
        return Panel(
            content,
            title=title,
            border_style=self.theme['warning'] if self._keyboard_command_mode else self.theme['border'],
            box=box.ROUNDED
        )
    
    def _generate_models_panel(self) -> Panel:
        """Generate the models panel for multi-model mode."""
        if self._num_models == 0:
            content = Text("Single model mode", style=self.theme['dim'])
        else:
            content = Text()
            
            # Header with current model prominently displayed
            content.append(f"Active: ", style=self.theme['dim'])
            if self._current_model_idx < len(self._model_names):
                current_name = self._model_names[self._current_model_idx]
                content.append(f"{current_name}", style=self.theme['accent'])
            content.append(f" [{self._current_model_idx + 1}/{self._num_models}]\n", style=self.theme['secondary'])
            content.append("\n")
            
            # List all models
            for i, name in enumerate(self._model_names):
                is_current = i == self._current_model_idx
                
                # Marker for current model
                if is_current:
                    content.append("► ", style=self.theme['accent'])
                else:
                    content.append("  ", style=self.theme['dim'])
                
                # Model index
                content.append(f"[{i + 1}] ", style=self.theme['dim'])
                
                # Model name
                display_name = name[:20] if len(name) > 20 else name
                if is_current:
                    content.append(display_name, style=self.theme['accent'])
                else:
                    content.append(display_name, style=self.theme['secondary'])
                
                # Show obs dim if available
                if i < len(self._model_obs_dims) and self._model_obs_dims[i]:
                    obs_dim = self._model_obs_dims[i]
                    content.append(f" (obs={obs_dim})", style=self.theme['dim'])
                
                content.append("\n")
            
            # Key hints
            content.append("\n")
            content.append("Keys: ,=prev  .=next  /=info", style=self.theme['dim'])
        
        title = Text()
        title.append("🤖 MODELS", style=self.theme['title'])
        
        return Panel(
            content,
            title=title,
            border_style=self.theme['accent'] if self._num_models > 1 else self.theme['border'],
            box=box.ROUNDED
        )
    
    def _generate_observation_panel(self) -> Panel:
        """Generate the observation components panel."""
        if not self._observation_components:
            content = Text("Waiting for observations...", style=self.theme['dim'])
        else:
            table = Table(show_header=True, expand=True, box=box.SIMPLE)
            table.add_column("", width=2)  # Used indicator
            table.add_column("Component", style=self.theme['primary'], width=16)
            table.add_column("Values", style=self.theme['secondary'], overflow="fold")
            table.add_column("Δ", justify="right", width=6)
            
            # Sort components: command obs first, then other used, then debug
            sorted_names = []
            command_names = [n for n in self._observation_components if n in self._command_obs_components]
            used_names = [n for n in self._observation_components if n in self._used_obs_components and n not in self._command_obs_components]
            debug_names = [n for n in self._observation_components if n not in self._used_obs_components]
            sorted_names = command_names + used_names + debug_names
            
            for name in sorted_names[:self.config.max_obs_components]:
                data = self._observation_components[name]
                history = self._observation_history.get(name, [])
                
                # Check component type
                is_command = name in self._command_obs_components
                is_used = name in self._used_obs_components
                
                # Used indicator: 🎯 for command, ● for used, ○ for debug
                used_indicator = Text()
                if is_command:
                    used_indicator.append("🎯", style=self.theme['warning'])
                elif is_used:
                    used_indicator.append("●", style=self.theme['accent'])
                else:
                    used_indicator.append("○", style=self.theme['dim'])
                
                # Component name with styling
                name_text = Text(name)
                if is_command:
                    name_text.stylize(self.theme['warning'])
                elif is_used:
                    name_text.stylize(self.theme['accent'])
                else:
                    name_text.stylize(self.theme['dim'])
                
                # Format values (show all for commands, truncate others)
                if is_command:
                    # Show command values with active indicator for one-hot
                    if self._onehot_mode and len(data) > 0:
                        # Find active index and show it
                        active_idx = int(np.argmax(data))
                        active_name = self._command_names[active_idx] if active_idx < len(self._command_names) else f"cmd_{active_idx}"
                        val_str = f"→{active_name} ["
                        val_str += " ".join(f"{v:.0f}" for v in data)
                        val_str += "]"
                    elif len(data) <= 5:
                        val_str = " ".join(f"{v:+.2f}" for v in data)
                    else:
                        val_str = " ".join(f"{v:+.2f}" for v in data[:4]) + f".. [{len(data)}]"
                else:
                    if len(data) <= 3:
                        val_str = " ".join(f"{v:+.2f}" for v in data)
                    else:
                        val_str = " ".join(f"{v:+.2f}" for v in data[:2]) + f".. [{len(data)}]"
                
                # Calculate delta from previous step
                delta_text = Text()
                if len(history) >= 2:
                    delta = np.mean(np.abs(data - history[-2]))
                    if delta > 0.1:
                        delta_text.append(f"↑{delta:.2f}", style=self.theme['warning'])
                    elif delta > 0.01:
                        delta_text.append(f"~{delta:.2f}", style=self.theme['dim'])
                    else:
                        delta_text.append("—", style=self.theme['dim'])
                
                table.add_row(used_indicator, name_text, val_str, delta_text)
            
            content = table
        
        # Build title
        title = Text()
        title.append("📊 OBS ", style=self.theme['title'])
        title.append("(🎯 cmd  ● used  ○ debug)", style=self.theme['dim'])
        
        return Panel(
            content,
            title=title,
            border_style=self.theme['border'],
            box=box.ROUNDED
        )
    
    def _generate_system_panel(self) -> Panel:
        """Generate the system performance panel."""
        # Calculate metrics
        freq = 1.0 / self._loop_dt if self._loop_dt > 0 else 0
        avg_reward = np.mean(self._reward_history[-20:]) if self._reward_history else 0
        
        # Build content using Text for proper styling
        content = Text()
        
        # Timing row
        content.append("dt:", style=self.theme['dim'])
        content.append(f"{self._loop_dt*1000:.1f}ms ", style=self.theme['accent'])
        content.append("@", style=self.theme['dim'])
        content.append(f"{freq:.0f}Hz\n", style=self.theme['accent'])
        
        # Communication row
        content.append("Cmd:", style=self.theme['dim'])
        content.append(f"{self._cmd_count} ", style=self.theme['secondary'])
        content.append("Fb:", style=self.theme['dim'])
        content.append(f"{self._fb_count}\n", style=self.theme['secondary'])
        
        # Rewards row
        content.append("R:", style=self.theme['dim'])
        content.append(f"{self._last_reward:+.2f} ", style=self.theme['accent'])
        content.append("Σ:", style=self.theme['dim'])
        content.append(f"{self._episode_reward:+.1f}\n", style=self.theme['secondary'])
        
        # Avg reward with sparkline
        content.append("Avg:", style=self.theme['dim'])
        content.append(f"{avg_reward:+.3f} ", style=self.theme['secondary'])
        if len(self._reward_history) > 3:
            sparkline = self._generate_sparkline(self._reward_history[-15:])
            content.append(sparkline, style=self.theme['accent'])
        
        title = Text()
        title.append("⚡ PERF", style=self.theme['title'])
        
        return Panel(
            content,
            title=title,
            border_style=self.theme['border'],
            box=box.ROUNDED
        )
    
    def _generate_gauges_panel(self) -> Panel:
        """Generate panel for all custom gauges."""
        from rich.columns import Columns
        
        if not self._gauges:
            content = Text("No gauges configured", style=self.theme['dim'])
            return Panel(content, title="GAUGES", border_style=self.theme['border'], box=box.ROUNDED)
        
        # If single gauge, show detailed view
        if len(self._gauges) == 1:
            name, gauge = list(self._gauges.items())[0]
            return self._generate_single_gauge_panel(name, gauge)
        
        # Multiple gauges - show compact view
        content = Text()
        for i, (name, gauge) in enumerate(self._gauges.items()):
            if i > 0:
                content.append("\n")
            self._append_gauge_compact(content, name, gauge)
        
        title = Text()
        title.append("📊 GAUGES", style=self.theme['title'])
        
        return Panel(
            content,
            title=title,
            border_style=self.theme['border'],
            box=box.ROUNDED
        )
    
    def _generate_single_gauge_panel(self, name: str, gauge: Dict[str, Any]) -> Panel:
        """Generate a detailed panel for a single gauge."""
        content = Text()
        
        value = gauge['value']
        min_val = gauge['min_val']
        max_val = gauge['max_val']
        center_val = gauge['center_val']
        unit = gauge['unit']
        threshold = gauge['direction_threshold']
        
        if not gauge['ready']:
            # Not ready state
            content.append(f"Collecting data...\n\n", style=self.theme['warning'])
            content.append("Waiting for enough samples\n", style=self.theme['dim'])
            content.append(f"to compute {name}.", style=self.theme['dim'])
        else:
            # Direction indicator
            if value > center_val + threshold:
                direction = gauge['right_direction']
                dir_style = self.theme['warning']
            elif value < center_val - threshold:
                direction = gauge['left_direction']
                dir_style = self.theme['warning']
            else:
                direction = gauge['center_direction']
                dir_style = self.theme['accent']
            
            # Large value display (prominent)
            content.append("         ", style=self.theme['dim'])
            content.append(f"{value:+.1f}{unit}", style=f"bold {self.theme['accent']}")
            content.append(f"  {direction}\n\n", style=dir_style)
            
            # Gauge bar visualization
            bar_width = 30
            range_val = max_val - min_val
            if range_val > 0:
                # Position on bar (0 to bar_width)
                normalized = (value - min_val) / range_val
                pos = int(np.clip(normalized, 0, 1) * (bar_width - 1))
                
                # Center position on bar
                center_normalized = (center_val - min_val) / range_val
                center_pos = int(np.clip(center_normalized, 0, 1) * (bar_width - 1))
            else:
                pos = bar_width // 2
                center_pos = bar_width // 2
            
            # Bar with range markers
            content.append(f"  {min_val:+.0f}{unit}", style=self.theme['dim'])
            spacing = bar_width - 12 - len(str(int(max_val))) - len(unit)
            content.append(" " * max(1, spacing // 2), style=self.theme['dim'])
            content.append(f"{center_val:.0f}{unit}", style=self.theme['secondary'])
            content.append(" " * max(1, spacing - spacing // 2), style=self.theme['dim'])
            content.append(f"{max_val:+.0f}{unit}\n", style=self.theme['dim'])
            
            content.append("    ", style=self.theme['dim'])
            content.append("[", style=self.theme['dim'])
            for i in range(bar_width):
                if i == center_pos:
                    content.append("│", style=self.theme['secondary'])
                elif i == pos:
                    content.append("◆", style=self.theme['accent'])
                elif (i < center_pos and i >= pos and value < center_val) or \
                     (i > center_pos and i <= pos and value > center_val):
                    content.append("═", style=self.theme['secondary'])
                else:
                    content.append("·", style=self.theme['dim'])
            content.append("]\n", style=self.theme['dim'])
            
            # Labels
            content.append("    ", style=self.theme['dim'])
            left_label = gauge['left_label']
            right_label = gauge['right_label']
            label_spacing = bar_width - len(left_label) - len(right_label)
            content.append(left_label, style=self.theme['dim'])
            content.append(" " * max(1, label_spacing), style=self.theme['dim'])
            content.append(f"{right_label}\n", style=self.theme['dim'])
            
            # Extra info
            for key, val in gauge['extra_info'].items():
                content.append(f"    {key}: ", style=self.theme['primary'])
                content.append(f"{val}\n", style=self.theme['secondary'])
            
            # History sparkline
            if gauge['track_history'] and len(gauge['history']) > 5:
                sparkline = self._generate_sparkline(gauge['history'][-20:], width=20)
                content.append("    History: ", style=self.theme['dim'])
                content.append(sparkline, style=self.theme['secondary'])
        
        title = Text()
        title.append(gauge['title'], style=self.theme['title'])
        
        return Panel(
            content,
            title=title,
            border_style=self.theme['accent'] if gauge['ready'] else self.theme['warning'],
            box=box.ROUNDED
        )
    
    def _append_gauge_compact(self, content: Text, name: str, gauge: Dict[str, Any]) -> None:
        """Append a compact gauge display to content."""
        value = gauge['value']
        center_val = gauge['center_val']
        threshold = gauge['direction_threshold']
        unit = gauge['unit']
        
        # Title/emoji
        title = gauge['title']
        if title and len(title) > 0:
            content.append(f"{title}: ", style=self.theme['primary'])
        else:
            content.append(f"{name}: ", style=self.theme['primary'])
        
        if not gauge['ready']:
            content.append("...", style=self.theme['dim'])
        else:
            # Direction
            if value > center_val + threshold:
                direction = gauge['right_direction'].split()[0] if ' ' in gauge['right_direction'] else gauge['right_direction']
                dir_style = self.theme['warning']
            elif value < center_val - threshold:
                direction = gauge['left_direction'].split()[0] if ' ' in gauge['left_direction'] else gauge['left_direction']
                dir_style = self.theme['warning']
            else:
                direction = gauge['center_direction'].split()[0] if ' ' in gauge['center_direction'] else gauge['center_direction']
                dir_style = self.theme['accent']
            
            content.append(f"{value:+.1f}{unit} ", style=self.theme['accent'])
            content.append(direction, style=dir_style)
            
            # Extra info inline
            for key, val in list(gauge['extra_info'].items())[:1]:
                content.append(f" │ {key}:{val}", style=self.theme['dim'])
    
    def _generate_log_panel(self) -> Panel:
        """Generate the log/messages panel."""
        content = Text()
        
        # Always show module connection status at top if we have expected modules
        if self._expected_modules:
            connected = len(self._connected_modules)
            total = len(self._expected_modules)
            missing = self.get_missing_modules()
            
            if connected < total:
                # Show progress bar style
                content.append(f"⏳ Modules: ", style=self.theme['warning'])
                content.append(f"{connected}", style=self.theme['accent'])
                content.append(f"/{total} ", style=self.theme['dim'])
                # Show progress dots
                for i, mid in enumerate(self._expected_modules):
                    if mid in self._connected_modules:
                        content.append("●", style=self.theme['accent'])
                    else:
                        content.append("○", style=self.theme['dim'])
                content.append("\n")
                
                # Show which are missing
                if missing:
                    content.append(f"  Missing: {missing}\n", style=self.theme['dim'])
            else:
                # All connected
                content.append(f"✓ All {total} modules ready ", style=self.theme['accent'])
                content.append("●" * total, style=self.theme['accent'])
                content.append("\n")
        
        # Get recent log messages (reduce count if showing module status)
        max_logs = self._max_log_lines - 2 if self._expected_modules else self._max_log_lines
        recent_logs = self._log_messages[-max_logs:]
        
        if recent_logs:
            now = time.time()
            for timestamp, level, message in recent_logs:
                # Time ago
                elapsed = now - timestamp
                if elapsed < 60:
                    time_str = f"{elapsed:.0f}s"
                else:
                    time_str = f"{elapsed/60:.0f}m"
                
                # Level styling
                if level == "error":
                    level_style = self.theme['error']
                    prefix = "✗"
                elif level == "warn":
                    level_style = self.theme['warning']
                    prefix = "⚠"
                elif level == "success":
                    level_style = self.theme['accent']
                    prefix = "✓"
                else:
                    level_style = self.theme['dim']
                    prefix = "•"
                
                content.append(f"{time_str:>4} ", style=self.theme['dim'])
                content.append(f"{prefix} ", style=level_style)
                # Truncate long messages
                msg = message[:45] + "..." if len(message) > 48 else message
                content.append(f"{msg}\n", style=self.theme['secondary'])
        elif not self._expected_modules:
            content.append("No messages yet...", style=self.theme['dim'])
        
        title = Text()
        title.append("📋 LOG", style=self.theme['title'])
        
        return Panel(
            content,
            title=title,
            border_style=self.theme['border'],
            box=box.ROUNDED
        )
    
    def _generate_footer(self) -> Panel:
        """Generate the footer with controls info."""
        controls = Text()
        
        # Motor controls
        controls.append("Motor: ", style=self.theme['dim'])
        controls.append("[e]", style=self.theme['accent'])
        controls.append("nbl ", style=self.theme['dim'])
        controls.append("[d]", style=self.theme['accent'])
        controls.append("is ", style=self.theme['dim'])
        controls.append("[r]", style=self.theme['accent'])
        controls.append("st ", style=self.theme['dim'])
        controls.append("[c]", style=self.theme['accent'])
        controls.append("al ", style=self.theme['dim'])
        
        # Command controls
        controls.append(" │ Cmd: ", style=self.theme['dim'])
        controls.append("[0-9]", style=self.theme['warning'])
        controls.append(" sel ", style=self.theme['dim'])
        controls.append("[+-]", style=self.theme['warning'])
        controls.append(" adj ", style=self.theme['dim'])
        controls.append("[R]", style=self.theme['warning'])
        controls.append(" resmp ", style=self.theme['dim'])
        controls.append("[k]", style=self.theme['warning'])
        controls.append(" kb ", style=self.theme['dim'])
        controls.append("[i]", style=self.theme['secondary'])
        controls.append(" info", style=self.theme['dim'])
        
        # Quit
        controls.append(" │ ", style=self.theme['dim'])
        controls.append("[q]", style=self.theme['error'])
        controls.append(" Quit", style=self.theme['dim'])
        
        # Add custom status items
        if self._custom_status:
            controls.append("  │  ", style=self.theme['dim'])
            for k, v in list(self._custom_status.items())[:2]:
                controls.append(f"{k}:", style=self.theme['dim'])
                controls.append(f"{v} ", style=self.theme['accent'])
        
        return Panel(controls, style=self.theme['border'], box=box.SIMPLE)
    
    def _generate_sparkline(self, values: List[float], width: int = 15) -> str:
        """Generate a mini sparkline for the footer."""
        if not values:
            return ""
        
        chars = "▁▂▃▄▅▆▇█"
        min_val, max_val = min(values), max(values)
        range_val = max_val - min_val if max_val != min_val else 1
        
        # Sample if too many values
        if len(values) > width:
            step = len(values) / width
            sampled = [values[int(i * step)] for i in range(width)]
        else:
            sampled = values
        
        sparkline = ""
        for v in sampled:
            idx = int((v - min_val) / range_val * (len(chars) - 1))
            sparkline += chars[idx]
        
        return sparkline
    
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False


# Convenience function for quick dashboard creation
def create_dashboard(
    title: str = "Capybarish Dashboard",
    motor_mode: bool = False,
    rl_mode: bool = False,
    **kwargs
) -> Union[RichDashboard, MotorDashboard, RLDashboard]:
    """Create a dashboard with common configuration.
    
    Args:
        title: Dashboard title
        motor_mode: Use motor-specific columns
        rl_mode: Use RL-enhanced multi-panel dashboard
        **kwargs: Additional DashboardConfig parameters
        
    Returns:
        Configured dashboard instance
    """
    if rl_mode:
        config = RLDashboardConfig(title=title, **kwargs)
        return RLDashboard(config)
    
    config = DashboardConfig(title=title, **kwargs)
    
    if motor_mode:
        return MotorDashboard(config)
    return RichDashboard(config)
