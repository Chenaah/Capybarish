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

from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.layout import Layout
from rich.style import Style


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
            **kwargs
        )


# Convenience function for quick dashboard creation
def create_dashboard(
    title: str = "Capybarish Dashboard",
    motor_mode: bool = False,
    **kwargs
) -> RichDashboard:
    """Create a dashboard with common configuration.
    
    Args:
        title: Dashboard title
        motor_mode: Use motor-specific columns
        **kwargs: Additional DashboardConfig parameters
        
    Returns:
        Configured dashboard instance
    """
    config = DashboardConfig(title=title, **kwargs)
    
    if motor_mode:
        return MotorDashboard(config)
    return RichDashboard(config)
