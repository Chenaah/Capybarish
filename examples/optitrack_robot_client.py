#!/usr/bin/env python3
"""
OptiTrack Robot Client Example with Rich Dashboard.

This script demonstrates how to:
1. Connect to OptiTrack Motive and receive rigid body position data
2. Calculate 2D distance from current position to a goal
3. Send SensorData messages (with goal_distance) to a server
4. Display a live dashboard with all tracked rigid bodies

Usage:
    python optitrack_robot_client.py
    python optitrack_robot_client.py --optitrack-ip 129.105.73.172 --rigid-body-id 3
    python optitrack_robot_client.py --goal-x 1.0 --goal-y 2.0

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>
"""

import argparse
import os
import signal
import socket
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, str(__file__).rsplit("/", 2)[0])

# Import capybarish message types
from capybarish.generated import (
    MotorCommand,
    SensorData,
    MotorData,
    IMUData,
    ErrorData,
)

# Import NatNet client
from capybarish.natnet.NatNetClient import NatNetClient

# Rich imports for dashboard
try:
    from rich.console import Console
    from rich.live import Live
    from rich.table import Table
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.text import Text
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    print("Warning: 'rich' not installed. Install with: pip install rich")
    print("Falling back to simple text output.\n")


# =============================================================================
# Configuration
# =============================================================================

DEFAULT_OPTITRACK_SERVER = "129.105.73.172"
DEFAULT_CLIENT_ADDRESS = "0.0.0.0"
DEFAULT_RIGID_BODY_ID = 3

DEFAULT_SERVER_IP = "127.0.0.1"
DEFAULT_SERVER_PORT = 6666
DEFAULT_COMMAND_PORT = 6667

FEEDBACK_RATE = 50.0
DASHBOARD_REFRESH_RATE = 10  # Hz


# =============================================================================
# OptiTrack Robot Client
# =============================================================================

class OptiTrackRobotClient:
    """Robot client with OptiTrack tracking and goal distance calculation."""
    
    def __init__(
        self,
        module_id: int = 1,
        optitrack_server_ip: str = DEFAULT_OPTITRACK_SERVER,
        optitrack_client_ip: str = DEFAULT_CLIENT_ADDRESS,
        rigid_body_id: int = DEFAULT_RIGID_BODY_ID,
        server_ip: str = DEFAULT_SERVER_IP,
        server_port: int = DEFAULT_SERVER_PORT,
        command_port: int = DEFAULT_COMMAND_PORT,
        goal_x: float = 0.0,
        goal_y: float = 0.0,
    ):
        self.module_id = module_id
        self.optitrack_server_ip = optitrack_server_ip
        self.optitrack_client_ip = optitrack_client_ip
        self.rigid_body_id = rigid_body_id
        self.server_ip = server_ip
        self.server_port = server_port
        self.command_port = command_port
        
        # 2D goal position (x, z in OptiTrack coords)
        self.goal_position = np.array([goal_x, goal_y])
        
        # OptiTrack data: {rb_id: (position, rotation, last_update_time)}
        self.optitrack_data: Dict[int, Tuple[List[float], List[float], float]] = {}
        self.current_position: Optional[np.ndarray] = None
        self.current_rotation: Optional[np.ndarray] = None
        
        # NatNet client
        self.streaming_client: Optional[NatNetClient] = None
        
        # Network sockets
        self._send_socket: Optional[socket.socket] = None
        self._recv_socket: Optional[socket.socket] = None
        
        # Threading
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        # Statistics
        self.send_count = 0
        self.recv_count = 0
        self.optitrack_frame_count = 0
        self.last_cmd_time = 0.0
        self.optitrack_fps = 0.0
        self._fps_counter = 0
        self._fps_last_time = time.time()
        
        # Timestamps
        self._start_time = time.time()
        
        # Motor state
        self.motor_enabled = False
        self.motor_target = 0.0
        
        # Connection status
        self.optitrack_connected = False
    
    def _receive_rigid_body_frame(
        self,
        new_id: int,
        position: Tuple[float, float, float],
        rotation: Tuple[float, float, float, float],
    ) -> None:
        """Callback for rigid body data from OptiTrack."""
        now = time.time()
        with self._lock:
            self.optitrack_data[new_id] = (list(position), list(rotation), now)
            self.optitrack_connected = True
            
            if new_id == self.rigid_body_id:
                self.current_position = np.array(position)
                self.current_rotation = np.array(rotation)
                self.optitrack_frame_count += 1
                self._fps_counter += 1
                
                # Calculate FPS every second
                if now - self._fps_last_time >= 1.0:
                    self.optitrack_fps = self._fps_counter / (now - self._fps_last_time)
                    self._fps_counter = 0
                    self._fps_last_time = now
    
    def _receive_new_frame(self, data_dict: dict) -> None:
        """Callback for new frame from OptiTrack."""
        pass
    
    def calculate_goal_distance(self) -> float:
        """Calculate 2D distance to goal (X-Y plane, Z-up coordinate system)."""
        if self.current_position is None:
            return -1.0
        current_2d = np.array([self.current_position[0], self.current_position[1]])
        return float(np.linalg.norm(current_2d - self.goal_position))
    
    def get_position_2d(self) -> Optional[np.ndarray]:
        """Get current 2D position [x, y] (Z-up coordinate system)."""
        if self.current_position is None:
            return None
        return np.array([self.current_position[0], self.current_position[1]])
    
    def _receive_commands_loop(self) -> None:
        """Background thread for receiving motor commands."""
        while self._running:
            try:
                data, addr = self._recv_socket.recvfrom(1024)
                if len(data) < MotorCommand._SIZE:
                    continue
                cmd = MotorCommand.deserialize(data)
                with self._lock:
                    self.motor_target = cmd.target
                    self.motor_enabled = (cmd.switch_ == 1)
                    self.last_cmd_time = cmd.timestamp
                    self.recv_count += 1
            except socket.timeout:
                continue
            except Exception:
                if self._running:
                    pass  # Silently ignore errors during shutdown
    
    def send_sensor_data(self) -> None:
        """Send SensorData with goal distance."""
        now = time.time()
        goal_dist = self.calculate_goal_distance()
        
        msg = SensorData(
            module_id=self.module_id,
            receive_dt=0,
            timestamp=int((now - self._start_time) * 1e6),
            switch_off=0 if self.motor_enabled else 1,
            last_rcv_timestamp=self.last_cmd_time,
            info=0,
            motor=MotorData(),
            imu=IMUData(),
            error=ErrorData(),
            goal_distance=goal_dist,
        )
        
        try:
            self._send_socket.sendto(msg.serialize(), (self.server_ip, self.server_port))
            self.send_count += 1
        except Exception:
            pass
    
    def start(self) -> bool:
        """Start the client."""
        # Initialize NatNet
        self.streaming_client = NatNetClient()
        self.streaming_client.set_client_address(self.optitrack_client_ip)
        self.streaming_client.set_server_address(self.optitrack_server_ip)
        self.streaming_client.set_use_multicast(False)
        self.streaming_client.rigid_body_listener = self._receive_rigid_body_frame
        self.streaming_client.new_frame_listener = self._receive_new_frame
        self.streaming_client.set_print_level(0)
        
        is_running = self.streaming_client.run('d')
        if not is_running:
            return False
        
        time.sleep(0.5)
        
        # Create sockets
        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        self._recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._recv_socket.bind(("0.0.0.0", self.command_port))
        except OSError:
            pass
        self._recv_socket.settimeout(0.1)
        
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_commands_loop, daemon=True)
        self._recv_thread.start()
        
        return True
    
    def stop(self) -> None:
        """Stop the client."""
        self._running = False
        
        # Stop NatNet
        if self.streaming_client:
            try:
                self.streaming_client.shutdown()
            except Exception:
                pass
        
        # Wait for thread
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=0.5)
        
        # Close sockets
        try:
            if self._send_socket:
                self._send_socket.close()
            if self._recv_socket:
                self._recv_socket.close()
        except Exception:
            pass
    
    def run_with_dashboard(self, rate_hz: float = FEEDBACK_RATE) -> None:
        """Run with rich dashboard display."""
        period = 1.0 / rate_hz
        dashboard_period = 1.0 / DASHBOARD_REFRESH_RATE
        last_dashboard_update = 0.0
        
        console = Console()
        
        with Live(self._generate_dashboard(), refresh_per_second=DASHBOARD_REFRESH_RATE, console=console) as live:
            try:
                while self._running:
                    loop_start = time.time()
                    
                    self.send_sensor_data()
                    
                    # Update dashboard
                    if time.time() - last_dashboard_update >= dashboard_period:
                        live.update(self._generate_dashboard())
                        last_dashboard_update = time.time()
                    
                    elapsed = time.time() - loop_start
                    if elapsed < period:
                        time.sleep(period - elapsed)
            except KeyboardInterrupt:
                pass
    
    def run_simple(self, rate_hz: float = FEEDBACK_RATE) -> None:
        """Run with simple text output (no rich)."""
        period = 1.0 / rate_hz
        last_print = 0.0
        
        print(f"\nRunning at {rate_hz} Hz... Press Ctrl+C to stop\n")
        
        try:
            while self._running:
                loop_start = time.time()
                self.send_sensor_data()
                
                # Print status every second
                if time.time() - last_print >= 1.0:
                    self._print_status()
                    last_print = time.time()
                
                elapsed = time.time() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)
        except KeyboardInterrupt:
            pass
    
    def _print_status(self) -> None:
        """Print simple text status."""
        print("-" * 70)
        print(f"Rigid Bodies Detected: {list(self.optitrack_data.keys())}")
        print(f"Tracking RB ID: {self.rigid_body_id} | OptiTrack FPS: {self.optitrack_fps:.1f}")
        
        pos_2d = self.get_position_2d()
        if pos_2d is not None:
            print(f"Position (2D): ({pos_2d[0]:.3f}, {pos_2d[1]:.3f})")
            print(f"Goal: ({self.goal_position[0]:.2f}, {self.goal_position[1]:.2f})")
            print(f"Distance to Goal: {self.calculate_goal_distance():.3f} m")
        else:
            print("Waiting for tracked rigid body data...")
        
        print(f"TX: {self.send_count} | RX: {self.recv_count}")
    
    def _generate_dashboard(self) -> Layout:
        """Generate the rich dashboard layout."""
        layout = Layout()
        
        # Split into top and bottom
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main"),
            Layout(name="footer", size=3),
        )
        
        # Header
        header_text = Text("OptiTrack Robot Client", style="bold cyan", justify="center")
        layout["header"].update(Panel(header_text, style="cyan"))
        
        # Main content split
        layout["main"].split_row(
            Layout(name="left", ratio=1),
            Layout(name="right", ratio=1),
        )
        
        # Left: Rigid Bodies Table
        layout["left"].update(self._create_rigid_bodies_panel())
        
        # Right: Tracking Info
        layout["right"].split_column(
            Layout(name="tracking", ratio=2),
            Layout(name="stats", ratio=1),
        )
        layout["right"]["tracking"].update(self._create_tracking_panel())
        layout["right"]["stats"].update(self._create_stats_panel())
        
        # Footer
        runtime = time.time() - self._start_time
        footer_text = Text(
            f"Runtime: {runtime:.1f}s | Press Ctrl+C to stop",
            style="dim",
            justify="center"
        )
        layout["footer"].update(Panel(footer_text, style="dim"))
        
        return layout
    
    def _create_rigid_bodies_panel(self) -> Panel:
        """Create panel showing all detected rigid bodies."""
        table = Table(title="Detected Rigid Bodies", expand=True)
        table.add_column("ID", style="cyan", justify="center", width=6)
        table.add_column("X", justify="right", width=10)
        table.add_column("Y", justify="right", width=10)
        table.add_column("Z", justify="right", width=10)
        table.add_column("Status", justify="center", width=10)
        
        now = time.time()
        with self._lock:
            if not self.optitrack_data:
                table.add_row("-", "-", "-", "-", "[yellow]No data[/]")
            else:
                for rb_id in sorted(self.optitrack_data.keys()):
                    pos, rot, last_time = self.optitrack_data[rb_id]
                    age = now - last_time
                    
                    # Highlight tracked rigid body
                    if rb_id == self.rigid_body_id:
                        id_str = f"[bold green]►{rb_id}[/]"
                        status = "[bold green]TRACKING[/]"
                    else:
                        id_str = str(rb_id)
                        status = "[dim]idle[/]" if age < 0.5 else "[red]stale[/]"
                    
                    table.add_row(
                        id_str,
                        f"{pos[0]:8.3f}",
                        f"{pos[1]:8.3f}",
                        f"{pos[2]:8.3f}",
                        status,
                    )
        
        return Panel(table, title="[bold]OptiTrack Data[/]", border_style="blue")
    
    def _create_tracking_panel(self) -> Panel:
        """Create panel showing tracking and goal info."""
        lines = []
        
        # Connection status
        if self.optitrack_connected:
            lines.append(f"[green]● Connected to OptiTrack[/]")
            lines.append(f"  Server: {self.optitrack_server_ip}")
        else:
            lines.append(f"[red]○ Waiting for OptiTrack...[/]")
            lines.append(f"  Server: {self.optitrack_server_ip}")
        
        lines.append("")
        lines.append(f"[cyan]Tracking Rigid Body ID: {self.rigid_body_id}[/]")
        lines.append("")
        
        # Current position
        pos_2d = self.get_position_2d()
        if pos_2d is not None:
            lines.append("[bold]Current Position (2D):[/]")
            lines.append(f"  X: [yellow]{pos_2d[0]:+8.4f}[/] m")
            lines.append(f"  Y: [yellow]{pos_2d[1]:+8.4f}[/] m")
            
            if self.current_position is not None:
                lines.append(f"  Z: [dim]{self.current_position[2]:+8.4f}[/] m (height)")
        else:
            lines.append("[yellow]Position: Waiting for data...[/]")
        
        lines.append("")
        
        # Goal info
        lines.append("[bold]Goal Position (2D):[/]")
        lines.append(f"  X: [magenta]{self.goal_position[0]:+8.4f}[/] m")
        lines.append(f"  Y: [magenta]{self.goal_position[1]:+8.4f}[/] m")
        lines.append("")
        
        # Distance
        dist = self.calculate_goal_distance()
        if dist >= 0:
            if dist < 0.1:
                dist_style = "bold green"
                dist_label = "AT GOAL!"
            elif dist < 0.5:
                dist_style = "yellow"
                dist_label = "Close"
            else:
                dist_style = "white"
                dist_label = ""
            lines.append(f"[bold]Distance to Goal:[/] [{dist_style}]{dist:.4f} m[/] {dist_label}")
        else:
            lines.append("[dim]Distance: N/A (no position data)[/]")
        
        content = "\n".join(lines)
        return Panel(content, title="[bold]Tracking & Goal[/]", border_style="green")
    
    def _create_stats_panel(self) -> Panel:
        """Create panel showing statistics."""
        lines = [
            f"[cyan]OptiTrack FPS:[/] {self.optitrack_fps:.1f}",
            f"[cyan]Frames Received:[/] {self.optitrack_frame_count}",
            "",
            f"[green]SensorData TX:[/] {self.send_count}",
            f"[yellow]Commands RX:[/] {self.recv_count}",
            f"[dim]Robot Server: {self.server_ip}:{self.server_port}[/]",
        ]
        content = "\n".join(lines)
        return Panel(content, title="[bold]Statistics[/]", border_style="yellow")
    
    def set_goal(self, x: float, y: float) -> None:
        """Set a new 2D goal position."""
        self.goal_position = np.array([x, y])


# =============================================================================
# Entry Point
# =============================================================================

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OptiTrack Robot Client with Dashboard",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    
    parser.add_argument("--optitrack-ip", "-o", type=str, default=DEFAULT_OPTITRACK_SERVER)
    parser.add_argument("--client-ip", type=str, default=DEFAULT_CLIENT_ADDRESS)
    parser.add_argument("--rigid-body-id", "-r", type=int, default=DEFAULT_RIGID_BODY_ID)
    parser.add_argument("--goal-x", "-gx", type=float, default=0.0)
    parser.add_argument("--goal-y", "-gy", type=float, default=0.0)
    parser.add_argument("--module-id", "-m", type=int, default=100)
    parser.add_argument("--server-ip", "-s", type=str, default=DEFAULT_SERVER_IP)
    parser.add_argument("--server-port", "-p", type=int, default=DEFAULT_SERVER_PORT)
    parser.add_argument("--command-port", "-c", type=int, default=DEFAULT_COMMAND_PORT)
    parser.add_argument("--rate", type=float, default=FEEDBACK_RATE)
    parser.add_argument("--no-dashboard", action="store_true", help="Disable rich dashboard")
    
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    
    client = OptiTrackRobotClient(
        module_id=args.module_id,
        optitrack_server_ip=args.optitrack_ip,
        optitrack_client_ip=args.client_ip,
        rigid_body_id=args.rigid_body_id,
        server_ip=args.server_ip,
        server_port=args.server_port,
        command_port=args.command_port,
        goal_x=args.goal_x,
        goal_y=args.goal_y,
    )
    
    # Signal handler with forced exit
    shutdown_requested = threading.Event()
    
    def signal_handler(signum, frame):
        if shutdown_requested.is_set():
            # Second signal - force exit
            print("\nForce exit...")
            os._exit(1)
        shutdown_requested.set()
        client._running = False
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    use_dashboard = RICH_AVAILABLE and not args.no_dashboard
    
    if not use_dashboard:
        print("=" * 60)
        print("OptiTrack Robot Client")
        print("=" * 60)
        print(f"OptiTrack Server: {args.optitrack_ip}")
        print(f"Rigid Body ID:    {args.rigid_body_id}")
        print(f"Goal Position:    ({args.goal_x}, {args.goal_y})")
        print("=" * 60)
    
    try:
        if not client.start():
            print("ERROR: Could not start client")
            sys.exit(1)
        
        if use_dashboard:
            client.run_with_dashboard(rate_hz=args.rate)
        else:
            client.run_simple(rate_hz=args.rate)
    finally:
        client.stop()
        
        # Print summary
        print("\n" + "=" * 60)
        print("Summary:")
        print(f"  Rigid bodies seen: {list(client.optitrack_data.keys())}")
        print(f"  OptiTrack frames: {client.optitrack_frame_count}")
        print(f"  SensorData sent: {client.send_count}")
        print(f"  Commands received: {client.recv_count}")
        if client.current_position is not None:
            print(f"  Final distance: {client.calculate_goal_distance():.3f} m")
        print("=" * 60)
        
        # Force exit to kill any lingering NatNet threads
        os._exit(0)


if __name__ == "__main__":
    main()
