#!/usr/bin/env python3
"""
Dummy ESP32 Client for Testing.

This script simulates an ESP32 motor controller client that communicates
with the server (basic_usage_pubsub.py). Each client instance:
- Sends SensorData (motor feedback) to the server
- Receives MotorCommand (motor commands) from the server
- Simulates motor dynamics (position tracking toward target)

To run multiple clients on the same machine, each client uses a unique
loopback IP address (127.0.0.x where x is the module_id). This allows
the server to see them as distinct devices.

Usage:
    # Run a single client with module_id 1
    python dummy_esp32_client.py --module-id 1

    # Run multiple clients in separate terminals
    python dummy_esp32_client.py --module-id 1
    python dummy_esp32_client.py --module-id 2
    python dummy_esp32_client.py --module-id 3

    # Or use the launcher mode to run multiple clients in one process
    python dummy_esp32_client.py --launch 3

    # Custom server address (if server is on another machine)
    python dummy_esp32_client.py --module-id 1 --server-ip 192.168.1.100

Requirements:
    - Server (basic_usage_pubsub.py) should be running
    - On Linux, binding to 127.0.0.x IPs works out of the box
    - On macOS, you may need to add loopback aliases:
        sudo ifconfig lo0 alias 127.0.0.2 up
        sudo ifconfig lo0 alias 127.0.0.3 up

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
"""

import argparse
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

# Import capybarish message types
from capybarish.generated import (
    MotorCommand,
    SensorData,
    MotorData,
    IMUData,
    ErrorData,
    UWBDistances,
)


# =============================================================================
# Configuration
# =============================================================================

DEFAULT_SERVER_IP = "127.0.0.1"
DEFAULT_SERVER_PORT = 6666        # Port server listens on for feedback
DEFAULT_COMMAND_PORT = 6667       # Port client listens on for commands

FEEDBACK_RATE = 50.0              # Hz - rate of sending feedback to server
MOTOR_TIME_CONSTANT = 0.1         # Seconds - motor response time constant
MOTOR_MAX_VELOCITY = 10.0         # rad/s - maximum motor velocity
MOTOR_NOISE_STD = 0.001           # rad - position measurement noise


# =============================================================================
# Motor Simulator
# =============================================================================

@dataclass
class MotorState:
    """Simulated motor state."""
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    target: float = 0.0
    target_vel: float = 0.0
    kp: float = 10.0
    kd: float = 0.5
    enabled: bool = False
    
    def update(self, dt: float) -> None:
        """Update motor state based on PD control simulation."""
        if not self.enabled:
            # When disabled, gradually stop
            self.velocity *= 0.9
            self.position += self.velocity * dt
            self.torque = 0.0
            return
        
        # Simple PD control simulation
        pos_error = self.target - self.position
        vel_error = self.target_vel - self.velocity
        
        # Compute desired torque (simplified)
        self.torque = self.kp * pos_error + self.kd * vel_error
        
        # Apply torque to velocity (with time constant)
        accel = self.torque * 10.0  # Simplified inertia
        self.velocity += accel * dt
        
        # Clamp velocity
        self.velocity = np.clip(self.velocity, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY)
        
        # Update position
        self.position += self.velocity * dt
        
        # Apply low-pass filter (simulating motor dynamics)
        alpha = dt / (dt + MOTOR_TIME_CONSTANT)
        self.velocity = (1 - alpha) * self.velocity + alpha * self.target_vel


# =============================================================================
# Dummy ESP32 Client
# =============================================================================

class DummyESP32Client:
    """Simulates an ESP32 motor controller client."""
    
    def __init__(
        self,
        module_id: int,
        server_ip: str = DEFAULT_SERVER_IP,
        server_port: int = DEFAULT_SERVER_PORT,
        command_port: int = DEFAULT_COMMAND_PORT,
    ):
        """Initialize the dummy ESP32 client.
        
        Args:
            module_id: Unique identifier for this module (1-254)
            server_ip: IP address of the server
            server_port: Port the server listens on for feedback
            command_port: Port this client listens on for commands
        """
        self.module_id = module_id
        self.server_ip = server_ip
        self.server_port = server_port
        self.command_port = command_port
        
        # Use unique loopback IP for this client (127.0.0.x)
        self.client_ip = f"127.0.0.{module_id}"
        
        # Motor state
        self.motor = MotorState()
        
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
        self.last_cmd_time = 0.0
        
        # Timestamps
        self._start_time = time.time()
        self._last_update = time.time()
    
    def start(self) -> None:
        """Start the client."""
        # Create sending socket (binds to unique loopback IP)
        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            # Bind to our unique IP so server sees us as a distinct device
            self._send_socket.bind((self.client_ip, 0))
        except OSError as e:
            print(f"[Module {self.module_id}] Warning: Could not bind to {self.client_ip}: {e}")
            print(f"[Module {self.module_id}] Falling back to default interface")
            self._send_socket.bind(("", 0))
        
        # Create receiving socket for commands
        self._recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Try to enable SO_REUSEPORT for multiple clients on same port
        try:
            self._recv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass  # Not available on all platforms
        
        try:
            # Bind to our unique IP on the command port
            self._recv_socket.bind((self.client_ip, self.command_port))
            print(f"[Module {self.module_id}] Listening for commands on {self.client_ip}:{self.command_port}")
        except OSError as e:
            print(f"[Module {self.module_id}] Warning: Could not bind to {self.client_ip}:{self.command_port}: {e}")
            # Try binding to all interfaces with SO_REUSEPORT
            try:
                self._recv_socket.bind(("0.0.0.0", self.command_port))
                print(f"[Module {self.module_id}] Listening on 0.0.0.0:{self.command_port}")
            except OSError as e2:
                print(f"[Module {self.module_id}] Error: Could not bind receive socket: {e2}")
                raise
        
        self._recv_socket.settimeout(0.1)
        
        # Start receive thread
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()
        
        print(f"[Module {self.module_id}] Started - sending to {self.server_ip}:{self.server_port}")
    
    def stop(self) -> None:
        """Stop the client."""
        self._running = False
        
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        
        if self._send_socket:
            self._send_socket.close()
        
        if self._recv_socket:
            self._recv_socket.close()
        
        print(f"[Module {self.module_id}] Stopped (sent={self.send_count}, recv={self.recv_count})")
    
    def _receive_loop(self) -> None:
        """Background thread for receiving commands."""
        while self._running:
            try:
                data, addr = self._recv_socket.recvfrom(1024)
                
                # Check size
                if len(data) < MotorCommand._SIZE:
                    continue
                
                # Deserialize command
                cmd = MotorCommand.deserialize(data)
                
                # Update motor state from command
                with self._lock:
                    self.motor.target = cmd.target
                    self.motor.target_vel = cmd.target_vel
                    self.motor.kp = cmd.kp
                    self.motor.kd = cmd.kd
                    self.motor.enabled = (cmd.switch_ == 1)
                    self.last_cmd_time = cmd.timestamp
                    self.recv_count += 1
                
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"[Module {self.module_id}] Receive error: {e}")
    
    def update(self) -> None:
        """Update motor simulation and send feedback."""
        now = time.time()
        dt = now - self._last_update
        self._last_update = now
        
        # Update motor simulation
        with self._lock:
            self.motor.update(dt)
            motor_state = MotorState(
                position=self.motor.position,
                velocity=self.motor.velocity,
                torque=self.motor.torque,
                target=self.motor.target,
                enabled=self.motor.enabled,
            )
        
        # Create feedback message
        motor_data = MotorData(
            pos=motor_state.position + np.random.normal(0, MOTOR_NOISE_STD),
            large_pos=motor_state.position,
            vel=motor_state.velocity,
            torque=motor_state.torque,
            voltage=24.0,  # Simulated voltage
            current=abs(motor_state.torque) * 0.1,  # Simulated current
            temperature=45,  # Simulated temperature
            motor_error=0,  # Motor error flags
            motor_mode=2 if motor_state.enabled else 0,  # Mode: 0=Off, 1=Cal, 2=On
            driver_error=0,  # Driver chip error
        )
        
        imu_data = IMUData()  # Default zeros
        error_data = ErrorData(reset_reason0=0, reset_reason1=0)
        uwb_data = UWBDistances()  # Default zeros
        
        # Create SensorData message
        msg = SensorData(
            module_id=self.module_id,
            receive_dt=int(dt * 1e6),  # microseconds
            timestamp=int((now - self._start_time) * 1e6),  # microseconds
            switch_off=0 if motor_state.enabled else 1,
            last_rcv_timestamp=self.last_cmd_time,
            info=0,
            motor=motor_data,
            imu=imu_data,
            error=error_data,
            goal_distance=0.233,  # Simulated goal distance
            uwb=uwb_data,
        )
        
        # Send to server
        try:
            data = msg.serialize()
            self._send_socket.sendto(data, (self.server_ip, self.server_port))
            self.send_count += 1
        except Exception as e:
            print(f"[Module {self.module_id}] Send error: {e}")
    
    def run(self, rate_hz: float = FEEDBACK_RATE) -> None:
        """Run the client main loop.
        
        Args:
            rate_hz: Feedback rate in Hz
        """
        period = 1.0 / rate_hz
        
        try:
            while self._running:
                loop_start = time.time()
                
                self.update()
                
                # Sleep to maintain rate
                elapsed = time.time() - loop_start
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            pass


# =============================================================================
# Multi-Client Launcher
# =============================================================================

def launch_multiple_clients(
    num_clients: int,
    server_ip: str = DEFAULT_SERVER_IP,
    server_port: int = DEFAULT_SERVER_PORT,
    command_port: int = DEFAULT_COMMAND_PORT,
    rate_hz: float = FEEDBACK_RATE,
) -> None:
    """Launch multiple dummy ESP32 clients in threads.
    
    Args:
        num_clients: Number of clients to launch
        server_ip: Server IP address
        server_port: Server port for feedback
        command_port: Port for receiving commands
        rate_hz: Feedback rate in Hz
    """
    clients = []
    threads = []
    
    print(f"Launching {num_clients} dummy ESP32 clients...")
    print(f"Server: {server_ip}:{server_port}")
    print(f"Command port: {command_port}")
    print()
    
    # Create and start clients
    for i in range(num_clients):
        client = DummyESP32Client(
            module_id=i,
            server_ip=server_ip,
            server_port=server_port,
            command_port=command_port,
        )
        client.start()
        clients.append(client)
        
        # Run client in thread
        thread = threading.Thread(target=client.run, args=(rate_hz,), daemon=True)
        thread.start()
        threads.append(thread)
    
    print()
    print("=" * 60)
    print(f"All {num_clients} clients running. Press Ctrl+C to stop.")
    print("=" * 60)
    
    # Wait for interrupt
    try:
        while True:
            time.sleep(1.0)
            
            # Print status
            status_parts = []
            for client in clients:
                status_parts.append(
                    f"M{client.module_id}: pos={client.motor.position:.2f} "
                    f"tx={client.send_count} rx={client.recv_count}"
                )
            print("\r" + " | ".join(status_parts), end="", flush=True)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    
    # Stop all clients
    for client in clients:
        client._running = False
    
    for client in clients:
        client.stop()
    
    print("Done.")


# =============================================================================
# Entry Point
# =============================================================================

def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Dummy ESP32 Client for Testing",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    
    parser.add_argument(
        "--module-id", "-m",
        type=int,
        default=1,
        help="Module ID (1-254). Also determines loopback IP (127.0.0.x)",
    )
    
    parser.add_argument(
        "--server-ip", "-s",
        type=str,
        default=DEFAULT_SERVER_IP,
        help="Server IP address",
    )
    
    parser.add_argument(
        "--server-port", "-p",
        type=int,
        default=DEFAULT_SERVER_PORT,
        help="Server port for sending feedback",
    )
    
    parser.add_argument(
        "--command-port", "-c",
        type=int,
        default=DEFAULT_COMMAND_PORT,
        help="Port to listen for commands",
    )
    
    parser.add_argument(
        "--rate", "-r",
        type=float,
        default=FEEDBACK_RATE,
        help="Feedback rate in Hz",
    )
    
    parser.add_argument(
        "--launch", "-l",
        type=int,
        default=0,
        help="Launch N clients in threads (0 = single client mode)",
    )
    
    return parser.parse_args()


def main() -> None:
    """Main entry point."""
    args = parse_arguments()
    
    # Validate module_id
    if args.module_id < 1 or args.module_id > 254:
        print("Error: module-id must be between 1 and 254")
        sys.exit(1)
    
    # Multi-client launcher mode
    if args.launch > 0:
        launch_multiple_clients(
            num_clients=args.launch,
            server_ip=args.server_ip,
            server_port=args.server_port,
            command_port=args.command_port,
            rate_hz=args.rate,
        )
        return
    
    # Single client mode
    print("=" * 60)
    print(f"Dummy ESP32 Client - Module ID: {args.module_id}")
    print("=" * 60)
    print(f"Client IP: 127.0.0.{args.module_id}")
    print(f"Server: {args.server_ip}:{args.server_port}")
    print(f"Command port: {args.command_port}")
    print(f"Feedback rate: {args.rate} Hz")
    print()
    print("Press Ctrl+C to stop")
    print("=" * 60)
    print()
    
    # Create and run client
    client = DummyESP32Client(
        module_id=args.module_id,
        server_ip=args.server_ip,
        server_port=args.server_port,
        command_port=args.command_port,
    )
    
    # Handle signals
    def signal_handler(signum, frame):
        print(f"\n[Signal] Received signal {signum}")
        client._running = False
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        client.start()
        client.run(rate_hz=args.rate)
    finally:
        client.stop()
        
        # Print summary
        print()
        print("=" * 60)
        print("Summary:")
        print(f"  Module ID: {args.module_id}")
        print(f"  Feedback sent: {client.send_count}")
        print(f"  Commands received: {client.recv_count}")
        print(f"  Final position: {client.motor.position:.4f} rad")
        print("=" * 60)


if __name__ == "__main__":
    main()




