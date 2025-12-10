#!/usr/bin/env python3
"""
Basic Usage Example with Pub/Sub API and Rich Dashboard.

This example demonstrates how to:
- Use the NetworkServer API for ESP32 communication
- Use the modular RichDashboard for real-time status monitoring
- Send sinusoidal control commands to motors
- Handle real-time keyboard input for motor control

The example runs a continuous control loop that sends sinusoidal position
commands to all discovered ESP32 devices while displaying real-time
status in a Rich-based terminal dashboard.

Keyboard Controls:
    'e': Enable motors (switch_on = 1)
    'd': Disable motors (switch_on = 0)
    'q': Quit the program
    Ctrl+C: Exit the program

Usage:
    python basic_usage_pubsub.py
    python basic_usage_pubsub.py --port 6666
    python basic_usage_pubsub.py --help

Requirements:
    - ESP32 devices configured to send data to this server
    - Network connectivity to ESP32 devices

Copyright 2025 Chen Yu <chenyu@u.northwestern.edu>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
"""

import argparse
import signal
import sys
import time
from typing import Optional

import numpy as np

# Import capybarish components
import capybarish as cpy
from capybarish.pubsub import NetworkServer, Rate
from capybarish.generated import ReceivedData, SentData
from capybarish.dashboard import MotorDashboard, DashboardConfig
from capybarish.kbhit import KBHit


# =============================================================================
# Configuration
# =============================================================================

DEFAULT_LISTEN_PORT = 6666    # Port to receive feedback from ESP32s
DEFAULT_COMMAND_PORT = 6667   # Port ESP32s listen on for commands

CONTROL_RATE = 50.0           # Hz - control loop frequency
SINUSOIDAL_AMPLITUDE = 0.6    # Amplitude of sinusoidal command
SINUSOIDAL_FREQUENCY = 0.5    # Hz - frequency of sinusoidal command
DEFAULT_KP_GAIN = 8.0
DEFAULT_KD_GAIN = 0.2

DEVICE_TIMEOUT = 2.0          # Seconds before device considered inactive


# =============================================================================
# Global State
# =============================================================================

# Global variables for cleanup
server: Optional[NetworkServer] = None
dashboard: Optional[MotorDashboard] = None
kb: Optional[KBHit] = None
running = True


def signal_handler(signum: int, frame) -> None:
    """Handle interrupt signals for graceful shutdown."""
    global running
    print(f"\n[Signal] Received signal {signum}. Shutting down...")
    running = False


def cleanup() -> None:
    """Perform cleanup operations."""
    global server, dashboard, kb
    
    print("\n[Cleanup] Shutting down...")
    
    # Stop dashboard
    if dashboard:
        try:
            dashboard.stop()
        except Exception as e:
            print(f"[Warning] Dashboard cleanup error: {e}")
    
    # Close server
    if server:
        try:
            server.close()
        except Exception as e:
            print(f"[Warning] Server cleanup error: {e}")
    
    # Restore terminal
    if kb:
        try:
            kb.set_normal_term()
        except Exception as e:
            print(f"[Warning] Keyboard cleanup error: {e}")
    
    print("[Cleanup] Done.")


# =============================================================================
# Callbacks
# =============================================================================

def on_feedback(msg: SentData, sender_ip: str) -> None:
    """Callback when feedback is received from an ESP32.
    
    Args:
        msg: The received SentData message
        sender_ip: IP address of the sender
    """
    global dashboard
    
    if dashboard is None:
        return
    
    # Extract motor data
    motor = msg.motor if hasattr(msg, 'motor') and msg.motor else None
    
    # Check for errors - only show if there's a real error
    error_str = ""
    if hasattr(msg, 'error') and msg.error:
        err = msg.error
        if hasattr(err, 'reset_reason0') and hasattr(err, 'reset_reason1'):
            if err.reset_reason0 != 0 or err.reset_reason1 != 0:
                error_str = f"Reset: {err.reset_reason0}/{err.reset_reason1}"
        elif isinstance(err, (int, float)) and err != 0:
            error_str = f"Error: {err}"
    
    # Update dashboard with motor data
    dashboard.update_motor(
        address=sender_ip,
        name=f"ESP32_{sender_ip.split('.')[-1]}",
        position=motor.pos if motor else 0.0,
        velocity=motor.vel if motor else 0.0,
        torque=motor.torque if motor else 0.0,
        voltage=getattr(msg, 'voltage', 0.0),
        current=getattr(msg, 'current', 0.0),
        mode="Running" if motor and getattr(motor, 'mode', 0) == 2 else "Idle",
        switch=dashboard._switch_on,
        error=error_str,
    )


# =============================================================================
# Main Control Loop
# =============================================================================

def run_control_loop(args: argparse.Namespace) -> None:
    """Run the main control loop.
    
    Args:
        args: Parsed command line arguments
    """
    global server, dashboard, kb, running
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Initialize keyboard handler
        kb = KBHit()
        
        # Create network server
        server = NetworkServer(
            recv_type=SentData,
            send_type=ReceivedData,
            recv_port=args.listen_port,
            send_port=args.command_port,
            callback=on_feedback,
            timeout_sec=DEVICE_TIMEOUT,
        )
        
        # Print info BEFORE starting dashboard (Rich Live will take over console)
        print("=" * 60)
        print("ESP32 Motor Controller - Pub/Sub API + Rich Dashboard")
        print("=" * 60)
        print(f"[Info] Listening for ESP32 feedback on port {args.listen_port}")
        print(f"[Info] Sending commands to ESP32s on port {args.command_port}")
        print()
        print("Controls: 'e' = enable, 'd' = disable, 'q' = quit")
        print("=" * 60)
        print()
        
        # Create dashboard AFTER prints
        config = DashboardConfig(
            title="ESP32 Motor Controller",
            refresh_rate=20,
            timeout_sec=DEVICE_TIMEOUT,
        )
        dashboard = MotorDashboard(config)
        
        # Start dashboard
        dashboard.start()
        
        # Control state
        switch_on = False
        time_step = 0
        start_time = time.time()
        
        # Rate limiter
        rate = Rate(CONTROL_RATE)
        
        # Statistics
        cmd_count = 0
        fb_count = 0
        
        while running:
            loop_start = time.perf_counter()
            
            # Process incoming messages
            received = server.spin_once()
            fb_count += received
            
            # Generate sinusoidal target
            elapsed = time.time() - start_time
            target_pos = SINUSOIDAL_AMPLITUDE * np.sin(
                2 * np.pi * SINUSOIDAL_FREQUENCY * elapsed
            )
            target_vel = SINUSOIDAL_AMPLITUDE * 2 * np.pi * SINUSOIDAL_FREQUENCY * np.cos(
                2 * np.pi * SINUSOIDAL_FREQUENCY * elapsed
            )
            
            # Create command
            cmd = ReceivedData(
                target=target_pos,
                target_vel=target_vel,
                kp=DEFAULT_KP_GAIN,
                kd=DEFAULT_KD_GAIN,
                enable_filter=1,
                switch_=1 if switch_on else 0,
                calibrate=0,
                restart=0,
                timestamp=elapsed,
            )
            
            # Send to all active ESP32s
            sent = server.send_to_all(cmd)
            cmd_count += sent
            
            # Handle keyboard input
            if kb.kbhit():
                key = kb.getch().lower()
                if key == 'e':
                    switch_on = True
                    dashboard.set_switch(True)
                    print("\n[Command] Motors ENABLED")
                elif key == 'd':
                    switch_on = False
                    dashboard.set_switch(False)
                    print("\n[Command] Motors DISABLED")
                elif key == 'q':
                    print("\n[Command] Quit requested")
                    running = False
                    break
            
            # Update performance metrics
            loop_time = time.perf_counter() - loop_start
            dashboard.set_performance(loop_dt=loop_time)
            
            # Update dashboard status
            dashboard.set_status("Cmd/Fb", f"{cmd_count}/{fb_count}")
            
            # Update dashboard display
            dashboard.update()
            
            # Maintain control rate
            rate.sleep()
            
            time_step += 1
        
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup()
        
        # Print summary
        elapsed = time.time() - start_time if 'start_time' in dir() else 0
        print()
        print("=" * 60)
        print("Summary:")
        print(f"  Runtime: {elapsed:.1f}s")
        print(f"  Commands sent: {cmd_count if 'cmd_count' in dir() else 0}")
        print(f"  Feedback received: {fb_count if 'fb_count' in dir() else 0}")
        if server:
            print(f"  Discovered devices: {list(server.devices.keys())}")
        print("=" * 60)


# =============================================================================
# Entry Point
# =============================================================================

def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="ESP32 Motor Controller with Pub/Sub API and Rich Dashboard",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    
    parser.add_argument(
        "--listen-port", "-l",
        type=int,
        default=DEFAULT_LISTEN_PORT,
        help="Port to listen for ESP32 feedback",
    )
    
    parser.add_argument(
        "--command-port", "-c",
        type=int,
        default=DEFAULT_COMMAND_PORT,
        help="Port to send commands to ESP32s",
    )
    
    parser.add_argument(
        "--rate", "-r",
        type=float,
        default=CONTROL_RATE,
        help="Control loop rate in Hz",
    )
    
    parser.add_argument(
        "--amplitude", "-a",
        type=float,
        default=SINUSOIDAL_AMPLITUDE,
        help="Sinusoidal command amplitude",
    )
    
    parser.add_argument(
        "--frequency", "-f",
        type=float,
        default=SINUSOIDAL_FREQUENCY,
        help="Sinusoidal command frequency in Hz",
    )
    
    return parser.parse_args()


def main() -> None:
    """Main entry point."""
    args = parse_arguments()
    
    # Update global config from args
    global CONTROL_RATE, SINUSOIDAL_AMPLITUDE, SINUSOIDAL_FREQUENCY
    CONTROL_RATE = args.rate
    SINUSOIDAL_AMPLITUDE = args.amplitude
    SINUSOIDAL_FREQUENCY = args.frequency
    
    run_control_loop(args)


if __name__ == "__main__":
    main()
