#!/usr/bin/env python3
"""
ESP32 Pub/Sub Companion Script

This script runs on the server (PC) and communicates with ESP32 modules
using the pub/sub pattern. It demonstrates bidirectional communication:
- Server sends commands to ESP32 (MotorCommand)
- ESP32 sends feedback to server (SensorData)

Usage:
    python examples/esp32_companion.py

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import socket
import struct
import threading
from dataclasses import dataclass

# Add parent directory for development
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import capybarish as cpy
from capybarish.pubsub import Node, Rate, QoSProfile, spin_once, init, shutdown, ok

# Import generated messages
from capybarish.generated import MotorCommand, SensorData, MotorData, IMUData


# =============================================================================
# Configuration
# =============================================================================

SERVER_IP = "0.0.0.0"  # Listen on all interfaces
SERVER_PORT = 6666      # Port to receive feedback from ESP32
SEND_PORT = 6666        # Port ESP32 listens on

# ESP32 module addresses (update with your ESP32 IPs)
ESP32_MODULES = [
    ("192.168.1.101", SEND_PORT),  # Module 1
    # ("192.168.1.102", SEND_PORT),  # Module 2
    # Add more modules as needed
]

CONTROL_RATE = 100.0  # Hz


# =============================================================================
# Feedback Receiver (UDP Server)
# =============================================================================

class FeedbackReceiver:
    """Receives feedback from ESP32 modules."""
    
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.thread = None
        
        # Latest feedback from each module (keyed by IP)
        self.feedback: dict[str, SensorData] = {}
        self.lock = threading.Lock()
        
        # Statistics
        self.recv_count = 0
        
    def start(self):
        """Start the receiver thread."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(0.1)
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        
        print(f"[FeedbackReceiver] Listening on {self.host}:{self.port}")
    
    def stop(self):
        """Stop the receiver."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.socket:
            self.socket.close()
    
    def _receive_loop(self):
        """Background thread for receiving feedback."""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(4096)
                
                # Deserialize feedback
                if len(data) >= SensorData._SIZE:
                    feedback = SensorData.deserialize(data)
                    
                    with self.lock:
                        self.feedback[addr[0]] = feedback
                        self.recv_count += 1
                        
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[FeedbackReceiver] Error: {e}")
    
    def get_feedback(self, ip: str) -> SensorData | None:
        """Get latest feedback from a module."""
        with self.lock:
            return self.feedback.get(ip)
    
    def get_all_feedback(self) -> dict[str, SensorData]:
        """Get feedback from all modules."""
        with self.lock:
            return dict(self.feedback)


# =============================================================================
# Command Sender
# =============================================================================

class CommandSender:
    """Sends commands to ESP32 modules."""
    
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_count = 0
    
    def send(self, cmd: MotorCommand, address: tuple[str, int]) -> bool:
        """Send command to an ESP32 module."""
        try:
            data = cmd.serialize()
            self.socket.sendto(data, address)
            self.send_count += 1
            return True
        except Exception as e:
            print(f"[CommandSender] Error sending to {address}: {e}")
            return False
    
    def broadcast(self, cmd: MotorCommand, addresses: list[tuple[str, int]]):
        """Send command to multiple modules."""
        for addr in addresses:
            self.send(cmd, addr)


# =============================================================================
# Main Controller (using Pub/Sub Node)
# =============================================================================

def main():
    print("\n" + "=" * 60)
    print("ESP32 Pub/Sub Companion - Server Side")
    print("=" * 60 + "\n")
    
    # Initialize pub/sub system
    init()
    
    # Create node
    node = Node('esp32_controller')
    logger = node.get_logger()
    
    # Start feedback receiver
    receiver = FeedbackReceiver(SERVER_IP, SERVER_PORT)
    receiver.start()
    
    # Create command sender
    sender = CommandSender()
    
    # Control state
    target_pos = 0.0
    direction = 1
    
    # Create 100 Hz control timer
    rate = Rate(CONTROL_RATE)
    
    logger.info(f"Sending commands to {len(ESP32_MODULES)} module(s)")
    logger.info(f"Control rate: {CONTROL_RATE} Hz")
    logger.info("Press Ctrl+C to stop\n")
    
    start_time = time.time()
    loop_count = 0
    last_print = time.time()
    
    try:
        while ok():
            # Generate command (simple oscillation)
            target_pos += direction * 0.01
            if target_pos > 1.0:
                direction = -1
            elif target_pos < -1.0:
                direction = 1
            
            # Create command message
            cmd = MotorCommand(
                target=target_pos,
                target_vel=0.0,
                kp=10.0,
                kd=0.5,
                enable_filter=1,
                switch_=1,
                calibrate=0,
                restart=0,
                timestamp=time.time() - start_time,
            )
            
            # Send to all modules
            sender.broadcast(cmd, ESP32_MODULES)
            
            loop_count += 1
            
            # Print status every second
            if time.time() - last_print >= 1.0:
                print(f"\r[{loop_count:6d}] cmd: {sender.send_count:6d}, "
                      f"fb: {receiver.recv_count:6d}, "
                      f"target: {target_pos:+.3f}", end="")
                
                # Print feedback from each module
                for ip, fb in receiver.get_all_feedback().items():
                    print(f" | {ip}: pos={fb.motor.pos:+.3f}", end="")
                
                print("", flush=True)
                last_print = time.time()
            
            # Maintain rate
            rate.sleep()
            
    except KeyboardInterrupt:
        print("\n\n[Interrupted]")
    
    finally:
        # Cleanup
        receiver.stop()
        shutdown()
        
        # Print final stats
        elapsed = time.time() - start_time
        print(f"\n{'=' * 60}")
        print(f"Final Statistics:")
        print(f"  Runtime: {elapsed:.1f}s")
        print(f"  Commands sent: {sender.send_count}")
        print(f"  Feedback received: {receiver.recv_count}")
        print(f"  Effective rate: {loop_count / elapsed:.1f} Hz")
        print(f"{'=' * 60}\n")


# =============================================================================
# Alternative: Pure Pub/Sub Implementation
# =============================================================================

def main_pubsub():
    """
    Alternative implementation using pure pub/sub with network binding.
    
    This version uses the capybarish pub/sub system directly for both
    sending and receiving, with UDP network transport.
    """
    print("\n" + "=" * 60)
    print("ESP32 Pub/Sub Companion (Pure Pub/Sub Mode)")
    print("=" * 60 + "\n")
    
    init()
    
    node = Node('esp32_controller')
    logger = node.get_logger()
    
    # Track received feedback
    feedback_data = {}
    
    def on_feedback(msg: SensorData):
        # In real usage, you'd need to track which module sent this
        feedback_data['latest'] = msg
        logger.info(f"Feedback: pos={msg.motor.pos:.3f}, vel={msg.motor.vel:.3f}")
    
    # Create publisher for commands (local pub, will send to network)
    cmd_pub = node.create_publisher(MotorCommand, '/motor/command', qos_depth=1)
    
    # Create subscriber for feedback
    fb_sub = node.create_subscription(SensorData, '/motor/feedback', on_feedback, qos_depth=10)
    
    # For network communication, we need to:
    # 1. Add remote endpoint to publisher
    # 2. Bind subscriber to network port
    for addr in ESP32_MODULES:
        cmd_pub.add_remote_endpoint(addr[0], addr[1])
    
    fb_sub.bind_network(SERVER_IP, SERVER_PORT)
    
    logger.info("Publishers and subscribers created")
    
    # Control loop
    rate = Rate(CONTROL_RATE)
    target = 0.0
    direction = 1
    
    try:
        while ok():
            target += direction * 0.01
            if abs(target) > 1.0:
                direction *= -1
            
            cmd = MotorCommand(
                target=target,
                target_vel=0.0,
                kp=10.0,
                kd=0.5,
                switch_=1,
                timestamp=time.time(),
            )
            
            cmd_pub.publish(cmd)
            spin_once(node)
            rate.sleep()
            
    except KeyboardInterrupt:
        print("\n[Interrupted]")
    
    finally:
        node.destroy()
        shutdown()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='ESP32 Pub/Sub Companion')
    parser.add_argument('--mode', choices=['direct', 'pubsub'], default='direct',
                        help='Communication mode (default: direct)')
    args = parser.parse_args()
    
    if args.mode == 'pubsub':
        main_pubsub()
    else:
        main()
