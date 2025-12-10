#!/usr/bin/env python3
"""
Server-Client Communication Example - Server Node

This script demonstrates how to create a server that can manage multiple
client connections using capybarish's NetworkServer API. The server:
- Auto-discovers clients that connect to it
- Sends coordinated commands to all clients
- Tracks individual client status and performance
- Provides real-time monitoring of the robot fleet

Run this alongside multiple client_node.py instances:
    Terminal 1: python examples/server_node.py
    Terminal 2: python examples/client_node.py --robot-id robot_1
    Terminal 3: python examples/client_node.py --robot-id robot_2
    Terminal 4: python examples/client_node.py --robot-id robot_3

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import signal
import socket
import math
import threading
from typing import Dict, Set

# Add parent directory for development
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from capybarish.pubsub import NetworkServer, Rate
from capybarish.generated import ReceivedData, SentData, MotorData, IMUData


class RobotFleetServer:
    """Server that manages a fleet of robots using NetworkServer API."""
    
    def __init__(self, listen_port: int = 6666, command_port: int = 6667):
        self.listen_port = listen_port
        self.command_port = command_port
        
        # Create network server
        self.server = NetworkServer(
            recv_type=SentData,
            send_type=ReceivedData,
            recv_port=listen_port,
            send_port=command_port,
            callback=self.on_robot_feedback,
            timeout_sec=3.0,  # 3 seconds timeout for robot connectivity
        )
        
        # Fleet management
        self.robot_names: Dict[str, str] = {}  # ip -> robot_name
        self.newly_discovered: Set[str] = set()
        self.command_count = 0
        self.feedback_count = 0
        self.start_time = time.time()
        
        # Mission parameters
        self.mission_type = "formation"  # "formation", "follow_leader", "scatter"
        self.mission_time = 0.0
        self.formation_radius = 1.5
        self.formation_speed = 0.3  # Hz for rotation
        
        # Display state
        self.display_lock = threading.Lock()
        self.last_display = time.time()
        
        print("=" * 80)
        print("Robot Fleet Management Server")
        print("=" * 80)
        print(f"Server listening on port {listen_port} for robot feedback")
        print(f"Server sending commands on port {command_port}")
        print()
        
        # Show server IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            server_ip = s.getsockname()[0]
            s.close()
            print(f"🖥️  Server IP: {server_ip}")
            print(f"📡 ESP32s should connect to: {server_ip}:{listen_port}")
        except:
            print("❌ Could not determine server IP")
        
        print()
        print("🎯 Mission Types:")
        print("   F - Formation (circular pattern)")
        print("   L - Follow Leader (line formation)")  
        print("   S - Scatter (random positions)")
        print("   Q - Quit")
        print()
        print("🤖 Waiting for ESP32 connections...")
        print("   Configure ESP32 to send feedback to this server")
        print()
        
    def on_robot_feedback(self, msg: SentData, sender_ip: str):
        """Callback when feedback is received from a robot."""
        self.feedback_count += 1
        
        # Check for new robot discovery
        if sender_ip not in self.robot_names:
            # Try to extract robot name from the message if available
            robot_name = f"robot_{len(self.robot_names) + 1}"
            self.robot_names[sender_ip] = robot_name
            self.newly_discovered.add(sender_ip)
            
        # Update display
        self.update_display()
    
    def generate_formation_command(self, robot_index: int, total_robots: int) -> ReceivedData:
        """Generate command for formation mission."""
        if total_robots == 0:
            return ReceivedData(target=0.0, target_vel=0.0, kp=5.0, kd=1.0)
            
        # Circular formation
        angle_offset = (2 * math.pi * robot_index) / total_robots
        current_angle = angle_offset + self.mission_time * self.formation_speed * 2 * math.pi
        
        target_x = self.formation_radius * math.cos(current_angle)
        target_y = self.formation_radius * math.sin(current_angle)
        
        # Convert to target position (using x-coordinate as primary target)
        target_pos = target_x
        target_vel = -self.formation_radius * self.formation_speed * 2 * math.pi * math.sin(current_angle)
        
        return ReceivedData(
            target=target_pos,
            target_vel=target_vel,
            kp=8.0,
            kd=2.0,
            enable_filter=1,
            switch_=1,
            calibrate=0,
            restart=0,
            timestamp=self.mission_time
        )
    
    def generate_follow_leader_command(self, robot_index: int, total_robots: int) -> ReceivedData:
        """Generate command for follow leader mission."""
        if total_robots == 0:
            return ReceivedData(target=0.0, target_vel=0.0, kp=5.0, kd=1.0)
        
        # Leader follows sine wave, followers follow with offset
        leader_pos = 2.0 * math.sin(self.mission_time * 0.5)
        leader_vel = 1.0 * math.cos(self.mission_time * 0.5)
        
        if robot_index == 0:  # Leader
            target_pos = leader_pos
            target_vel = leader_vel
        else:  # Followers
            offset = robot_index * 0.5  # Spacing between robots
            target_pos = leader_pos - offset
            target_vel = leader_vel * 0.8  # Slightly slower
            
        return ReceivedData(
            target=target_pos,
            target_vel=target_vel,
            kp=10.0,
            kd=1.5,
            enable_filter=1,
            switch_=1,
            timestamp=self.mission_time
        )
    
    def generate_scatter_command(self, robot_index: int, total_robots: int) -> ReceivedData:
        """Generate command for scatter mission."""
        # Each robot gets a unique random-ish target
        seed_offset = robot_index * 1234.5
        target_pos = 1.5 * math.sin(self.mission_time * 0.3 + seed_offset)
        target_vel = 0.45 * math.cos(self.mission_time * 0.3 + seed_offset)
        
        return ReceivedData(
            target=target_pos,
            target_vel=target_vel,
            kp=6.0,
            kd=1.0,
            enable_filter=1,
            switch_=1,
            timestamp=self.mission_time
        )
    
    def send_commands_to_fleet(self):
        """Send coordinated commands to all active robots."""
        active_robots = self.server.active_devices
        
        if not active_robots:
            return 0
            
        # Get sorted robot IPs for consistent indexing
        robot_ips = sorted(active_robots.keys())
        total_robots = len(robot_ips)
        
        sent_count = 0
        for i, robot_ip in enumerate(robot_ips):
            # Generate mission-specific command
            if self.mission_type == "formation":
                cmd = self.generate_formation_command(i, total_robots)
            elif self.mission_type == "follow_leader":
                cmd = self.generate_follow_leader_command(i, total_robots)
            elif self.mission_type == "scatter":
                cmd = self.generate_scatter_command(i, total_robots)
            else:
                cmd = ReceivedData(target=0.0, target_vel=0.0, kp=5.0, kd=1.0)
            
            # Send command to this robot
            if self.server.send_to(robot_ip, cmd):
                sent_count += 1
        
        self.command_count += sent_count
        return sent_count
    
    def update_display(self):
        """Update the real-time display."""
        with self.display_lock:
            now = time.time()
            if now - self.last_display < 0.1:  # Limit update rate
                return
            self.last_display = now
            
            # Clear screen and go to top
            print("\033[H\033[2J", end="")
            
            elapsed = now - self.start_time
            active_robots = self.server.active_devices
            
            print("=" * 80)
            print(f"🚀 Robot Fleet Server - Mission: {self.mission_type.upper()} | Runtime: {elapsed:.1f}s")
            print("=" * 80)
            
            # Statistics
            print(f"📊 Commands Sent: {self.command_count:6d} | Feedback Received: {self.feedback_count:6d}")
            print(f"🤖 Active Robots: {len(active_robots):2d} | Total Discovered: {len(self.robot_names)}")
            print()
            
            # Show newly discovered robots
            if self.newly_discovered:
                for ip in list(self.newly_discovered):
                    print(f"🆕 New robot discovered: {self.robot_names[ip]} @ {ip}")
                self.newly_discovered.clear()
                print()
            
            # Robot status table
            if active_robots:
                print("🎯 Robot Fleet Status:")
                print("   Robot Name        IP Address      Position    Velocity    Torque    RX/TX   Last Seen")
                print("   " + "-" * 75)
                
                for i, (ip, device) in enumerate(sorted(active_robots.items())):
                    robot_name = self.robot_names.get(ip, f"robot_{i+1}")
                    
                    if device.last_message:
                        msg = device.last_message
                        if hasattr(msg, 'motor') and msg.motor:
                            motor = msg.motor
                            pos_str = f"{motor.pos:+7.3f}"
                            vel_str = f"{motor.vel:+7.3f}"
                            torque_str = f"{motor.torque:+7.3f}"
                        else:
                            pos_str = vel_str = torque_str = "   N/A  "
                    else:
                        pos_str = vel_str = torque_str = "   N/A  "
                    
                    last_seen = now - device.last_seen
                    rx_tx = f"{device.recv_count:3d}/{device.send_count:3d}"
                    
                    print(f"   {robot_name:<15} {ip:<15} {pos_str} {vel_str} {torque_str}   {rx_tx}   {last_seen:.1f}s")
            else:
                print("⏳ No active ESP32s. Waiting for connections...")
                print("   Configure your ESP32 to connect to this server")
            
            print()
            print("🎮 Commands: [F]ormation [L]eader [S]catter [Q]uit")
    
    def handle_keyboard_input(self):
        """Handle keyboard commands in a separate thread."""
        import select
        import sys
        
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).upper()
                if key == 'F':
                    self.mission_type = "formation"
                    print(f"\n🎯 Mission changed to: FORMATION")
                elif key == 'L':
                    self.mission_type = "follow_leader"
                    print(f"\n🎯 Mission changed to: FOLLOW LEADER")
                elif key == 'S':
                    self.mission_type = "scatter"
                    print(f"\n🎯 Mission changed to: SCATTER")
                elif key == 'Q':
                    print(f"\n👋 Shutting down server...")
                    return
                time.sleep(0.1)
    
    def run(self):
        """Main server loop."""
        # Start keyboard input handler
        # keyboard_thread = threading.Thread(target=self.handle_keyboard_input, daemon=True)
        # keyboard_thread.start()
        print("🎯 Mission set to: FORMATION (keyboard input disabled for ESP32 compatibility)")
        
        # Main control loop
        rate = Rate(20.0)  # 20 Hz server update rate
        
        try:
            while True:
                current_time = time.time()
                self.mission_time = current_time - self.start_time
                
                # Process incoming messages
                self.server.spin_once()
                
                # Send commands to all robots
                self.send_commands_to_fleet()
                
                # Update display periodically
                if current_time - self.last_display > 2.0:  # Reduced frequency for ESP32 compatibility
                    self.update_display()
                
                # Maintain rate
                rate.sleep()
                
        except KeyboardInterrupt:
            print(f"\n\n👋 Server shutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up server resources."""
        elapsed = time.time() - self.start_time
        
        self.server.close()
        
        print()
        print("=" * 80)
        print("📊 Final Statistics:")
        print(f"   Runtime: {elapsed:.1f} seconds")
        print(f"   Commands sent: {self.command_count}")
        print(f"   Feedback received: {self.feedback_count}")
        print(f"   Robots discovered: {len(self.robot_names)}")
        print(f"   Average command rate: {self.command_count/elapsed:.1f} Hz")
        print(f"   Average feedback rate: {self.feedback_count/elapsed:.1f} Hz")
        print("=" * 80)


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully."""
    print("\n🛑 Received interrupt signal, shutting down...")
    sys.exit(0)


def main():
    """Main function."""
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and run server
    server = RobotFleetServer()
    server.run()


if __name__ == '__main__':
    main()