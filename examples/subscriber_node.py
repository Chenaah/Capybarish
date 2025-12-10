#!/usr/bin/env python3
"""
Inter-Process Communication Example - Subscriber Node

This script demonstrates how to create a subscriber node that receives commands
from other processes over the network using capybarish's multicast pub/sub system.

It simulates a robot motor controller that receives commands and sends back feedback.

Run this alongside publisher_node.py to see inter-process communication:
    Terminal 1: python examples/publisher_node.py  
    Terminal 2: python examples/subscriber_node.py

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import signal
import socket
import struct
import threading

# Add parent directory for development
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import capybarish as cpy
from capybarish.pubsub import Node, init, shutdown, Rate
from capybarish.generated import ReceivedData, SentData, MotorData, IMUData


class MotorSimulator:
    """Simple motor simulation for demonstration."""
    
    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.last_update = time.time()
        
    def update(self, target_pos: float, target_vel: float, kp: float, kd: float) -> MotorData:
        """Update motor simulation with PD control."""
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        
        if dt <= 0:
            dt = 0.001
            
        # Simple PD controller
        pos_error = target_pos - self.position
        vel_error = target_vel - self.velocity
        
        # Calculate torque command
        self.torque = kp * pos_error + kd * vel_error
        
        # Simple motor dynamics (first-order)
        motor_constant = 5.0  # Motor response speed
        damping = 2.0
        
        # Update velocity and position
        acceleration = (self.torque * motor_constant - self.velocity * damping) / 1.0
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        
        return MotorData(
            pos=self.position,
            vel=self.velocity,
            torque=self.torque
        )


class RobotSubscriber:
    """Subscriber node that simulates a robot receiving commands."""
    
    def __init__(self):
        self.node = Node('robot_subscriber')
        self.logger = self.node.get_logger()
        
        # Motor simulator
        self.motor = MotorSimulator()
        
        # Create subscriber for motor commands
        self.command_sub = self.node.create_subscription(
            ReceivedData,
            '/robot/motor_command', 
            self.command_callback,
            qos_depth=10
        )
        
        # Create publisher for feedback
        self.feedback_pub = self.node.create_publisher(
            SentData,
            '/robot/feedback',
            qos_depth=10
        )
        
        # Add network endpoint for inter-process communication
        self.feedback_pub.add_remote_endpoint('239.255.0.1', 7002)
        
        # Setup network listener for incoming commands
        self.setup_network_listener()
        
        # Statistics
        self.command_count = 0
        self.feedback_count = 0
        self.start_time = time.time()
        
        self.logger.info("Robot Subscriber Node started")
        self.logger.info(f"Listening for commands on: {self.command_sub.topic_name}")
        self.logger.info(f"Publishing feedback to: {self.feedback_pub.topic_name}")
        
    def setup_network_listener(self):
        """Setup multicast listener for network commands."""
        try:
            # Create multicast socket
            self.network_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.network_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to multicast group
            self.network_socket.bind(('', 7001))
            
            # Join multicast group
            mreq = struct.pack("4sl", socket.inet_aton('239.255.0.1'), socket.INADDR_ANY)
            self.network_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            
            # Start network listening thread
            self.network_running = True
            self.network_thread = threading.Thread(target=self.network_listener, daemon=True)
            self.network_thread.start()
            
            self.logger.info("Network listener started on multicast group 239.255.0.1:7001")
            
        except Exception as e:
            self.logger.warning(f"Failed to setup network listener: {e}")
            self.network_socket = None
    
    def network_listener(self):
        """Listen for network messages in separate thread."""
        while self.network_running:
            try:
                if hasattr(self.network_socket, 'settimeout'):
                    self.network_socket.settimeout(1.0)  # 1 second timeout
                
                data, addr = self.network_socket.recvfrom(1024)
                
                # Try to deserialize as ReceivedData
                try:
                    if hasattr(ReceivedData, 'deserialize'):
                        msg = ReceivedData.deserialize(data)
                        self.logger.info(f"Received network command from {addr[0]}")
                        self.process_command(msg)
                except:
                    # Fallback: create message from raw data if needed
                    pass
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.network_running:
                    self.logger.warning(f"Network error: {e}")
                break
    
    def command_callback(self, msg: ReceivedData):
        """Handle incoming motor commands."""
        self.process_command(msg)
    
    def process_command(self, msg: ReceivedData):
        """Process a motor command and send feedback."""
        self.command_count += 1
        
        self.logger.info(
            f"[{self.command_count:3d}] Command - "
            f"target: {msg.target:6.2f}, "
            f"vel: {msg.target_vel:6.2f}, "
            f"kp: {msg.kp:4.1f}"
        )
        
        # Update motor simulation
        motor_data = self.motor.update(msg.target, msg.target_vel, msg.kp, msg.kd)
        
        # Create feedback message
        feedback = SentData(
            motor=motor_data,
            imu=IMUData(  # Empty IMU data for this example
                accel_x=0.0,
                accel_y=0.0, 
                accel_z=9.81,
                gyro_x=0.0,
                gyro_y=0.0,
                gyro_z=0.0
            ),
            timestamp=time.time()
        )
        
        # Publish feedback
        self.feedback_pub.publish(feedback)
        self.feedback_count += 1
        
        self.logger.info(
            f"[{self.feedback_count:3d}] Feedback - "
            f"pos: {motor_data.pos:6.2f}, "
            f"vel: {motor_data.vel:6.2f}, "
            f"torque: {motor_data.torque:6.2f}"
        )
    
    def spin(self):
        """Main loop."""
        rate = Rate(50.0)  # 50 Hz for processing
        
        try:
            while True:
                # Process any local pub/sub messages
                cpy.spin_once(self.node)
                
                # Sleep to maintain rate
                rate.sleep()
                
        except KeyboardInterrupt:
            self.logger.info("Shutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        self.network_running = False
        
        if hasattr(self, 'network_socket') and self.network_socket:
            try:
                self.network_socket.close()
            except:
                pass
        
        elapsed = time.time() - self.start_time
        self.logger.info(f"Processed {self.command_count} commands in {elapsed:.1f}s")
        self.logger.info(f"Sent {self.feedback_count} feedback messages")
        
        if self.command_count > 0:
            self.logger.info(f"Average rate: {self.command_count/elapsed:.1f} Hz")
        
        self.node.destroy()


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully."""
    print("\nReceived interrupt signal, shutting down...")
    sys.exit(0)


def main():
    """Main function."""
    print("=" * 70)
    print("Inter-Process Communication Example - Subscriber Node")
    print("=" * 70)
    print("This node subscribes to motor commands via multicast UDP.")
    print("Run 'python examples/publisher_node.py' in another terminal")
    print("to send commands to this node!")
    print("Press Ctrl+C to stop.")
    print("=" * 70)
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize capybarish pub/sub system
    init()
    
    try:
        # Create and run subscriber
        subscriber = RobotSubscriber()
        subscriber.spin()
        
    finally:
        shutdown()


if __name__ == '__main__':
    main()