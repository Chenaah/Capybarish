#!/usr/bin/env python3
"""
Inter-Process Communication Example - Publisher Node

This script demonstrates how to create a publisher node that sends commands
to other processes over the network using capybarish's multicast pub/sub system.

Run this alongside subscriber_node.py to see inter-process communication:
    Terminal 1: python examples/publisher_node.py
    Terminal 2: python examples/subscriber_node.py

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import signal
import math

# Add parent directory for development
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import capybarish as cpy
from capybarish.pubsub import Node, init, shutdown, Rate
from capybarish.generated import MotorCommand, SensorData, MotorData, IMUData


class CommandPublisher:
    """Publisher node that sends motor commands."""
    
    def __init__(self):
        self.node = Node('command_publisher')
        self.logger = self.node.get_logger()
        
        # Create publishers for different types of data
        self.motor_cmd_pub = self.node.create_publisher(
            MotorCommand, 
            '/robot/motor_command',
            qos_depth=10
        )
        
        # Add network endpoint for inter-process communication
        # This enables sending to other processes via multicast
        self.motor_cmd_pub.add_remote_endpoint('239.255.0.1', 7001)
        
        # Also create a feedback subscriber to show bidirectional communication
        self.feedback_sub = self.node.create_subscription(
            SensorData,
            '/robot/feedback',
            self.feedback_callback,
            qos_depth=10
        )
        
        # Statistics
        self.command_count = 0
        self.feedback_count = 0
        self.start_time = time.time()
        
        # Command generation
        self.target_amplitude = 2.0
        self.target_frequency = 0.5  # Hz
        
        self.logger.info("Command Publisher Node started")
        self.logger.info(f"Publishing motor commands to: {self.motor_cmd_pub.topic_name}")
        self.logger.info(f"Listening for feedback on: {self.feedback_sub.topic_name}")
        
    def feedback_callback(self, msg: SensorData):
        """Handle feedback from subscriber nodes."""
        self.feedback_count += 1
        self.logger.info(
            f"[{self.feedback_count:3d}] Feedback - "
            f"pos: {msg.motor.pos:6.2f}, "
            f"vel: {msg.motor.vel:6.2f}, "
            f"torque: {msg.motor.torque:6.2f}"
        )
    
    def publish_command(self):
        """Generate and publish a motor command."""
        current_time = time.time() - self.start_time
        
        # Generate sinusoidal target position
        target_pos = self.target_amplitude * math.sin(2 * math.pi * self.target_frequency * current_time)
        target_vel = (2 * math.pi * self.target_frequency * self.target_amplitude * 
                     math.cos(2 * math.pi * self.target_frequency * current_time))
        
        # Create command message
        cmd = MotorCommand(
            target=target_pos,
            target_vel=target_vel,
            kp=10.0,
            kd=2.0,
            enable_filter=1,
            switch_=1,
            calibrate=0,
            restart=0,
            timestamp=time.time()
        )
        
        # Publish the command
        self.motor_cmd_pub.publish(cmd)
        self.command_count += 1
        
        self.logger.info(
            f"[{self.command_count:3d}] Command - "
            f"target: {cmd.target:6.2f}, "
            f"vel: {cmd.target_vel:6.2f}, "
            f"kp: {cmd.kp:4.1f}"
        )
    
    def spin(self):
        """Main loop with rate control."""
        rate = Rate(10.0)  # 10 Hz publishing rate
        
        try:
            while True:
                # Publish command
                self.publish_command()
                
                # Process any incoming messages (feedback)
                cpy.spin_once(self.node)
                
                # Sleep to maintain rate
                rate.sleep()
                
        except KeyboardInterrupt:
            self.logger.info("Shutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        elapsed = time.time() - self.start_time
        self.logger.info(f"Published {self.command_count} commands in {elapsed:.1f}s")
        self.logger.info(f"Received {self.feedback_count} feedback messages")
        self.logger.info(f"Average rate: {self.command_count/elapsed:.1f} Hz")
        
        self.node.destroy()


def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully."""
    print("\nReceived interrupt signal, shutting down...")
    sys.exit(0)


def main():
    """Main function."""
    print("=" * 70)
    print("Inter-Process Communication Example - Publisher Node")
    print("=" * 70)
    print("This node publishes motor commands via multicast UDP.")
    print("Run 'python examples/subscriber_node.py' in another terminal")
    print("to see inter-process communication in action!")
    print("Press Ctrl+C to stop.")
    print("=" * 70)
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize capybarish pub/sub system
    init()
    
    try:
        # Create and run publisher
        publisher = CommandPublisher()
        publisher.spin()
        
    finally:
        shutdown()


if __name__ == '__main__':
    main()