#!/usr/bin/env python3
"""
Pub/Sub Example - ROS2-like Communication Pattern

This example demonstrates how to use capybarish's pub/sub system
which provides a familiar ROS2-style API for communication.

Run this example:
    python examples/pubsub_example.py
"""

import time
import sys
import os

# Add parent directory for development
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import capybarish as cpy
from capybarish.pubsub import (
    Node,
    Publisher,
    Subscription,
    Timer,
    Rate,
    QoSProfile,
    spin,
    spin_once,
    ok,
    init,
    shutdown,
    get_topic_names_and_types,
    get_node_names,
)

# Import generated message types
from capybarish.generated import MotorCommand, SensorData, MotorData, IMUData


# =============================================================================
# Example 1: Simple Publisher/Subscriber
# =============================================================================

def simple_pubsub_example():
    """Basic publisher/subscriber example."""
    print("\n" + "=" * 60)
    print("Example 1: Simple Publisher/Subscriber")
    print("=" * 60)
    
    # Create a node
    node = Node('simple_example')
    logger = node.get_logger()
    
    # Track received messages
    received_messages = []
    
    def callback(msg: MotorCommand):
        received_messages.append(msg)
        logger.info(f"Received: target={msg.target:.2f}, vel={msg.target_vel:.2f}")
    
    # Create publisher and subscriber on the same topic
    pub = node.create_publisher(MotorCommand, '/motor/command', qos_depth=10)
    sub = node.create_subscription(MotorCommand, '/motor/command', callback, qos_depth=10)
    
    # Publish some messages
    for i in range(5):
        msg = MotorCommand(
            target=float(i) * 0.5,
            target_vel=1.0,
            kp=10.0,
            kd=0.5,
            enable_filter=1,
            switch_=1,
            calibrate=0,
            restart=0,
            timestamp=time.time(),
        )
        pub.publish(msg)
        logger.info(f"Published: target={msg.target:.2f}")
    
    # Process callbacks
    spin_once(node)
    
    print(f"\nReceived {len(received_messages)} messages")
    
    # Cleanup
    node.destroy()


# =============================================================================
# Example 2: Timer-based Publishing
# =============================================================================

def timer_publishing_example():
    """Timer-based publishing at fixed rate."""
    print("\n" + "=" * 60)
    print("Example 2: Timer-based Publishing (100Hz for 0.5s)")
    print("=" * 60)
    
    node = Node('timer_publisher')
    logger = node.get_logger()
    
    pub = node.create_publisher(MotorCommand, '/motor/command')
    
    publish_count = [0]  # Use list for closure
    
    def timer_callback():
        msg = MotorCommand(
            target=float(publish_count[0]) * 0.01,
            target_vel=0.0,
            kp=10.0,
            kd=0.5,
            timestamp=time.time(),
        )
        pub.publish(msg)
        publish_count[0] += 1
    
    # Create 100Hz timer
    timer = node.create_timer(0.01, timer_callback)
    
    # Run for 0.5 seconds
    start_time = time.time()
    while time.time() - start_time < 0.5:
        spin_once(node)
        time.sleep(0.001)
    
    logger.info(f"Published {publish_count[0]} messages in 0.5s")
    print(f"Effective rate: {publish_count[0] / 0.5:.1f} Hz")
    
    node.destroy()


# =============================================================================
# Example 3: Multiple Nodes Communication
# =============================================================================

def multi_node_example():
    """Multiple nodes communicating via topics."""
    print("\n" + "=" * 60)
    print("Example 3: Multiple Nodes Communication")
    print("=" * 60)
    
    # Controller node - sends commands
    controller = Node('controller')
    cmd_pub = controller.create_publisher(MotorCommand, '/robot/command')
    
    # Robot node - receives commands, sends feedback
    robot = Node('robot')
    feedback_pub = robot.create_publisher(SensorData, '/robot/feedback')
    
    command_count = [0]
    feedback_count = [0]
    
    def command_callback(msg: MotorCommand):
        command_count[0] += 1
        robot.get_logger().info(f"Robot received command: target={msg.target:.2f}")
        
        # Send feedback
        feedback = SensorData(
            motor=MotorData(
                pos=msg.target * 0.9,  # Simulated position
                vel=msg.target_vel,
                torque=0.5,
            ),
            imu=IMUData(),
            timestamp=time.time(),
        )
        feedback_pub.publish(feedback)
    
    def feedback_callback(msg: SensorData):
        feedback_count[0] += 1
        controller.get_logger().info(f"Controller received feedback: pos={msg.motor.pos:.2f}")
    
    # Create subscriptions
    robot.create_subscription(MotorCommand, '/robot/command', command_callback)
    controller.create_subscription(SensorData, '/robot/feedback', feedback_callback)
    
    # Send commands
    for i in range(3):
        cmd = MotorCommand(target=float(i + 1), target_vel=0.5, kp=10.0, kd=0.5)
        cmd_pub.publish(cmd)
        
        # Process both nodes
        spin_once(controller)
        spin_once(robot)
        spin_once(controller)  # Process feedback
    
    print(f"\nCommands sent/received: 3/{command_count[0]}")
    print(f"Feedbacks sent/received: {command_count[0]}/{feedback_count[0]}")
    
    controller.destroy()
    robot.destroy()


# =============================================================================
# Example 4: QoS Profiles
# =============================================================================

def qos_example():
    """Quality of Service configuration example."""
    print("\n" + "=" * 60)
    print("Example 4: QoS Profiles")
    print("=" * 60)
    
    node = Node('qos_example')
    
    # Sensor data QoS - best effort, small queue (for high-frequency data)
    sensor_qos = QoSProfile.sensor_data()
    print(f"Sensor QoS: reliability={sensor_qos.reliability.name}, depth={sensor_qos.depth}")
    
    # Default QoS - reliable delivery
    default_qos = QoSProfile.default()
    print(f"Default QoS: reliability={default_qos.reliability.name}, depth={default_qos.depth}")
    
    # Create publishers with different QoS
    imu_pub = node.create_publisher(
        IMUData, 
        '/imu/data', 
        qos_profile=QoSProfile.sensor_data()
    )
    
    cmd_pub = node.create_publisher(
        MotorCommand, 
        '/motor/command', 
        qos_profile=QoSProfile.default()
    )
    
    print(f"\nCreated IMU publisher (sensor_data QoS)")
    print(f"Created command publisher (default QoS)")
    
    node.destroy()


# =============================================================================
# Example 5: Topic Introspection
# =============================================================================

def introspection_example():
    """Topic and node introspection example."""
    print("\n" + "=" * 60)
    print("Example 5: Topic Introspection")
    print("=" * 60)
    
    # Create some nodes with publishers/subscribers
    node1 = Node('sensor_node')
    node2 = Node('control_node')
    
    node1.create_publisher(IMUData, '/sensors/imu')
    node1.create_publisher(MotorData, '/sensors/motor')
    
    node2.create_subscription(IMUData, '/sensors/imu', lambda m: None)
    node2.create_publisher(MotorCommand, '/control/command')
    
    # List all topics
    print("\nRegistered Topics:")
    for name, type_name in get_topic_names_and_types():
        print(f"  {name} [{type_name}]")
    
    # List all nodes
    print("\nRegistered Nodes:")
    for name in get_node_names():
        print(f"  {name}")
    
    node1.destroy()
    node2.destroy()


# =============================================================================
# Example 6: Rate-controlled Loop
# =============================================================================

def rate_example():
    """Rate-controlled main loop example."""
    print("\n" + "=" * 60)
    print("Example 6: Rate-controlled Loop (50Hz for 0.2s)")
    print("=" * 60)
    
    node = Node('rate_example')
    pub = node.create_publisher(MotorCommand, '/command')
    
    rate = Rate(50)  # 50 Hz
    count = 0
    start = time.time()
    
    while time.time() - start < 0.2:  # Run for 0.2 seconds
        msg = MotorCommand(target=float(count) * 0.1)
        pub.publish(msg)
        count += 1
        
        spin_once(node)
        rate.sleep()
    
    elapsed = time.time() - start
    actual_rate = count / elapsed
    print(f"Target: 50 Hz, Actual: {actual_rate:.1f} Hz")
    print(f"Published {count} messages in {elapsed:.3f}s")
    
    node.destroy()


# =============================================================================
# Example 7: Namespace Usage
# =============================================================================

def namespace_example():
    """Node namespace example."""
    print("\n" + "=" * 60)
    print("Example 7: Namespaces")
    print("=" * 60)
    
    # Create nodes with namespaces
    robot1 = Node('controller', namespace='robot1')
    robot2 = Node('controller', namespace='robot2')
    
    # These publish to different topics despite same relative name
    pub1 = robot1.create_publisher(MotorCommand, 'command')  # -> /robot1/command
    pub2 = robot2.create_publisher(MotorCommand, 'command')  # -> /robot2/command
    
    print(f"Robot1 publishes to: {pub1.topic_name}")
    print(f"Robot2 publishes to: {pub2.topic_name}")
    
    # Absolute topic names ignore namespace
    global_pub = robot1.create_publisher(MotorCommand, '/global/command')
    print(f"Global publisher topic: {global_pub.topic_name}")
    
    robot1.destroy()
    robot2.destroy()


# =============================================================================
# Example 8: Context Manager Usage
# =============================================================================

def context_manager_example():
    """Using node as context manager for automatic cleanup."""
    print("\n" + "=" * 60)
    print("Example 8: Context Manager")
    print("=" * 60)
    
    with Node('context_example') as node:
        pub = node.create_publisher(MotorCommand, '/test')
        
        for i in range(3):
            msg = MotorCommand(target=float(i))
            pub.publish(msg)
            print(f"Published message {i}")
        
        spin_once(node)
    
    print("Node automatically destroyed on exit")


# =============================================================================
# Main
# =============================================================================

def main():
    """Run all examples."""
    print("\n" + "=" * 60)
    print("Capybarish Pub/Sub Examples (ROS2-like API)")
    print("=" * 60)
    
    # Initialize pub/sub system
    init()
    
    try:
        simple_pubsub_example()
        timer_publishing_example()
        multi_node_example()
        qos_example()
        introspection_example()
        rate_example()
        namespace_example()
        context_manager_example()
        
        print("\n" + "=" * 60)
        print("All examples completed successfully!")
        print("=" * 60)
        
    finally:
        # Cleanup
        shutdown()


if __name__ == '__main__':
    main()
