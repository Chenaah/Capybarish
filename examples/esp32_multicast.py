#!/usr/bin/env python3
"""
ESP32 Multicast Communication - Works Across Subnets!

Unlike broadcast (which stays within a subnet), multicast can be routed
across subnets if the network supports it. This is how ROS2 DDS works.

Multicast group: 239.255.0.1 (default, like ROS2)
- Any device can join this group
- Messages sent to the group reach all members
- Works across subnets (if network routing allows)

Usage:
    python examples/esp32_multicast.py

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import socket
import struct

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from capybarish.generated import MotorCommand, SensorData


# =============================================================================
# Multicast Configuration
# =============================================================================

# Multicast group address (like ROS2 DDS default)
MULTICAST_GROUP = "239.255.0.1"

# Ports
FEEDBACK_PORT = 6666  # Port for receiving feedback from ESP32
COMMAND_PORT = 6667   # Port ESP32 listens on for commands

CONTROL_RATE = 100.0  # Hz


def create_multicast_receiver(port: int, multicast_group: str = MULTICAST_GROUP) -> socket.socket:
    """
    Create a socket that joins a multicast group to receive messages.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind to all interfaces on the specified port
    sock.bind(("", port))
    
    # Join the multicast group
    mreq = struct.pack("4sl", socket.inet_aton(multicast_group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    
    sock.setblocking(False)
    return sock


def create_multicast_sender(multicast_group: str = MULTICAST_GROUP, ttl: int = 2) -> socket.socket:
    """
    Create a socket for sending to a multicast group.
    
    TTL (Time To Live):
    - 0: Host only
    - 1: Same subnet only
    - 2-31: Can cross routers (if they forward multicast)
    - 32+: Same organization
    - 64+: Same region
    - 128+: Same continent
    - 255: Unrestricted
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    
    # Set TTL (Time To Live) for multicast packets
    # Higher TTL allows packets to cross more routers
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
    
    # Allow loopback (receive our own messages)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    
    return sock


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("ESP32 Multicast Communication")
    print("Works across subnets (unlike broadcast)!")
    print("=" * 60)
    print()
    
    # Show local IP for reference
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        print(f"[Info] Your IP: {local_ip}")
    except:
        local_ip = "unknown"
    
    print(f"[Info] Multicast group: {MULTICAST_GROUP}")
    print(f"[Info] Listening for feedback on port {FEEDBACK_PORT}")
    print(f"[Info] Sending commands to {MULTICAST_GROUP}:{COMMAND_PORT}")
    print()
    
    # Create multicast sockets
    recv_sock = create_multicast_receiver(FEEDBACK_PORT)
    send_sock = create_multicast_sender(ttl=2)  # TTL=2 allows crossing 1 router
    
    MULTICAST_ADDR = (MULTICAST_GROUP, COMMAND_PORT)
    
    print("[Ready] Multicasting commands and listening for feedback...")
    print("[Ready] Press Ctrl+C to stop")
    print()
    
    # Control state
    target_pos = 0.0
    direction = 1
    
    # Stats
    cmd_count = 0
    fb_count = 0
    discovered_modules = {}
    
    start_time = time.time()
    last_print = time.time()
    period = 1.0 / CONTROL_RATE
    last_send = time.time()
    
    try:
        while True:
            now = time.time()
            
            # Send commands at control rate
            if now - last_send >= period:
                # Generate oscillating target
                target_pos += direction * 0.01
                if abs(target_pos) > 1.0:
                    direction *= -1
                
                # Create command
                cmd = MotorCommand(
                    target=target_pos,
                    target_vel=0.0,
                    kp=10.0,
                    kd=0.5,
                    enable_filter=1,
                    switch_=1,
                    calibrate=0,
                    restart=0,
                    timestamp=now - start_time,
                )
                
                # Send to multicast group!
                send_sock.sendto(cmd.serialize(), MULTICAST_ADDR)
                cmd_count += 1
                last_send = now
            
            # Receive feedback (non-blocking)
            try:
                data, addr = recv_sock.recvfrom(4096)
                
                if len(data) >= SensorData._SIZE:
                    fb = SensorData.deserialize(data)
                    fb_count += 1
                    
                    # Track discovered modules
                    module_ip = addr[0]
                    discovered_modules[module_ip] = {
                        'time': now,
                        'pos': fb.motor.pos,
                        'vel': fb.motor.vel,
                    }
                    
            except BlockingIOError:
                pass  # No data available
            
            # Print status every second
            if now - last_print >= 1.0:
                elapsed = now - start_time
                
                status = f"\r[{elapsed:6.1f}s] Cmd: {cmd_count:6d} | Fb: {fb_count:6d} | Target: {target_pos:+.3f}"
                
                # Show discovered modules
                active_modules = []
                for ip, info in discovered_modules.items():
                    if now - info['time'] < 2.0:  # Active in last 2 seconds
                        active_modules.append(f"{ip}: pos={info['pos']:+.3f}")
                
                if active_modules:
                    status += f" | Modules: {', '.join(active_modules)}"
                else:
                    status += " | Waiting for ESP32..."
                
                print(status, end="", flush=True)
                last_print = now
            
            time.sleep(0.0001)
            
    except KeyboardInterrupt:
        print("\n\n[Stopped]")
    
    finally:
        recv_sock.close()
        send_sock.close()
        
        elapsed = time.time() - start_time
        print()
        print("=" * 60)
        print("Summary:")
        print(f"  Runtime: {elapsed:.1f}s")
        print(f"  Commands sent: {cmd_count}")
        print(f"  Feedback received: {fb_count}")
        print(f"  Discovered modules: {list(discovered_modules.keys())}")
        print("=" * 60)


if __name__ == "__main__":
    main()
