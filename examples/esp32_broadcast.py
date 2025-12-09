#!/usr/bin/env python3
"""
ESP32 Broadcast Companion - No IP Configuration Needed!

This script uses UDP broadcast to communicate with ESP32 devices.
You don't need to know the ESP32's IP address - just run this script
and any ESP32 on the network will receive commands and send feedback.

Like ROS2's auto-discovery, but simpler!

Usage:
    python examples/esp32_broadcast.py

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import socket

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from capybarish.generated import ReceivedData, SentData


# =============================================================================
# Configuration - Ports only, no IPs needed!
# =============================================================================

FEEDBACK_PORT = 6666  # Port ESP32 broadcasts feedback on (we listen here)
COMMAND_PORT = 6667   # Port ESP32 listens on (we broadcast here)

CONTROL_RATE = 100.0  # Hz


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("ESP32 Broadcast Companion")
    print("No IP configuration needed!")
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
    
    print(f"[Info] Listening for feedback on port {FEEDBACK_PORT}")
    print(f"[Info] Broadcasting commands on port {COMMAND_PORT}")
    print()
    
    # Create socket for receiving feedback
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    recv_sock.bind(("0.0.0.0", FEEDBACK_PORT))
    recv_sock.setblocking(False)
    
    # Create socket for sending commands (broadcast)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    # Broadcast address
    BROADCAST_ADDR = ("255.255.255.255", COMMAND_PORT)
    
    print("[Ready] Broadcasting commands and listening for feedback...")
    print("[Ready] Press Ctrl+C to stop")
    print()
    
    # Control state
    target_pos = 0.0
    direction = 1
    
    # Stats
    cmd_count = 0
    fb_count = 0
    discovered_modules = {}  # IP -> last feedback time
    
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
                cmd = ReceivedData(
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
                
                # Broadcast to all devices!
                send_sock.sendto(cmd.serialize(), BROADCAST_ADDR)
                cmd_count += 1
                last_send = now
            
            # Receive feedback (non-blocking)
            try:
                data, addr = recv_sock.recvfrom(4096)
                
                if len(data) >= SentData._SIZE:
                    fb = SentData.deserialize(data)
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
                
                # Clear line and print status
                status = f"\r[{elapsed:6.1f}s] Cmd: {cmd_count:6d} | Fb: {fb_count:6d} | Target: {target_pos:+.3f}"
                
                # Show discovered modules
                active_modules = []
                for ip, info in discovered_modules.items():
                    if now - info['time'] < 2.0:  # Active in last 2 seconds
                        active_modules.append(f"{ip}: pos={info['pos']:+.3f}")
                
                if active_modules:
                    status += f" | Modules: {', '.join(active_modules)}"
                else:
                    status += " | No modules discovered yet..."
                
                print(status, end="", flush=True)
                last_print = now
            
            # Small sleep to prevent busy loop
            time.sleep(0.0001)
            
    except KeyboardInterrupt:
        print("\n\n[Stopped]")
    
    finally:
        recv_sock.close()
        send_sock.close()
        
        # Print summary
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
