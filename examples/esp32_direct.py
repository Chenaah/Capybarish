#!/usr/bin/env python3
"""
ESP32 Direct IP Communication

For networks where broadcast doesn't work across subnets (like university networks),
use direct IP communication instead.

Usage:
    python examples/esp32_direct.py --esp32 129.105.73.193

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import socket
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from capybarish.generated import MotorCommand, SensorData


# =============================================================================
# Configuration
# =============================================================================

FEEDBACK_PORT = 6666  # Port we listen on (ESP32 sends here)
COMMAND_PORT = 6667   # Port ESP32 listens on (we send here)

CONTROL_RATE = 100.0  # Hz


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="ESP32 Direct Communication")
    parser.add_argument("--esp32", type=str, default="129.105.73.193",
                        help="ESP32 IP address (default: 129.105.73.193)")
    args = parser.parse_args()
    
    ESP32_IP = args.esp32
    
    print("=" * 60)
    print("ESP32 Direct Communication")
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
    
    print(f"[Info] ESP32 IP: {ESP32_IP}")
    print(f"[Info] Listening for feedback on port {FEEDBACK_PORT}")
    print(f"[Info] Sending commands to {ESP32_IP}:{COMMAND_PORT}")
    print()
    
    # Create socket for receiving feedback
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind(("0.0.0.0", FEEDBACK_PORT))
    recv_sock.setblocking(False)
    
    # Create socket for sending commands
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Direct address to ESP32
    ESP32_ADDR = (ESP32_IP, COMMAND_PORT)
    
    print("[Ready] Sending commands and listening for feedback...")
    print("[Ready] Press Ctrl+C to stop")
    print()
    
    # Control state
    target_pos = 0.0
    direction = 1
    
    # Stats
    cmd_count = 0
    fb_count = 0
    last_fb_time = None
    last_fb = None
    
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
                
                # Send directly to ESP32
                send_sock.sendto(cmd.serialize(), ESP32_ADDR)
                cmd_count += 1
                last_send = now
            
            # Receive feedback (non-blocking)
            try:
                data, addr = recv_sock.recvfrom(4096)
                
                if len(data) >= SensorData._SIZE:
                    last_fb = SensorData.deserialize(data)
                    last_fb_time = now
                    fb_count += 1
                    
            except BlockingIOError:
                pass  # No data available
            
            # Print status every second
            if now - last_print >= 1.0:
                elapsed = now - start_time
                
                status = f"\r[{elapsed:6.1f}s] Cmd: {cmd_count:6d} | Fb: {fb_count:6d} | Target: {target_pos:+.3f}"
                
                if last_fb is not None and last_fb_time is not None:
                    age = now - last_fb_time
                    if age < 2.0:
                        status += f" | Pos: {last_fb.motor.pos:+.3f} | Vel: {last_fb.motor.vel:+.3f}"
                    else:
                        status += f" | (no feedback for {age:.1f}s)"
                else:
                    status += " | Waiting for feedback..."
                
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
        if elapsed > 0 and fb_count > 0:
            print(f"  Feedback rate: {fb_count/elapsed:.1f} Hz")
        print("=" * 60)


if __name__ == "__main__":
    main()
