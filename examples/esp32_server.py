#!/usr/bin/env python3
"""
ESP32 Server - Using the Pub/Sub API

This uses the NetworkServer class from capybarish.pubsub which implements
the "reply to sender" pattern:
- Server doesn't need to know ESP32 IPs
- ESP32 sends to server's known IP
- Server auto-discovers ESP32s and replies to each

Usage:
    python examples/esp32_server.py

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import socket

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from capybarish.pubsub import NetworkServer, Rate
from capybarish.generated import ReceivedData, SentData


# =============================================================================
# Configuration
# =============================================================================

LISTEN_PORT = 6666    # Port to receive feedback from ESP32s
COMMAND_PORT = 6667   # Port ESP32s listen on for commands

CONTROL_RATE = 100.0  # Hz


# =============================================================================
# Callback
# =============================================================================

# Track new discoveries
discovered_ips = set()

def on_feedback(msg: SentData, sender_ip: str):
    """Callback when feedback is received from an ESP32."""
    global discovered_ips
    
    if sender_ip not in discovered_ips:
        discovered_ips.add(sender_ip)
        print(f"\n[NEW] Discovered ESP32 at {sender_ip}")


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("ESP32 Server - Using Pub/Sub API")
    print("ESP32 sends to us, we reply back to sender")
    print("=" * 60)
    print()
    
    # Show local IP for reference
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        print(f"[Info] Your IP: {local_ip}")
        print(f"[Info] Configure ESP32 to send to: {local_ip}:{LISTEN_PORT}")
    except:
        local_ip = "unknown"
    
    print(f"[Info] Listening for ESP32 feedback on port {LISTEN_PORT}")
    print(f"[Info] Will send commands to ESP32s on port {COMMAND_PORT}")
    print()
    
    # Create network server using the pub/sub API
    server = NetworkServer(
        recv_type=SentData,
        send_type=ReceivedData,
        recv_port=LISTEN_PORT,
        send_port=COMMAND_PORT,
        callback=on_feedback,
        timeout_sec=2.0,
    )
    
    print("[Ready] Waiting for ESP32 connections...")
    print("[Ready] Press Ctrl+C to stop")
    print()
    
    # Control state
    target_pos = 0.0
    direction = 1
    
    # Rate limiter
    rate = Rate(CONTROL_RATE)
    
    # Stats
    cmd_count = 0
    fb_count = 0
    
    start_time = time.time()
    last_print = time.time()
    
    try:
        while True:
            now = time.time()
            
            # Process incoming messages
            received = server.spin_once()
            fb_count += received
            
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
            
            # Send to all active ESP32s
            sent = server.send_to_all(cmd)
            cmd_count += sent
            
            # Print status every second
            if now - last_print >= 1.0:
                elapsed = now - start_time
                
                # Get active devices
                active = server.active_devices
                
                status = f"\r[{elapsed:6.1f}s] Cmd: {cmd_count:6d} | Fb: {fb_count:6d} | Target: {target_pos:+.3f}"
                status += f" | Active ESP32s: {len(active)}"
                
                # Show each ESP32's status
                for ip, dev in active.items():
                    if dev.last_message:
                        msg = dev.last_message
                        status += f"\n        {ip}: pos={msg.motor.pos:+.3f} vel={msg.motor.vel:+.3f} (rx:{dev.recv_count} tx:{dev.send_count})"
                
                if not active:
                    status += " | Waiting for ESP32..."
                
                # Clear and print
                print("\033[K" + status, end="", flush=True)
                if active:
                    print(f"\033[{len(active)}A", end="", flush=True)
                
                last_print = now
            
            # Maintain control rate
            rate.sleep()
            
    except KeyboardInterrupt:
        print("\n\n[Stopped]")
    
    finally:
        server.close()
        
        elapsed = time.time() - start_time
        print()
        print("=" * 60)
        print("Summary:")
        print(f"  Runtime: {elapsed:.1f}s")
        print(f"  Commands sent: {cmd_count}")
        print(f"  Feedback received: {fb_count}")
        print(f"  Discovered ESP32s: {list(server.devices.keys())}")
        print("=" * 60)


if __name__ == "__main__":
    main()
