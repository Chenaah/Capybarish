#!/usr/bin/env python3
"""
ESP32 Server - Listen for any ESP32, reply to sender

This is the most practical pattern:
- ESP32 knows the PC's IP (hardcoded)
- PC doesn't need to know ESP32's IP
- PC listens on a port and replies to whoever sent

This allows multiple ESP32s to connect without configuration!

Usage:
    python examples/esp32_server.py

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import time
import sys
import os
import socket

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from capybarish.generated import ReceivedData, SentData


# =============================================================================
# Configuration
# =============================================================================

LISTEN_PORT = 6666    # Port to receive feedback from ESP32s
COMMAND_PORT = 6667   # Port ESP32s listen on for commands

CONTROL_RATE = 100.0  # Hz


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("ESP32 Server - Auto-discover ESP32s")
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
    print(f"[Info] Will send commands back to ESP32 on port {COMMAND_PORT}")
    print()
    
    # Create socket for receiving feedback (and sending commands)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", LISTEN_PORT))
    sock.setblocking(False)
    
    print("[Ready] Waiting for ESP32 connections...")
    print("[Ready] Press Ctrl+C to stop")
    print()
    
    # Control state
    target_pos = 0.0
    direction = 1
    
    # Stats
    cmd_count = 0
    fb_count = 0
    discovered_esp32s = {}  # IP -> last feedback time and data
    
    start_time = time.time()
    last_print = time.time()
    period = 1.0 / CONTROL_RATE
    last_send = time.time()
    
    try:
        while True:
            now = time.time()
            
            # Receive feedback from any ESP32 (non-blocking)
            try:
                data, addr = sock.recvfrom(4096)
                esp32_ip = addr[0]
                
                if len(data) >= SentData._SIZE:
                    fb = SentData.deserialize(data)
                    fb_count += 1
                    
                    # Track this ESP32
                    discovered_esp32s[esp32_ip] = {
                        'time': now,
                        'pos': fb.motor.pos,
                        'vel': fb.motor.vel,
                        'port': addr[1],  # Remember source port (though we'll send to COMMAND_PORT)
                    }
                    
                    # First time seeing this ESP32?
                    if esp32_ip not in discovered_esp32s or now - discovered_esp32s[esp32_ip].get('first_seen', now) < 0.1:
                        discovered_esp32s[esp32_ip]['first_seen'] = now
                        print(f"\n[NEW] Discovered ESP32 at {esp32_ip}")
                    
            except BlockingIOError:
                pass  # No data available
            
            # Send commands to ALL discovered ESP32s at control rate
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
                
                # Send to each discovered ESP32 (reply to sender!)
                for esp32_ip, info in discovered_esp32s.items():
                    # Only send to ESP32s we've heard from recently (last 2 seconds)
                    if now - info['time'] < 2.0:
                        sock.sendto(cmd.serialize(), (esp32_ip, COMMAND_PORT))
                        cmd_count += 1
                
                last_send = now
            
            # Print status every second
            if now - last_print >= 1.0:
                elapsed = now - start_time
                
                # Count active ESP32s
                active_esp32s = [ip for ip, info in discovered_esp32s.items() 
                                 if now - info['time'] < 2.0]
                
                status = f"\r[{elapsed:6.1f}s] Cmd: {cmd_count:6d} | Fb: {fb_count:6d} | Target: {target_pos:+.3f}"
                status += f" | Active ESP32s: {len(active_esp32s)}"
                
                # Show each ESP32's status
                for ip in active_esp32s:
                    info = discovered_esp32s[ip]
                    status += f"\n        {ip}: pos={info['pos']:+.3f} vel={info['vel']:+.3f}"
                
                if not active_esp32s:
                    status += " | Waiting for ESP32..."
                
                # Clear previous lines and print
                print("\033[K" + status, end="", flush=True)
                if active_esp32s:
                    # Move cursor up for next update
                    print(f"\033[{len(active_esp32s)}A", end="", flush=True)
                
                last_print = now
            
            time.sleep(0.0001)
            
    except KeyboardInterrupt:
        print("\n\n[Stopped]")
    
    finally:
        sock.close()
        
        elapsed = time.time() - start_time
        print()
        print("=" * 60)
        print("Summary:")
        print(f"  Runtime: {elapsed:.1f}s")
        print(f"  Commands sent: {cmd_count}")
        print(f"  Feedback received: {fb_count}")
        print(f"  Discovered ESP32s: {list(discovered_esp32s.keys())}")
        print("=" * 60)


if __name__ == "__main__":
    main()
