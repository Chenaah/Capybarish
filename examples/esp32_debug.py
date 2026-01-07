#!/usr/bin/env python3
"""
ESP32 Communication Debugger

Simple script to test UDP communication with ESP32.
Run this to see what's happening on the network.

Usage:
    python examples/esp32_debug.py

Author: Chen Yu <chenyu@u.northwestern.edu>
"""

import socket
import struct
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from capybarish.generated import MotorCommand, SensorData

# =============================================================================
# Configuration - UPDATE THESE!
# =============================================================================

# Your computer's IP (run `ip addr` or `ifconfig` to find it)
# ESP32 will send feedback to this IP
SERVER_IP = "0.0.0.0"  # Listen on all interfaces

# Port to listen for ESP32 feedback
LISTEN_PORT = 6666

# ESP32's IP address (check Serial monitor on ESP32 after WiFi connects)
ESP32_IP = "192.168.1.101"  # <-- UPDATE THIS!

# Port ESP32 is listening on
ESP32_PORT = 6666

# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("ESP32 Communication Debugger")
    print("=" * 60)
    print()
    
    # Get local IP addresses
    print("[1] Your computer's IP addresses:")
    try:
        hostname = socket.gethostname()
        # Get all IPs
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            print(f"    {info[4][0]}")
    except:
        pass
    
    # Also try to get the IP used for external connections
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        print(f"    {local_ip} (likely the one ESP32 should use)")
    except:
        local_ip = "unknown"
    
    print()
    print(f"[2] Configuration:")
    print(f"    Listening on port: {LISTEN_PORT}")
    print(f"    ESP32 IP: {ESP32_IP}")
    print(f"    ESP32 Port: {ESP32_PORT}")
    print()
    
    # Create UDP socket for receiving
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        recv_sock.bind((SERVER_IP, LISTEN_PORT))
        print(f"[3] ✓ Successfully bound to port {LISTEN_PORT}")
    except OSError as e:
        print(f"[3] ✗ Failed to bind to port {LISTEN_PORT}: {e}")
        print("    Is another program using this port?")
        return
    
    recv_sock.settimeout(1.0)  # 1 second timeout
    
    # Create UDP socket for sending
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print()
    print("[4] Sending test command to ESP32...")
    
    # Create a test command
    cmd = MotorCommand(
        target=1.0,
        target_vel=0.0,
        kp=10.0,
        kd=0.5,
        enable_filter=1,
        switch_=1,
        calibrate=0,
        restart=0,
        timestamp=0.0,
    )
    
    try:
        data = cmd.serialize()
        send_sock.sendto(data, (ESP32_IP, ESP32_PORT))
        print(f"    ✓ Sent {len(data)} bytes to {ESP32_IP}:{ESP32_PORT}")
    except Exception as e:
        print(f"    ✗ Failed to send: {e}")
    
    print()
    print("[5] Waiting for ESP32 feedback (10 seconds)...")
    print("    (Make sure ESP32 is running and connected to same network)")
    print()
    
    received_count = 0
    for i in range(10):
        try:
            data, addr = recv_sock.recvfrom(4096)
            received_count += 1
            
            print(f"    ✓ Received {len(data)} bytes from {addr}")
            
            # Try to parse as SensorData
            if len(data) >= SensorData._SIZE:
                fb = SensorData.deserialize(data)
                print(f"      Motor pos: {fb.motor.pos:.3f}")
                print(f"      Motor vel: {fb.motor.vel:.3f}")
                print(f"      Timestamp: {fb.timestamp}")
            else:
                print(f"      (Raw data, wrong size: expected {SensorData._SIZE}, got {len(data)})")
                
        except socket.timeout:
            print(f"    ... waiting ({i+1}/10)")
            
            # Send another command
            cmd.timestamp = float(i)
            send_sock.sendto(cmd.serialize(), (ESP32_IP, ESP32_PORT))
    
    print()
    if received_count > 0:
        print(f"[Result] ✓ Communication working! Received {received_count} messages.")
    else:
        print("[Result] ✗ No messages received from ESP32.")
        print()
        print("Troubleshooting:")
        print("  1. Check ESP32 Serial monitor - is it connected to WiFi?")
        print("  2. What IP did ESP32 get? Update ESP32_IP in this script.")
        print("  3. Are ESP32 and computer on the same network/subnet?")
        print("  4. Check firewall - is UDP port 6666 allowed?")
        print("  5. On ESP32, is SERVER_IP set to your computer's IP?")
        print(f"     (Your IP is likely: {local_ip})")
    
    recv_sock.close()
    send_sock.close()


if __name__ == "__main__":
    main()
