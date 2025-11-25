#!/usr/bin/env python3
# follower_pi_bridge.py - Raspberry Pi Follower Bridge
# Receives encoder data from Leader Pi via WiFi UDP
# Applies smoothing and safety checks
# Forwards motor commands to Arduino Mega via USB serial

import serial
import time
import struct
import sys
import socket
import select
import numpy as np
import signal
from collections import deque

# BILK Protocol Constants
PRE = b"BILK"
MSG_LEADER_STATE = 0x01
MSG_SET_MODE = 0x10

# Configuration
UDP_PORT = 9001  # Port to receive UDP from Leader Pi
LATENCY_THRESH_MS = 100  # Watchdog timeout (ms) - send HOLD if exceeded
BAUD_RATE = 2000000  # Serial baud rate to Arduino

def crc16_ccitt_false(data: bytes) -> int:
    """Calculate CRC-16 CCITT-FALSE"""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def pack_frame(msg_type: int, payload: bytes) -> bytes:
    """Pack BILK protocol frame"""
    hdr = bytes([1, msg_type]) + struct.pack("<H", len(payload))
    crc = crc16_ccitt_false(hdr + payload)
    return PRE + hdr + payload + struct.pack("<H", crc)

def parse_frame(buf: bytes):
    """Parse BILK protocol frame"""
    if len(buf) < 10 or buf[0:4] != PRE:
        return None
    hdr = buf[4:8]
    ver, msg, plen = hdr[0], hdr[1], struct.unpack("<H", hdr[2:4])[0]
    if len(buf) != 8 + plen + 2:
        return None
    payload = buf[8:8+plen]
    crc = struct.unpack("<H", buf[8+plen:8+plen+2])[0]
    if crc16_ccitt_false(hdr + payload) != crc:
        return None
    return (msg, payload)

def minjerk_filter(hist: deque, window=0.050):
    """Minimum jerk filter for smooth motion"""
    t = time.time()
    while hist and (t - hist[0][0]) > window:
        hist.popleft()
    if not hist:
        return None
    qs = [x[1] for x in hist]
    return np.mean(np.stack(qs, axis=0), axis=0)

def main(arduino_port="/dev/ttyUSB0", baud=BAUD_RATE):
    """Main bridge function"""
    print("BILK Follower Pi Bridge - Starting...")
    print(f"  UDP Port: {UDP_PORT}")
    print(f"  Arduino Port: {arduino_port}")
    print(f"  Baud Rate: {baud}")
    
    # Setup UDP socket to receive from Leader Pi
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)
    print(f"  Listening for UDP on port {UDP_PORT}...")
    
    # Setup serial connection to Arduino Mega
    try:
        ser_arduino = serial.Serial(arduino_port, baudrate=baud, timeout=0.001)
        print(f"  Connected to Arduino on {arduino_port}")
    except Exception as e:
        print(f"ERROR: Failed to open Arduino serial port {arduino_port}: {e}")
        print("  Make sure Arduino is connected and port is correct")
        sys.exit(1)
    
    # State tracking
    hist = deque()  # History for smoothing filter
    last_forward = time.time()
    frame_count = 0
    
    print("Bridge running... (Ctrl+C to stop)")
    
    try:
        while True:
            got = False
            
            # Check for UDP data from Leader Pi
            ready, _, _ = select.select([sock], [], [], 0.0)
            if ready:
                try:
                    data, addr = sock.recvfrom(512)
                    f = parse_frame(data)
                    if f and f[0] == MSG_LEADER_STATE:
                        pl = f[1]
                        t_us = struct.unpack("<I", pl[0:4])[0]
                        q = np.array(struct.unpack("<ffff", pl[4:20]))
                        qd = struct.unpack("<ffff", pl[20:36])
                        buttons = pl[36]
                        
                        # Apply smoothing filter
                        hist.append((time.time(), q))
                        q_s = minjerk_filter(hist) or q
                        
                        # Rebuild payload with smoothed positions
                        pl2 = struct.pack("<I", t_us) + \
                              struct.pack("<ffff", *q_s) + \
                              struct.pack("<ffff", *qd) + \
                              bytes([buttons]) + b"\x00\x00\x00"
                        
                        # Forward to Arduino
                        ser_arduino.write(pack_frame(MSG_LEADER_STATE, pl2))
                        last_forward = time.time()
                        got = True
                        frame_count += 1
                        
                        if frame_count % 1000 == 0:
                            print(f"  Processed {frame_count} frames")
                            
                except BlockingIOError:
                    pass
                except Exception as e:
                    print(f"Error processing UDP frame: {e}")
            
            # Watchdog: if no data received for too long, send HOLD command
            if (time.time() - last_forward) * 1000 > LATENCY_THRESH_MS:
                ser_arduino.write(pack_frame(MSG_SET_MODE, bytes([2, 0, 0, 0])))  # HOLD mode
                last_forward = time.time()
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        sock.close()
        ser_arduino.close()
        print("Bridge stopped")

if __name__ == "__main__":
    # Handle command line arguments
    arduino_port = "/dev/ttyUSB0"  # Default Arduino port
    if len(sys.argv) >= 2:
        arduino_port = sys.argv[1]
    
    # Setup signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda s, f: sys.exit(0))
    
    main(arduino_port=arduino_port)

