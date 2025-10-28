#!/usr/bin/env python3
# tools/simulate_pi_leader.py
# Simulates a Raspberry Pi Leader with AS5600 encoders and PCA9548A multiplexer
# Provides more realistic simulation of the Pi-based leader system

import socket
import struct
import time
import math
import signal
import sys
import numpy as np
import random
from typing import List, Tuple

# Configuration
HOST = "127.0.0.1"
PORT = 9001
HZ = 200  # 200 Hz is stable on laptops; raise to 1000 if desired
DT = 1.0 / HZ

# AS5600 Simulation Parameters
AS5600_RESOLUTION = 4095  # 12-bit resolution
RAD_PER_COUNT = 2.0 * math.pi / AS5600_RESOLUTION
MAGNETIC_FIELD_MIN = 1000  # Minimum valid magnetic field strength
MAGNETIC_FIELD_MAX = 4000  # Maximum valid magnetic field strength

class SimulatedAS5600:
    """Simulates an AS5600 magnetic encoder"""
    
    def __init__(self, joint_id: int, base_frequency: float, amplitude: float, phase: float = 0.0):
        self.joint_id = joint_id
        self.base_frequency = base_frequency
        self.amplitude = amplitude
        self.phase = phase
        self.last_angle = 0.0
        self.unwrapped_angle = 0.0
        self.magnitude = random.randint(2000, 3500)  # Simulated magnetic field strength
        self.agc = random.randint(50, 200)  # Simulated AGC value
        self.valid = True
        self.read_failures = 0
        self.max_failures = 100  # Simulate occasional read failures
        
    def read_angle(self, t: float) -> Tuple[bool, float]:
        """Simulate reading angle from AS5600"""
        # Simulate occasional read failures
        if random.random() < 0.001:  # 0.1% failure rate
            self.read_failures += 1
            if self.read_failures > self.max_failures:
                self.valid = False
                return False, self.last_angle
        
        # Generate smooth sinusoidal trajectory
        angle = self.amplitude * math.sin(2 * math.pi * self.base_frequency * t + self.phase)
        
        # Add realistic noise (AS5600 has ~0.1° accuracy)
        noise = random.gauss(0, 0.001)  # 0.1° standard deviation
        angle += noise
        
        # Simulate magnetic field variations
        if random.random() < 0.01:  # 1% chance of field variation
            self.magnitude += random.randint(-100, 100)
            self.magnitude = max(MAGNETIC_FIELD_MIN, min(MAGNETIC_FIELD_MAX, self.magnitude))
        
        # Check magnetic field strength
        if self.magnitude < MAGNETIC_FIELD_MIN:
            self.valid = False
            return False, self.last_angle
        
        # Update angle with proper unwrapping
        angle_diff = angle - self.last_angle
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        self.unwrapped_angle += angle_diff
        self.last_angle = angle
        self.valid = True
        
        return True, angle

class SimulatedPCA9548A:
    """Simulates PCA9548A I2C multiplexer"""
    
    def __init__(self, encoders: List[SimulatedAS5600]):
        self.encoders = encoders
        self.current_channel = 0
        self.channel_switch_delay = 0.001  # 1ms settling time
        
    def select_channel(self, channel: int) -> bool:
        """Simulate channel selection with settling time"""
        if channel >= len(self.encoders):
            return False
        self.current_channel = channel
        time.sleep(self.channel_switch_delay)  # Simulate settling time
        return True
    
    def read_encoder(self, channel: int, t: float) -> Tuple[bool, float]:
        """Read from specific encoder channel"""
        if not self.select_channel(channel):
            return False, 0.0
        return self.encoders[channel].read_angle(t)

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

def pack_leader_state(t_us: int, q: List[float], qd: List[float], buttons: int = 0) -> bytes:
    """Pack BILK LeaderState frame"""
    payload = struct.pack("<I", t_us) \
        + struct.pack("<ffff", *q) \
        + struct.pack("<ffff", *qd) \
        + bytes([buttons]) + b"\x00\x00\x00"
    hdr = bytes([1, 0x01]) + struct.pack("<H", len(payload))
    crc = crc16_ccitt_false(hdr + payload)
    return b"BILK" + hdr + payload + struct.pack("<H", crc)

def wrap_pi(angle: float) -> float:
    """Wrap angle to [-π, π]"""
    while angle <= -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle

def lp_alpha(dt: float, tau: float) -> float:
    """Low-pass filter alpha calculation"""
    if tau <= 1e-6:
        return 1.0
    return dt / (tau + dt)

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    print(f"\n[simulate_pi_leader] Received signal {signum}, shutting down gracefully...")
    sys.exit(0)

def print_encoder_diagnostics(encoders: List[SimulatedAS5600], multiplexer: SimulatedPCA9548A):
    """Print encoder diagnostics similar to real Pi leader"""
    print("=== Simulated AS5600 Encoder Diagnostics ===")
    for i, encoder in enumerate(encoders):
        print(f"Encoder {i}: Valid={encoder.valid}, Angle={encoder.last_angle:.3f} rad, "
              f"Mag={encoder.magnitude}, AGC={encoder.agc}, Failures={encoder.read_failures}")
    print(f"Current Mux Channel: {multiplexer.current_channel}")
    print("=============================================")

def main():
    """Main simulation loop"""
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("🏥 BILK Pi Leader Simulation - Starting...")
    print("Simulating Raspberry Pi with AS5600 encoders and PCA9548A multiplexer")
    
    # Create simulated encoders with different trajectories
    encoders = [
        SimulatedAS5600(0, 0.30, 1.0, 0.0),      # J1: Base rotation
        SimulatedAS5600(1, 0.20, 0.6, math.pi/4), # J2: Shoulder
        SimulatedAS5600(2, 0.12, 0.3, math.pi/2), # J3: Elbow
        SimulatedAS5600(3, 0.0, 0.15, 0.0)        # J4: Wrist (constant)
    ]
    
    # Create simulated multiplexer
    multiplexer = SimulatedPCA9548A(encoders)
    
    # Initialize state
    last_angles = [0.0] * 4
    last_velocities = [0.0] * 4
    unwrapped_angles = [0.0] * 4
    lp_tau_ms = 10  # Low-pass filter time constant
    
    # Setup network
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time()
    
    print(f"[simulate_pi_leader] Streaming to {HOST}:{PORT} at {HZ} Hz. Ctrl+C to stop.")
    print("Simulating realistic I2C timing and encoder behavior...")
    
    # Print initial diagnostics
    print_encoder_diagnostics(encoders, multiplexer)
    
    try:
        last_diag_time = time.time()
        while True:
            loop_start = time.time()
            t = time.time() - t0
            
            # Read all encoders through multiplexer (simulating I2C switching)
            for i in range(4):
                success, angle = multiplexer.read_encoder(i, t)
                
                if success:
                    # Calculate velocity with proper unwrapping
                    angle_diff = angle - last_angles[i]
                    if angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    elif angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    # Update unwrapped angle
                    unwrapped_angles[i] += angle_diff
                    
                    # Calculate velocity with low-pass filtering
                    raw_velocity = angle_diff / DT
                    alpha = lp_alpha(DT, lp_tau_ms / 1000.0)
                    last_velocities[i] = alpha * raw_velocity + (1.0 - alpha) * last_velocities[i]
                    
                    # Update angle
                    last_angles[i] = wrap_pi(angle)
                else:
                    # Encoder read failed - maintain last known values
                    if random.random() < 0.1:  # Only print occasionally
                        print(f"Simulated encoder {i} read failed")
            
            # Build and send frame
            timestamp_us = int(t * 1e6)
            frame = pack_leader_state(timestamp_us, last_angles, last_velocities)
            sock.sendto(frame, (HOST, PORT))
            
            # Periodic diagnostics (every 5 seconds)
            if time.time() - last_diag_time > 5.0:
                print_encoder_diagnostics(encoders, multiplexer)
                last_diag_time = time.time()
            
            # Maintain timing
            elapsed = time.time() - loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                if random.random() < 0.01:  # Only warn occasionally
                    print(f"Warning: Loop time exceeded target ({elapsed*1000:.1f}ms)")
                    
    except KeyboardInterrupt:
        print("\n[simulate_pi_leader] Interrupted, shutting down...")
    finally:
        sock.close()
        print("Simulation complete!")

if __name__ == "__main__":
    main()
