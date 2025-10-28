#!/usr/bin/env python3
# leader_pi_as5600.py - Raspberry Pi Leader with AS5600 Encoders
# Reads AS5600 encoders via PCA9548A multiplexer and streams BILK protocol

import smbus
import time
import socket
import struct
import signal
import sys
import threading
from typing import List, Tuple, Optional

# Hardware Configuration
I2C_BUS = 1  # Raspberry Pi I2C bus (usually 1)
PCA9548A_ADDR = 0x70  # PCA9548A I2C multiplexer address
AS5600_ADDR = 0x36    # AS5600 I2C address
SAMPLE_RATE_HZ = 1000  # 1kHz sampling rate
SAMPLE_US = 1000       # 1ms in microseconds

# AS5600 Register Definitions
AS5600_REG_RAW_ANGLE_HI = 0x0C  # Raw angle high byte
AS5600_REG_RAW_ANGLE_LO = 0x0D  # Raw angle low byte
AS5600_REG_STATUS = 0x0B        # Status register
AS5600_REG_AGC = 0x1A          # Automatic Gain Control
AS5600_REG_MAGNITUDE_HI = 0x1B  # Magnitude high byte
AS5600_REG_MAGNITUDE_LO = 0x1C  # Magnitude low byte

# BILK Protocol Constants
BILK_PRE = b'BILK'
BILK_VERSION = 0x01
BILK_MSG_LEADER_STATE = 0x01
AS5600_RESOLUTION = 4095  # 12-bit resolution (0-4095)
RAD_PER_COUNT = 2.0 * 3.14159265359 / AS5600_RESOLUTION

# Network Configuration
HOST_IP = "192.168.1.100"  # Host bridge IP
HOST_PORT = 9001
USB_SERIAL_PORT = "/dev/ttyUSB0"  # USB serial fallback

class AS5600Encoder:
    """AS5600 magnetic encoder interface"""
    
    def __init__(self, bus: smbus.SMBus, mux_channel: int):
        self.bus = bus
        self.mux_channel = mux_channel
        self.last_angle = 0.0
        self.unwrapped_angle = 0.0
        self.valid = False
        self.magnitude = 0
        self.agc = 0
        
    def select_channel(self) -> bool:
        """Select multiplexer channel"""
        try:
            self.bus.write_byte(PCA9548A_ADDR, 1 << self.mux_channel)
            time.sleep(0.001)  # 1ms settling time
            return True
        except Exception as e:
            print(f"Error selecting channel {self.mux_channel}: {e}")
            return False
    
    def read_register(self, reg: int, length: int = 1) -> Optional[bytes]:
        """Read register from AS5600"""
        try:
            return self.bus.read_i2c_block_data(AS5600_ADDR, reg, length)
        except Exception as e:
            print(f"Error reading register 0x{reg:02X}: {e}")
            return None
    
    def read_raw_angle(self) -> Optional[int]:
        """Read raw angle from AS5600"""
        data = self.read_register(AS5600_REG_RAW_ANGLE_HI, 2)
        if data is None or len(data) != 2:
            return None
        return (data[0] << 8) | data[1]
    
    def read_status(self) -> Optional[int]:
        """Read status register"""
        data = self.read_register(AS5600_REG_STATUS)
        return data[0] if data else None
    
    def read_magnitude(self) -> Optional[int]:
        """Read magnetic field magnitude"""
        data = self.read_register(AS5600_REG_MAGNITUDE_HI, 2)
        if data is None or len(data) != 2:
            return None
        return (data[0] << 8) | data[1]
    
    def read_agc(self) -> Optional[int]:
        """Read AGC value"""
        data = self.read_register(AS5600_REG_AGC)
        return data[0] if data else None
    
    def read_angle(self) -> Tuple[bool, float]:
        """Read angle with error checking"""
        if not self.select_channel():
            self.valid = False
            return False, self.last_angle
        
        # Read raw angle
        raw_angle = self.read_raw_angle()
        if raw_angle is None:
            self.valid = False
            return False, self.last_angle
        
        # Read status for error checking
        status = self.read_status()
        if status is None:
            self.valid = False
            return False, self.last_angle
        
        # Read magnitude for field strength check
        magnitude = self.read_magnitude()
        if magnitude is None:
            self.valid = False
            return False, self.last_angle
        
        # Read AGC for diagnostics
        agc = self.read_agc()
        if agc is not None:
            self.agc = agc
        
        # Store diagnostic info
        self.magnitude = magnitude
        
        # Check for magnetic field issues
        if magnitude < 1000:  # Too weak
            self.valid = False
            return False, self.last_angle
        
        # Check status register for errors (bits 3 and 4)
        if status & 0x18:  # MH (too strong) or ML (too weak)
            self.valid = False
            return False, self.last_angle
        
        # Convert to radians
        angle = raw_angle * RAD_PER_COUNT
        
        # Handle angle wrapping
        angle_diff = angle - self.last_angle
        if angle_diff > 3.14159:
            angle_diff -= 2 * 3.14159
        elif angle_diff < -3.14159:
            angle_diff += 2 * 3.14159
        
        self.unwrapped_angle += angle_diff
        self.last_angle = angle
        self.valid = True
        
        return True, angle

class BILKLeader:
    """BILK Leader implementation for Raspberry Pi"""
    
    def __init__(self):
        self.bus = smbus.SMBus(I2C_BUS)
        self.encoders = [AS5600Encoder(self.bus, i) for i in range(4)]
        self.socket = None
        self.running = False
        self.last_angles = [0.0] * 4
        self.last_velocities = [0.0] * 4
        self.unwrapped_angles = [0.0] * 4
        
        # Low-pass filter parameters
        self.lp_tau_ms = 10
        self.last_time = time.time()
        
    def crc16_ccitt_false(self, data: bytes) -> int:
        """Calculate CRC-16 CCITT-FALSE"""
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) & 0xFFFF) ^ 0x1021
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc
    
    def wrap_pi(self, angle: float) -> float:
        """Wrap angle to [-π, π]"""
        while angle <= -3.14159:
            angle += 2 * 3.14159
        while angle > 3.14159:
            angle -= 2 * 3.14159
        return angle
    
    def lp_alpha(self, dt: float, tau: float) -> float:
        """Low-pass filter alpha calculation"""
        if tau <= 1e-6:
            return 1.0
        return dt / (tau + dt)
    
    def build_leader_state_frame(self, timestamp_us: int, buttons: int = 0) -> bytes:
        """Build BILK LeaderState frame"""
        # Pack leader state data
        leader_state = struct.pack('<I', timestamp_us)  # timestamp
        leader_state += struct.pack('<ffff', *self.last_angles)  # q[4]
        leader_state += struct.pack('<ffff', *self.last_velocities)  # qd[4]
        leader_state += struct.pack('B', buttons)  # buttons
        leader_state += b'\x00\x00\x00'  # reserved[3]
        
        # Build frame header
        header = struct.pack('BBH', BILK_VERSION, BILK_MSG_LEADER_STATE, len(leader_state))
        
        # Calculate CRC
        crc_data = header + leader_state
        crc = self.crc16_ccitt_false(crc_data)
        
        # Build complete frame
        frame = BILK_PRE + header + leader_state + struct.pack('<H', crc)
        return frame
    
    def read_encoders(self) -> None:
        """Read all encoders and update state"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        for i, encoder in enumerate(self.encoders):
            success, angle = encoder.read_angle()
            
            if success:
                # Calculate velocity
                angle_diff = angle - self.last_angles[i]
                if angle_diff > 3.14159:
                    angle_diff -= 2 * 3.14159
                elif angle_diff < -3.14159:
                    angle_diff += 2 * 3.14159
                
                # Update unwrapped angle
                self.unwrapped_angles[i] += angle_diff
                
                # Calculate velocity with low-pass filtering
                raw_velocity = angle_diff / dt if dt > 0 else 0
                alpha = self.lp_alpha(dt, self.lp_tau_ms / 1000.0)
                self.last_velocities[i] = alpha * raw_velocity + (1.0 - alpha) * self.last_velocities[i]
                
                # Update angle
                self.last_angles[i] = self.wrap_pi(angle)
            else:
                # Encoder read failed - maintain last known values
                print(f"Encoder {i} read failed")
    
    def setup_network(self) -> bool:
        """Setup network connection"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"Network setup complete, target: {HOST_IP}:{HOST_PORT}")
            return True
        except Exception as e:
            print(f"Network setup failed: {e}")
            return False
    
    def send_frame(self, frame: bytes) -> None:
        """Send frame via UDP and USB serial"""
        # Send via UDP
        if self.socket:
            try:
                self.socket.sendto(frame, (HOST_IP, HOST_PORT))
            except Exception as e:
                print(f"UDP send failed: {e}")
        
        # Send via USB serial (fallback)
        try:
            with open(USB_SERIAL_PORT, 'wb') as f:
                f.write(frame)
        except Exception as e:
            print(f"USB serial send failed: {e}")
    
    def print_diagnostics(self) -> None:
        """Print encoder diagnostics"""
        print("=== AS5600 Encoder Diagnostics ===")
        for i, encoder in enumerate(self.encoders):
            print(f"Encoder {i}: Valid={encoder.valid}, Angle={self.last_angles[i]:.3f} rad, "
                  f"Mag={encoder.magnitude}, AGC={encoder.agc}")
        print("==================================")
    
    def run(self) -> None:
        """Main execution loop"""
        print("BILK Leader Pi - Starting...")
        
        # Initialize encoders
        print("Initializing AS5600 encoders...")
        for i, encoder in enumerate(self.encoders):
            success, angle = encoder.read_angle()
            if success:
                self.last_angles[i] = angle
                self.unwrapped_angles[i] = angle
                print(f"Encoder {i} initialized: {angle:.3f} rad")
            else:
                print(f"Encoder {i} initialization failed!")
        
        # Setup network
        if not self.setup_network():
            print("Warning: Network setup failed, continuing with USB only")
        
        # Print initial diagnostics
        self.print_diagnostics()
        
        # Main loop
        self.running = True
        last_diag_time = time.time()
        
        print(f"Starting main loop at {SAMPLE_RATE_HZ}Hz...")
        
        try:
            while self.running:
                loop_start = time.time()
                
                # Read encoders
                self.read_encoders()
                
                # Build and send frame
                timestamp_us = int(time.time() * 1e6)
                frame = self.build_leader_state_frame(timestamp_us)
                self.send_frame(frame)
                
                # Periodic diagnostics
                if time.time() - last_diag_time > 5.0:
                    self.print_diagnostics()
                    last_diag_time = time.time()
                
                # Maintain timing
                elapsed = time.time() - loop_start
                sleep_time = (1.0 / SAMPLE_RATE_HZ) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(f"Warning: Loop time exceeded target ({elapsed*1000:.1f}ms)")
                    
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.running = False
            if self.socket:
                self.socket.close()

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    print(f"\nReceived signal {signum}, shutting down...")
    sys.exit(0)

def main():
    """Main entry point"""
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run leader
    leader = BILKLeader()
    leader.run()

if __name__ == "__main__":
    main()
