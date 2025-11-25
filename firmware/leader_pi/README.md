# BILK Leader - Raspberry Pi Implementation

This directory contains the Raspberry Pi-based leader implementation for the BILK medical robotics system.

## Files

- `leader_pi_as5600.py` - Main leader implementation
- `start_leader.sh` - Startup script with dependency checking
- `requirements.txt` - Python dependencies
- `README.md` - This file

## Hardware Requirements

- Raspberry Pi (Pi 4+ recommended)
- 4x AS5600 magnetic encoders
- 1x PCA9548A I2C multiplexer
- I2C breakout boards
- Magnets for each encoder
- WiFi connectivity

## Software Requirements

- Python 3.7+
- I2C enabled on Raspberry Pi
- smbus library for I2C communication
- Network connectivity

## Setup Instructions

### 1. Enable I2C on Raspberry Pi
```bash
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable
sudo reboot
```

### 2. Install Dependencies
```bash
pip3 install -r requirements.txt
```

### 3. Verify I2C Devices
```bash
# Scan for I2C devices
i2cdetect -y 1

# Should show:
# 70 - PCA9548A multiplexer
# 36 - AS5600 encoders (when channel selected)
```

### 4. Configure Network
Edit `leader_pi_as5600.py` and update:
```python
FOLLOWER_PI_IP = "192.168.1.100"  # Follower Raspberry Pi IP address
UDP_PORT = 9001  # UDP port for communication
USB_SERIAL_PORT = "/dev/ttyUSB0"  # USB serial fallback (optional)
```

**Note:** Set `FOLLOWER_PI_IP` to the IP address of your Follower Raspberry Pi. Both Pi boards must be on the same WiFi network.

### 5. Run the Leader
```bash
bash start_leader.sh
```

## Features

### ✅ **AS5600 Integration**
- Proper 12-bit resolution (0-4095)
- Correct register usage (0x0C/0x0D)
- Magnetic field strength validation
- Status register error checking
- AGC monitoring for diagnostics

### ✅ **PCA9548A Multiplexer Support**
- 8-channel I2C multiplexing
- Proper channel selection timing
- Error handling for communication failures

### ✅ **BILK Protocol**
- Complete BILK frame construction
- CRC-16 validation
- Real-time data streaming at 1kHz

### ✅ **WiFi Communication**
- WiFi UDP to Follower Raspberry Pi
- USB serial fallback (optional)
- Automatic error handling

### ✅ **Diagnostic Capabilities**
- Real-time encoder health monitoring
- Magnetic field strength reporting
- Performance timing analysis
- Periodic status reporting

## Configuration

### I2C Settings
```python
I2C_BUS = 1  # Raspberry Pi I2C bus
PCA9548A_ADDR = 0x70  # Multiplexer address
AS5600_ADDR = 0x36    # Encoder address
```

### Timing Settings
```python
SAMPLE_RATE_HZ = 1000  # 1kHz sampling
SAMPLE_US = 1000       # 1ms intervals
```

### Filter Settings
```python
lp_tau_ms = 10  # Low-pass filter time constant
```

## Troubleshooting

### Common Issues

1. **I2C Not Detected**
   - Check if I2C is enabled: `ls /dev/i2c-*`
   - Verify wiring connections
   - Check power supply (3.3V)

2. **Encoder Read Failures**
   - Verify magnetic field strength (>1000 counts)
   - Check magnet positioning (1-2mm gap)
   - Ensure proper I2C addressing

3. **Network Issues**
   - Check WiFi connectivity
   - Verify FOLLOWER_PI_IP configuration (must match Follower Pi's IP)
   - Ensure both Pi boards are on the same WiFi network
   - Test with: `ping <FOLLOWER_PI_IP>`

4. **Permission Errors**
   - Add user to i2c group: `sudo usermod -a -G i2c $USER`
   - Check USB serial permissions

### Diagnostic Commands

```bash
# Check I2C devices
i2cdetect -y 1

# Monitor I2C traffic
sudo i2cdump -y 1 0x70

# Check system resources
htop
```

## Performance

- **Sampling Rate**: 1kHz (1ms intervals)
- **Latency**: <2ms per encoder read
- **CPU Usage**: <10% on Pi 4
- **Memory Usage**: <50MB

## Safety Features

- **Magnetic Field Validation**: Detects weak/strong fields
- **Status Register Checking**: Validates encoder health
- **Communication Redundancy**: WiFi + USB serial
- **Error Recovery**: Graceful handling of failures
- **Diagnostic Reporting**: Real-time health monitoring

## Integration

This leader implementation integrates with:
- **Follower Pi Bridge**: `firmware/follower_pi/follower_pi_bridge.py`
- **Follower Arduino**: `firmware/follower_arduino/Follower_Arduino.ino`
- **Simulation**: `tools/simulate_with_diagnostics.py`

**Communication Flow:**
```
Leader Pi (this code) → WiFi UDP → Follower Pi → USB Serial → Arduino Mega → Motors
```

## Development

### Adding New Features
1. Modify `leader_pi_as5600.py`
2. Update `requirements.txt` if needed
3. Test with `start_leader.sh`
4. Update documentation

### Testing
```bash
# Run with verbose output
python3 leader_pi_as5600.py

# Test I2C communication
python3 -c "import smbus; bus = smbus.SMBus(1); print(bus.read_byte(0x70))"
```
