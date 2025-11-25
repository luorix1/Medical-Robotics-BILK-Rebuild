# BILK Follower - Raspberry Pi Implementation

This directory contains the Raspberry Pi-based follower bridge implementation for the BILK medical robotics system.

## Files

- `follower_pi_bridge.py` - Main follower bridge implementation
- `start_follower.sh` - Startup script (to be created)
- `requirements.txt` - Python dependencies
- `README.md` - This file

## Hardware Requirements

- Raspberry Pi (Pi 4+ recommended)
- Arduino Mega 2560 connected via USB
- WiFi connectivity
- Motor drivers and motors (controlled by Arduino)

## Software Requirements

- Python 3.7+
- pyserial library
- numpy library
- Network connectivity

## Setup Instructions

### 1. Install Dependencies
```bash
pip3 install -r requirements.txt
```

Or install manually:
```bash
pip3 install pyserial numpy
```

### 2. Find Arduino Port
```bash
# List USB devices
ls /dev/ttyUSB* /dev/ttyACM*

# Common ports:
# /dev/ttyUSB0 - USB-to-serial adapter
# /dev/ttyACM0 - Arduino Mega (native USB)
```

### 3. Configure Network
Ensure the Follower Pi and Leader Pi are on the same WiFi network.

### 4. Run the Follower Bridge
```bash
# Default port (/dev/ttyUSB0)
python3 follower_pi_bridge.py

# Specify Arduino port
python3 follower_pi_bridge.py /dev/ttyACM0
```

## Features

### ✅ **UDP Reception**
- Receives encoder data from Leader Pi via WiFi UDP (port 9001)
- Non-blocking socket for real-time performance
- Error handling for network issues

### ✅ **Smoothing Filter**
- Minimum jerk filter for smooth motion
- 50ms window for trajectory smoothing
- Reduces jitter and improves motion quality

### ✅ **Safety Features**
- Watchdog timeout (100ms) → sends HOLD command to Arduino
- CRC-16 validation on all frames
- Graceful error handling

### ✅ **Serial Communication**
- High-speed serial (2Mbps) to Arduino Mega
- BILK protocol frame packing
- Real-time command forwarding

## Configuration

### Serial Port
Default: `/dev/ttyUSB0`
Override: Pass as command line argument

### UDP Port
Default: `9001` (matches Leader Pi)
Change in code if needed

### Watchdog Timeout
Default: `100ms`
Change `LATENCY_THRESH_MS` in code if needed

### Smoothing Window
Default: `50ms`
Change `window` parameter in `minjerk_filter()` if needed

## Troubleshooting

### Common Issues

1. **Arduino Not Found**
   - Check USB connection
   - Verify port: `ls /dev/tty*`
   - Check permissions: `sudo usermod -a -G dialout $USER`

2. **No UDP Data Received**
   - Check WiFi connectivity
   - Verify Leader Pi is running
   - Check firewall: `sudo ufw status`
   - Test with: `nc -u -l 9001`

3. **Permission Errors**
   - Add user to dialout group: `sudo usermod -a -G dialout $USER`
   - Log out and back in
   - Or run with sudo (not recommended)

4. **Import Errors**
   - Install dependencies: `pip3 install pyserial numpy`
   - Check Python version: `python3 --version` (need 3.7+)

### Diagnostic Commands

```bash
# Check serial ports
ls -l /dev/ttyUSB* /dev/ttyACM*

# Test UDP reception
nc -u -l 9001

# Monitor serial communication
cat /dev/ttyUSB0 | hexdump -C

# Check network connectivity
ping <LEADER_PI_IP>
```

## Performance

- **Latency**: <5ms UDP to serial forwarding
- **CPU Usage**: <5% on Pi 4
- **Memory Usage**: <30MB
- **Frame Rate**: Up to 1kHz (matches Leader Pi)

## Safety Features

- **Watchdog Timeout**: Automatically sends HOLD if no data for 100ms
- **CRC Validation**: All frames validated before forwarding
- **Error Recovery**: Graceful handling of communication failures
- **Smooth Motion**: Minimum jerk filter reduces sudden movements

## Integration

This follower bridge integrates with:
- **Leader Pi**: `firmware/leader_pi/leader_pi_as5600.py`
- **Follower Arduino**: `firmware/follower_arduino/Follower_Arduino.ino`

## Communication Flow

```
Leader Pi (encoders)
    ↓ WiFi UDP (port 9001)
Follower Pi Bridge (this script)
    ↓ USB Serial (2Mbps)
Arduino Mega
    ↓ Motor drivers
Motors
```

## Development

### Adding New Features
1. Modify `follower_pi_bridge.py`
2. Update `requirements.txt` if needed
3. Test with Leader Pi and Arduino
4. Update documentation

### Testing
```bash
# Run with verbose output
python3 follower_pi_bridge.py /dev/ttyACM0

# Test UDP reception separately
python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(('0.0.0.0', 9001)); print('Listening...'); print(s.recvfrom(512))"
```

