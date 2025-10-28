# BILK Firmware - Finalized Files

This directory contains the production-ready firmware files for the BILK medical robotics system.

## Directory Structure

```
firmware/
├── leader_esp32/
│   └── Leader_ESP32_AS5600_UDP.ino    # ESP32 leader with AS5600 encoders
├── follower_arduino/
│   └── Follower_Arduino.ino           # Arduino Mega follower with PID control
└── README.md                          # This file
```

## Firmware Files

### Leader ESP32 (`leader_esp32/Leader_ESP32_AS5600_UDP.ino`)

**Purpose**: Main leader device that reads AS5600 magnetic encoders and streams data

**Features**:
- ✅ **Correct AS5600 Implementation**: Proper register usage and 12-bit resolution
- ✅ **TCA9548A Multiplexer Support**: 4-channel I2C multiplexer for multiple encoders
- ✅ **WiFi UDP Streaming**: Real-time data transmission at 1kHz
- ✅ **USB Fallback**: Serial output for debugging and backup communication
- ✅ **Comprehensive Error Handling**: Magnetic field validation and status checking
- ✅ **Diagnostic Capabilities**: Real-time encoder health monitoring
- ✅ **Robust Timing**: Proper multiplexer settling and I2C timing

**Hardware Requirements**:
- ESP32 development board
- 4x AS5600 magnetic encoders
- TCA9548A I2C multiplexer
- Magnets for each encoder
- WiFi connectivity

**Configuration**:
- Update WiFi credentials in the code
- Set correct host IP address
- Verify I2C pin assignments (SDA=21, SCL=22)

### Follower Arduino (`follower_arduino/Follower_Arduino.ino`)

**Purpose**: Follower device that receives commands and controls motors via PID

**Features**:
- ✅ **BILK Protocol Support**: Complete frame parsing with CRC validation
- ✅ **PID Control**: Configurable PID parameters for each joint
- ✅ **Watchdog Safety**: Automatic HOLD mode on communication loss
- ✅ **Mode Management**: IDLE, FOLLOW, HOLD, SHUTDOWN modes
- ✅ **Motor Control Interface**: Ready for PWM/DIR motor driver integration

**Hardware Requirements**:
- Arduino Mega 2560
- Motor drivers (PWM/DIR interface)
- Motors for each joint
- Serial communication to host

**Configuration**:
- Adjust PID parameters (Kp, Ki, Kd) for each joint
- Map PWM/DIR pins to your motor drivers
- Set appropriate watchdog timeout

## Dependencies

### Required Libraries
- **ESP32**: Built-in libraries (WiFi, Wire, Arduino)
- **Arduino**: Built-in libraries (Arduino, Wire)

### External Dependencies
- **Protocol Header**: `shared/protocol.h` (shared between leader and follower)

## Compilation and Upload

### Leader ESP32
1. Open `Leader_ESP32_AS5600_UDP.ino` in Arduino IDE
2. Select ESP32 board and correct port
3. Install ESP32 board package if needed
4. Compile and upload

### Follower Arduino
1. Open `Follower_Arduino.ino` in Arduino IDE
2. Select Arduino Mega 2560 board
3. Compile and upload

## System Integration

### Communication Flow
```
AS5600 Encoders → TCA9548A Mux → ESP32 Leader → WiFi UDP → Host Bridge → Serial → Arduino Follower → Motors
```

### Protocol
- **Format**: BILK protocol with CRC-16 validation
- **Frequency**: 1kHz (1ms intervals)
- **Data**: Joint positions (q) and velocities (qd)
- **Safety**: Watchdog timeout triggers HOLD mode

## Testing and Validation

### Leader Testing
- Verify encoder readings with diagnostic output
- Check WiFi connectivity and UDP transmission
- Monitor magnetic field strength and encoder health

### Follower Testing
- Test BILK protocol parsing with simulation
- Verify PID control response
- Test watchdog functionality

### System Testing
- Use simulation tools for end-to-end testing
- Validate timing and data integrity
- Test safety features and error handling

## Safety Considerations

### Hardware Safety
- Ensure proper magnet positioning for AS5600 encoders
- Verify magnetic field strength (1000+ counts)
- Implement hardware emergency stop

### Software Safety
- Watchdog timeout protection
- Encoder failure detection
- Communication loss handling
- Mode transition validation

## Troubleshooting

### Common Issues
1. **Encoder Read Failures**: Check magnetic field strength and positioning
2. **WiFi Connection**: Verify credentials and network availability
3. **I2C Errors**: Check wiring and multiplexer addressing
4. **Communication Loss**: Verify host bridge and serial connections

### Diagnostic Tools
- Serial monitor for real-time diagnostics
- Simulation tools for testing without hardware
- Diagnostic plots for performance analysis

## Version History

- **v1.0**: Initial implementation with AS5600 bugs
- **v2.0**: Fixed AS5600 implementation with proper error handling
- **v2.1**: Added comprehensive diagnostics and robust error recovery

## Support

For technical support or questions about the firmware implementation, refer to:
- `AS5600_ANALYSIS.md` - Detailed AS5600 implementation analysis
- `SIMULATION_DIAGNOSTICS.md` - Simulation and testing procedures
- `docs/` directory - Additional documentation
