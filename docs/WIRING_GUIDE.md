# BILK Hardware Wiring Guide

This guide covers the wiring for the BILK medical robotics system with the available hardware.

## Available Hardware
- 2x Raspberry Pi boards
- 2x Arduino Mega 2560
- 4x AS5600 magnetic encoders
- 1x PCA9548A I2C multiplexer
- 1x Pololu dual VNH5019 motor driver shield
- 1x L298N motor driver
- 3x Pololu motors
- 1x Servo motor

## System Architecture

### Leader System (Raspberry Pi #1)
- Reads AS5600 encoders via PCA9548A multiplexer
- Streams encoder data via WiFi UDP to Follower Raspberry Pi
- Connected to I2C bus for encoder communication

### Follower System (Raspberry Pi #2)
- Receives encoder data from Leader via WiFi UDP
- Applies smoothing and safety checks
- Forwards motor commands to Arduino Mega via USB serial

### Follower Control System (Arduino Mega #1)
- Receives commands from Follower Raspberry Pi via USB serial
- Controls 3x Pololu motors + 1x Servo
- Uses VNH5019 shield for 2 motors
- Uses L298N driver for 1 motor + servo

### Communication Flow
```
Leader Pi (encoders) → WiFi UDP → Follower Pi → USB Serial → Arduino Mega → Motors
```

### Optional System (Arduino Mega #2)
- I/O expansion, sensor fusion, or backup
- Can communicate with main follower via I2C/Serial

## Leader Wiring (Raspberry Pi + PCA9548A + AS5600)

### PCA9548A Multiplexer Connections
The PCA9548A connects to the Raspberry Pi's I2C bus (I2C1) on the 40-pin GPIO header:

| PCA9548A Pin | Raspberry Pi Connection | Physical Pin | GPIO/Function |
|--------------|-------------------------|--------------|---------------|
| VCC/VIN      | 3.3V                    | Pin 1 (or Pin 17) | 3.3V Power |
| GND          | GND                     | Pin 6 (or any GND) | Ground |
| SDA          | GPIO 2 (SDA1)           | Pin 3        | I2C1 Data |
| SCL          | GPIO 3 (SCL1)           | Pin 5        | I2C1 Clock |
| A0           | GND                     | Pin 6 (shared) | Address bit 0 (sets I2C address) |
| A1           | GND                     | Pin 6 (shared) | Address bit 1 (sets I2C address) |
| A2           | GND                     | Pin 6 (shared) | Address bit 2 (sets I2C address) |

**Note:** With A0, A1, A2 all tied to GND, the PCA9548A will have I2C address 0x70 (default). All address pins share the same GND connection and do not require separate GPIO pins.

### AS5600 Encoder Connections (4x encoders)
Each AS5600 connects to a separate PCA9548A channel to avoid I2C address conflicts (all AS5600 encoders share address 0x36):

**Joint 1 Encoder (Channel 0):**
| AS5600 Pin | PCA9548A Pin | Function |
|------------|--------------|----------|
| VDD        | VCC          | 3.3V Power |
| GND        | GND          | Ground |
| SDA        | SD0          | I2C Data (Channel 0) |
| SCL        | SC0          | I2C Clock (Channel 0) |
| DIR        | GND          | Direction (tied low) |
| OUT/GPO    | NC           | Not used |

**Joint 2 Encoder (Channel 1):**
| AS5600 Pin | PCA9548A Pin | Function |
|------------|--------------|----------|
| VDD        | VCC          | 3.3V Power |
| GND        | GND          | Ground |
| SDA        | SD1          | I2C Data (Channel 1) |
| SCL        | SC1          | I2C Clock (Channel 1) |
| DIR        | GND          | Direction (tied low) |
| OUT/GPO    | NC           | Not used |

**Joint 3 Encoder (Channel 2):**
| AS5600 Pin | PCA9548A Pin | Function |
|------------|--------------|----------|
| VDD        | VCC          | 3.3V Power |
| GND        | GND          | Ground |
| SDA        | SD2          | I2C Data (Channel 2) |
| SCL        | SC2          | I2C Clock (Channel 2) |
| DIR        | GND          | Direction (tied low) |
| OUT/GPO    | NC           | Not used |

**Joint 4 Encoder (Channel 3):**
| AS5600 Pin | PCA9548A Pin | Function |
|------------|--------------|----------|
| VDD        | VCC          | 3.3V Power |
| GND        | GND          | Ground |
| SDA        | SD3          | I2C Data (Channel 3) |
| SCL        | SC3          | I2C Clock (Channel 3) |
| DIR        | GND          | Direction (tied low) |
| OUT/GPO    | NC           | Not used |

## Follower System Wiring

### Follower Raspberry Pi to Arduino Mega Connection
The Follower Raspberry Pi connects to the Arduino Mega via USB serial:

| Connection | Details |
|------------|---------|
| **USB Cable** | Standard USB-A to USB-B cable |
| **Raspberry Pi Port** | USB-A port (any available USB port) |
| **Arduino Mega Port** | USB-B port (programming port) |
| **Communication** | Serial communication at configured baud rate (typically 115200) |
| **Power** | Arduino can be powered via USB (for testing) or external power supply |

**Note:** The Arduino Mega should be powered via external supply when driving motors. USB power is sufficient for testing and programming only.

## Follower Arduino Wiring (Arduino Mega + Motor Drivers)

### VNH5019 Shield Connections
The VNH5019 shield plugs directly onto the Arduino Mega:

| VNH5019 Pin | Arduino Pin | Function |
|-------------|-------------|----------|
| M1PWM       | Pin 9       | Motor 1 PWM |
| M1INA       | Pin 8       | Motor 1 Direction A |
| M1INB       | Pin 7       | Motor 1 Direction B |
| M2PWM       | Pin 10      | Motor 2 PWM |
| M2INA       | Pin 12      | Motor 2 Direction A |
| M2INB       | Pin 11      | Motor 2 Direction B |
| VIN         | Motor Power | 6-12V Motor Supply |
| GND         | GND         | Ground |

### L298N Driver Connections
| L298N Pin | Arduino Pin | Function |
|-----------|-------------|----------|
| ENA       | Pin 5       | Motor 3 Enable |
| IN1       | Pin 4       | Motor 3 Direction 1 |
| IN2       | Pin 3       | Motor 3 Direction 2 |
| OUT1      | Motor 3+    | Motor 3 Positive |
| OUT2      | Motor 3-    | Motor 3 Negative |
| VCC       | 5V          | Logic Power |
| GND       | GND         | Ground |
| VIN       | Motor Power | 6-12V Motor Supply |

### Servo Connections
| Servo Wire | Arduino Pin | Function |
|------------|-------------|----------|
| Red        | 5V          | Power |
| Black      | GND         | Ground |
| Yellow     | Pin 6       | Signal |

### Motor Assignments
- **Motor 1 (VNH5019)**: Joint 1 - Base rotation
- **Motor 2 (VNH5019)**: Joint 2 - Shoulder
- **Motor 3 (L298N)**: Joint 3 - Elbow
- **Servo (Pin 6)**: Joint 4 - Gripper

## Power System

### Power Requirements
- **Logic Power**: 5V/3.3V for Arduino, Pi, encoders
- **Motor Power**: 6-12V for motors (depends on motor specs)
- **Servo Power**: 5V for servo

### Power Distribution
```
Main Power Supply (12V)
├── Motor Power Bus (12V)
│   ├── VNH5019 Shield VIN
│   └── L298N VIN
├── 5V Regulator
│   ├── Arduino 5V
│   ├── Servo Power
│   └── Raspberry Pi 5V
└── 3.3V Regulator
    ├── AS5600 Encoders
    └── PCA9548A Multiplexer
```

## Safety Considerations

### Electrical Safety
- Use separate power supplies for logic and motors
- Implement proper grounding (common ground)
- Add fuses on power lines
- Use appropriate wire gauge for current requirements

### Mechanical Safety
- Implement hard E-stop that cuts motor power
- Add limit switches on joints
- Use appropriate motor current limits
- Implement software current limiting

### Software Safety
- Watchdog timeout (100ms) → HOLD mode
- CRC-16 validation on all communications
- Encoder validation and error detection
- Graceful shutdown procedures

## Testing and Validation

### Initial Testing
1. **Power On Test**: Verify all power rails
2. **I2C Test**: Check encoder communication
3. **Motor Test**: Test each motor individually
4. **Servo Test**: Verify servo positioning
5. **Communication Test**: Test BILK protocol

### System Integration
1. **Leader Test**: Verify encoder readings on Leader Pi
2. **WiFi Communication Test**: Verify UDP communication between Pi boards
3. **Follower Pi Test**: Verify data reception and processing
4. **USB Serial Test**: Verify communication between Follower Pi and Arduino Mega
5. **Motor Control Test**: Verify motor control on Arduino Mega
6. **Full System Test**: End-to-end operation (Leader → WiFi → Follower Pi → USB → Arduino → Motors)

## Troubleshooting

### Common Issues
- **I2C Communication**: Check pull-up resistors, wiring on Leader Pi
- **WiFi Communication**: Check network connectivity, IP addresses, firewall settings
- **USB Serial Communication**: Check USB cable connection, baud rate settings, device permissions (`/dev/ttyUSB0` or `/dev/ttyACM0`)
- **Motor Not Moving**: Check power, direction pins, PWM signals
- **Servo Issues**: Check power supply, signal wiring
- **Encoder Errors**: Check magnetic field, I2C address, multiplexer channel selection

### Debug Tools
- Arduino Serial Monitor for follower debugging
- Raspberry Pi I2C tools (`i2cdetect`, `i2cdump`)
- Multimeter for voltage/continuity checks
- Oscilloscope for signal analysis
