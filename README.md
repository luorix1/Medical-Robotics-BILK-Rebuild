# BILK Revamp - Medical Robotics System

A hybrid wireless medical robotics system featuring real-time leader-follower control with AS5600 magnetic encoders and safety-critical communication protocols.

## 🏗️ System Architecture

```
┌─────────────────┐    WiFi UDP     ┌─────────────────┐    USB Serial    ┌─────────────────┐
│  Leader Pi #1   │ ──────────────► │ Host Bridge Pi  │ ──────────────► │ Follower Arduino│
│                 │    :9001        │      #2         │    2 Mbps       │      Mega #1    │
│ • AS5600 Encoders│                 │                 │                 │                 │
│ • PCA9548A Mux  │                 │ • Smoothing     │                 │ • PID Control   │
│ • Python I2C    │                 │ • Watchdog      │                 │ • VNH5019 Shield│
└─────────────────┘                 └─────────────────┘                 │ • L298N Driver  │
                                                                        │ • 3x Pololu + 1x│
                                                                        │   Servo Motors  │
                                                                        └─────────────────┘
                                                                        
┌─────────────────┐
│ Arduino Mega #2 │  (Optional: I/O expansion, sensor fusion, or backup)
│                 │
│ • Additional I/O│
│ • Sensor Fusion │
│ • Backup System │
└─────────────────┘
```

**Available Hardware:**
- 2x Raspberry Pi boards
- 2x Arduino Mega 2560
- 4x AS5600 magnetic encoders
- 1x PCA9548A I2C multiplexer
- 1x Pololu dual VNH5019 motor driver shield
- 1x L298N motor driver
- 3x Pololu motors
- 1x Servo motor

## 📁 System Components & File Mapping

### **Leader System (Raspberry Pi #1)**
- **File**: `firmware/leader_pi/leader_pi_as5600.py`
- **Hardware**: Raspberry Pi + 4x AS5600 encoders + PCA9548A multiplexer
- **Function**: Reads joint positions via I2C, streams via WiFi UDP + USB backup
- **Frequency**: 1kHz (1ms intervals)
- **Protocol**: BILK with CRC-16 validation
- **Language**: Python with smbus for I2C communication

### **Host System (Raspberry Pi #2)**
- **File**: `tools/host_bridge_udp.py`
- **Hardware**: Raspberry Pi with USB ports
- **Function**: Data smoothing, safety watchdog, protocol bridging
- **Safety**: 100ms watchdog timeout → HOLD mode
- **Fallback**: USB serial from leader if WiFi fails

### **Follower System (Arduino Mega #1)**
- **File**: `firmware/follower_arduino/Follower_Arduino.ino`
- **Hardware**: Arduino Mega 2560 + VNH5019 shield + L298N driver
- **Motors**: 3x Pololu motors + 1x Servo motor
- **Function**: PID control, motor actuation, safety monitoring
- **Control**: Configurable PID parameters per joint
- **Safety**: Communication loss → HOLD mode
- **Drivers**: VNH5019 for 2x Pololu motors, L298N for 1x Pololu + 1x Servo

### **Optional System (Arduino Mega #2)**
- **Purpose**: I/O expansion, sensor fusion, or backup system
- **Function**: Additional sensors, data logging, or redundant control
- **Communication**: I2C or Serial with main follower

## 🚀 Quick Start

### 1. Environment Setup
```bash
# Clone repository
git clone <repository-url>
cd bilk_revamp

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Hardware Setup
- **Leader Pi #1**: Connect AS5600 encoders to PCA9548A multiplexer via I2C
- **Host Pi #2**: Connect USB cables to Arduino Mega
- **Follower**: Connect motor drivers to Arduino Mega

### 3. Configuration
- **Leader Pi**: Update HOST_IP in `leader_pi_as5600.py`
- **Follower**: Configure PWM/DIR pins in `Follower_Arduino.ino`
- **Host Pi**: Set correct USB device paths in host bridge

### 4. Deployment
```bash
# Setup Leader Pi #1
cd firmware/leader_pi
pip3 install -r requirements.txt
bash start_leader.sh

# Setup Host Pi #2
python tools/host_bridge_udp.py /dev/ttyUSB_FOLLOWER /dev/ttyUSB_LEADER

# Flash Follower Arduino
# Upload Follower_Arduino.ino to Arduino Mega
```

## 🧪 Simulation & Testing

### **Offline Simulation**
```bash
# Run complete simulation with diagnostics
./tools/run_simulation.sh -d 30

# Or manually
python tools/simulate_with_diagnostics.py --duration 30
```

### **What You Get**
- Real-time joint position visualization
- 3D trajectory plots
- Latency analysis and performance metrics
- Comprehensive diagnostic reports

## 📊 System Specifications

| Component | Specification | Details |
|-----------|---------------|---------|
| **Sampling Rate** | 1kHz | 1ms intervals |
| **Communication** | WiFi UDP + USB | Redundant connectivity |
| **Protocol** | BILK | CRC-16 validated frames |
| **Encoders** | AS5600 | 12-bit magnetic (0-4095) |
| **Multiplexer** | PCA9548A | 8-channel I2C |
| **Safety** | Watchdog | 100ms timeout → HOLD |
| **Latency** | <5ms | End-to-end system |

## 🔧 Configuration Files

### **Leader Configuration**
```cpp
// WiFi Settings
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
IPAddress HOST_IP(192, 168, 1, 100);

// Hardware Pins
#define I2C_SDA 21
#define I2C_SCL 22
#define PIN_BTN_FINE 32
#define PIN_ESTOP_IN 33
```

### **Follower Configuration**
```cpp
// PID Parameters (tune for your system)
float Kp[4] = {5, 5, 5, 2};      // Proportional gains
float Ki[4] = {0.3, 0.2, 0.2, 0}; // Integral gains  
float Kd[4] = {0.05, 0.05, 0.05, 0}; // Derivative gains

// Watchdog timeout
const uint32_t kWatchdogUs = 100000; // 100ms

✅ Leader Wiring (Raspberry Pi + PCA9548A + AS5600 Encoders)
✨ Signal Flow

AS5600 sensors → PCA9548A multiplexer → Raspberry Pi I²C bus → Wi-Fi UDP

📌 Connections
Raspberry Pi Pin	Connects To	Notes
3.3V (Pin 1)	PCA9548A VCC	Do NOT use 5V here (I²C should be 3.3V)
GND (Pin 6)	PCA9548A GND	Must share ground with AS5600 sensors
GPIO 2 (SDA, Pin 3)	PCA9548A SDA	Use short twisted pair recommended
GPIO 3 (SCL, Pin 5)	PCA9548A SCL	Add pull-ups if not on breakout
GPIO 18 (Pin 12)	Fine Control Button	Active LOW (use GND side)
GPIO 24 (Pin 18)	E-Stop Button Input	Active LOW (GND triggers HOLD mode)

✅ Use breakout with LEVEL SHIFTING removed (AS5600 is 3.3V-only I²C)

🔀 Multiplexer Routing (I²C Switching)
Joint	PCA Channel	Connects To	Sensor Address
J1 (Base)	0	SDA0 / SCL0 → AS5600	0x36
J2 (Shoulder)	1	SDA1 / SCL1 → AS5600	0x36
J3 (Elbow)	2	SDA2 / SCL2 → AS5600	0x36
J4 (Wrist)	3	SDA3 / SCL3 → AS5600	0x36

They all use the same I²C address — multiplexer isolates them ✅

🧲 Magnet Placement (critical!)

Use diametrically magnetized cylinder magnet (6–8 mm recommended)

Mount magnet centered on the joint's axis

Maintain 1–2 mm gap to AS5600 face

Place chip coaxial with shaft

⚠️ Off-axis rotation causes non-linear angle reading and jerk → filter needed.

✅ Host Wiring (Raspberry Pi / PC)

You don’t wire much to the Host — just USB cables.

Cable	Connects	Notes
USB #1	Pi ↔ Arduino Mega	Control path to motors and servos
USB #2	Pi ↔ ESP32 (optional)	Fallback Leader comm when Wi-Fi drops

If using a desktop PC: same idea — treat it as the Host

✅ Follower Wiring (Arduino Mega + Motor Drivers)

This depends on your actuator hardware — but here is the standard teleop robotic arm wiring:

Servo / Motor Signal Routing
Arduino Mega Pin	Function	Connects To
PWM Pins (e.g., 6,7,8)	Joint Motor Control	Motor Driver PWM inputs
Digital DIR Pins (e.g., 24,25,26)	Motor Direction	Motor Driver DIR inputs
Pin 9	Gripper Servo Control	Servo signal pin
GND	Servo & Driver Ground	All logic GND must be common

⚠️ Do NOT power motors from Arduino 5V pin
Use a dedicated motor supply

Power System ✅ Required for Safety
Power Path	Voltage	Details
Logic (Arduino + Leader + Sensors)	5V / 3.3V	Common ground ✅
Motor Supply	6–12V (depends)	Isolated from logic
Hard E-Stop	Cuts ONLY motor power	Relay or switch controlled
```

## 🛡️ Safety Features

### **Hardware Safety**
- **Hard E-Stop**: Physical relay cuts motor power
- **Power Isolation**: Separate power domains
- **Common Ground**: Proper grounding scheme

### **Software Safety**
- **CRC-16 Validation**: Data integrity checking
- **Watchdog Timeout**: Communication loss detection
- **Mode Management**: IDLE/FOLLOW/HOLD/SHUTDOWN
- **Encoder Validation**: Magnetic field strength checking

### **Emergency Procedures**
1. **Hard E-Stop**: Physical button cuts all power
2. **Software HOLD**: Automatic on communication loss
3. **Encoder Failure**: System detects and reports issues
4. **WiFi Loss**: Automatic fallback to USB

## 📚 Documentation

| File | Purpose |
|------|---------|
| `docs/RUN.md` | Detailed setup and operation instructions |
| `docs/SAFETY.md` | Safety procedures and requirements |
| `docs/TROUBLESHOOTING.md` | Common issues and solutions |
| `firmware/README.md` | Firmware-specific documentation |
| `AS5600_ANALYSIS.md` | Encoder implementation analysis |
| `SIMULATION_DIAGNOSTICS.md` | Simulation and testing procedures |

## 🔍 Troubleshooting

### **Common Issues**
- **WiFi Connection**: Check SSID/password and network
- **Encoder Errors**: Verify magnetic field strength and positioning
- **Communication Loss**: Check USB connections and permissions
- **Timing Issues**: Monitor system load and I2C timing

### **Diagnostic Tools**
- **Simulation**: Test without hardware using `tools/run_simulation.sh`
- **Serial Monitor**: Real-time diagnostics from ESP32
- **Performance Plots**: Latency and trajectory analysis
- **Encoder Health**: Magnetic field and AGC monitoring

## 🏥 Medical Robotics Applications

This system is designed for medical robotics applications requiring:
- **High Precision**: 12-bit encoder resolution
- **Real-time Control**: 1kHz sampling rate
- **Safety Critical**: Multiple safety layers
- **Reliable Communication**: Redundant connectivity
- **Diagnostic Capabilities**: Comprehensive monitoring

## 📋 Requirements

### **Hardware Requirements**
- 2x Raspberry Pi boards (Pi 4+ recommended)
- 2x Arduino Mega 2560
- 4x AS5600 magnetic encoders
- 1x PCA9548A I2C multiplexer
- 1x Pololu dual VNH5019 motor driver shield
- 1x L298N motor driver
- 3x Pololu motors
- 1x Servo motor
- I2C breakout boards and wiring

### **Software Requirements**
- Python 3.7+ (on all Raspberry Pi boards)
- Arduino IDE (for Arduino Mega)
- I2C tools (i2cdetect, i2cdump)
- See `requirements.txt` for Python dependencies
