# BILK Revamp - Medical Robotics System

A hybrid wireless medical robotics system featuring real-time leader-follower control with AS5600 magnetic encoders and safety-critical communication protocols.

## 🏗️ System Architecture

```
┌─────────────────┐    WiFi UDP     ┌─────────────────┐    USB Serial    ┌─────────────────┐
│   Leader ESP32  │ ──────────────► │  Host Bridge    │ ──────────────► │ Follower Arduino│
│                 │    :9001        │   (RPi/PC)      │    2 Mbps       │      Mega       │
│ • AS5600 Encoders│                 │                 │                 │                 │
│ • TCA9548A Mux  │                 │ • Smoothing     │                 │ • PID Control   │
│ • WiFi + USB    │                 │ • Watchdog      │                 │ • Motor Drivers │
└─────────────────┘                 └─────────────────┘                 └─────────────────┘
```

## 📁 System Components & File Mapping

### **Leader System (ESP32)**
- **File**: `firmware/leader_esp32/Leader_ESP32_AS5600_UDP.ino`
- **Hardware**: ESP32 + 4x AS5600 encoders + TCA9548A multiplexer
- **Function**: Reads joint positions, streams via WiFi UDP + USB backup
- **Frequency**: 1kHz (1ms intervals)
- **Protocol**: BILK with CRC-16 validation

### **Host System (Raspberry Pi/PC)**
- **File**: `tools/host_bridge_udp.py`
- **Hardware**: Raspberry Pi 4+ or PC with USB ports
- **Function**: Data smoothing, safety watchdog, protocol bridging
- **Safety**: 100ms watchdog timeout → HOLD mode
- **Fallback**: USB serial from leader if WiFi fails

### **Follower System (Arduino Mega)**
- **File**: `firmware/follower_arduino/Follower_Arduino.ino`
- **Hardware**: Arduino Mega 2560 + motor drivers
- **Function**: PID control, motor actuation, safety monitoring
- **Control**: Configurable PID parameters per joint
- **Safety**: Communication loss → HOLD mode

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
- **Leader**: Connect AS5600 encoders to TCA9548A multiplexer
- **Follower**: Connect motor drivers to Arduino Mega
- **Host**: Ensure WiFi connectivity and USB ports available

### 3. Configuration
- **Leader**: Update WiFi credentials in `Leader_ESP32_AS5600_UDP.ino`
- **Follower**: Configure PWM/DIR pins in `Follower_Arduino.ino`
- **Host**: Set correct USB device paths in host bridge

### 4. Deployment
```bash
# Flash firmware
# 1. Upload Leader_ESP32_AS5600_UDP.ino to ESP32
# 2. Upload Follower_Arduino.ino to Arduino Mega

# Start host bridge
python tools/host_bridge_udp.py /dev/ttyUSB_FOLLOWER /dev/ttyUSB_LEADER
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
| **Multiplexer** | TCA9548A | 8-channel I2C |
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
- ESP32 development board
- Arduino Mega 2560
- 4x AS5600 magnetic encoders
- TCA9548A I2C multiplexer
- Motor drivers (PWM/DIR interface)
- Raspberry Pi 4+ or PC (host system)

### **Software Requirements**
- Python 3.7+
- Arduino IDE
- ESP32 board package
- See `requirements.txt` for Python dependencies

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly (simulation + hardware)
5. Submit a pull request

## 📄 License

[Add your license information here]

## 📞 Support

For technical support or questions:
- Check `docs/TROUBLESHOOTING.md` for common issues
- Review `AS5600_ANALYSIS.md` for encoder-specific problems
- Use simulation tools for testing and validation