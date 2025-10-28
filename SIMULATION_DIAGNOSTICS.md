# BILK Simulation with Diagnostics

This enhanced simulation system provides comprehensive diagnostics and visualization for the BILK medical robotics system, including realistic Raspberry Pi leader simulation with AS5600 encoders and PCA9548A multiplexer.

## Quick Start

### Option 1: Simple Wrapper (Recommended)
```bash
# Run for 30 seconds (default)
./tools/run_simulation.sh

# Run for 60 seconds
./tools/run_simulation.sh -d 60

# Run for 2 minutes
./tools/run_simulation.sh --duration 120
```

### Option 2: Direct Python Script
```bash
# Activate virtual environment
source .venv/bin/activate

# Run simulation
python tools/simulate_with_diagnostics.py --duration 30
```

## What You Get

Each simulation run creates a timestamped directory (`simulation_logs_YYYYMMDD_HHMMSS/`) containing:

### 📊 Diagnostic Plots (`diagnostic_plots.png` & `.pdf`)
- **Joint Positions Over Time**: Shows all 4 joints' sinusoidal trajectories
- **3D Trajectory**: 3D visualization of the first 3 joints' path in space
- **Latency Analysis**: Timing analysis showing deviation from expected 200Hz
- **Joint Velocities**: Velocity profiles for all joints

### 📈 Performance Statistics (`summary_stats.json`)
- Simulation duration and sample count
- Average frequency (Hz)
- Timing statistics (mean, std, min, max intervals)
- Joint position ranges and movements

### 📝 Raw Data (`follower_data.json`)
- Complete position data for further analysis
- Timestamps for each sample
- Mode change events

### 📋 Process Logs
- `*_stdout.log`: Standard output from each process
- `*_stderr.log`: Error logs from each process

## Example Output

```
=== SIMULATION SUMMARY ===
Duration: 15.0 seconds
Total samples: 937
Average frequency: 158.5 Hz
Timing - Mean: 6.32ms, Std: 0.46ms
Joint ranges:
  J1: -1.000 to 1.000 (range: 2.000)
  J2: -0.600 to 0.600 (range: 1.200)
  J3: -0.289 to 0.300 (range: 0.589)
  J4: 0.150 to 0.150 (range: 0.000)
=====================================
```

## System Architecture

The simulation runs three processes in parallel:

1. **Simulated Leader** (`simulate_leader.py`)
   - Generates smooth sinusoidal trajectories for 4 joints
   - Streams at 200Hz via UDP port 9001
   - Uses CRC-16 for data integrity

2. **Host Bridge** (`host_bridge_udp_mirror.py`)
   - Receives data from leader on port 9001
   - Applies min-jerk smoothing for safety
   - Implements watchdog (HOLD mode if >100ms delay)
   - Forwards to follower on port 9011

3. **Virtual Follower** (`virtual_follower.py`)
   - Receives commands on port 9011
   - Parses BILK protocol frames
   - Logs all position data for analysis

## Raspberry Pi Leader Simulation

The new `simulate_pi_leader.py` provides a realistic simulation of the Raspberry Pi leader system:

### Features
- **AS5600 Encoder Simulation**: Realistic 12-bit magnetic encoder behavior
- **PCA9548A Multiplexer**: Simulates I2C channel switching with settling delays
- **Magnetic Field Validation**: Simulates magnetic field strength checks
- **Read Failures**: Occasional encoder read failures for realism
- **Noise and Drift**: Realistic encoder noise and magnetic field variations
- **I2C Timing**: Proper settling delays between channel switches

### Encoder Behavior
- 12-bit resolution (0-4095 counts)
- Magnetic field strength validation
- AGC (Automatic Gain Control) simulation
- Proper angle unwrapping for continuous rotation
- Low-pass filtering for velocity calculation

### Multiplexer Behavior
- 1ms settling delay after channel selection
- Realistic I2C communication timing
- Channel validation and error handling

## Dependencies

- Python 3.7+
- numpy
- matplotlib
- Standard library modules (socket, struct, time, etc.)

## Troubleshooting

### Port Already in Use
The script automatically cleans up existing processes, but if you encounter port conflicts:
```bash
pkill -f "simulate_leader.py|host_bridge_udp_mirror.py|virtual_follower.py"
```

### Missing Dependencies
```bash
pip install numpy matplotlib
```

### No Data Collected
- Check that all processes started successfully
- Verify UDP ports 9001 and 9011 are available
- Check stderr logs for error messages

## Customization

### Modify Trajectories
Edit `tools/simulate_leader.py` to change the joint trajectories:
```python
q = np.array([
    math.sin(2*math.pi*0.30*t),      # J1 frequency
    0.6*math.sin(2*math.pi*0.20*t),  # J2 amplitude & frequency
    0.3*math.sin(2*math.pi*0.12*t),  # J3 amplitude & frequency
    0.15                              # J4 constant
], dtype=float)
```

### Adjust Timing
Modify the frequency in `simulate_leader.py`:
```python
HZ = 200  # Change to desired frequency
```

### Customize Plots
Edit the plotting functions in `simulate_with_diagnostics.py` to add more visualizations or modify existing ones.

## Integration with Real Hardware

This simulation system validates the complete data flow before deploying to real hardware:

1. **Leader**: Replace `simulate_leader.py` with actual ESP32 firmware
2. **Host**: Use `host_bridge_udp.py` instead of the mirror version
3. **Follower**: Replace virtual follower with Arduino Mega running `Follower_Arduino.ino`

The diagnostic plots help verify that the system meets performance requirements before hardware deployment.
