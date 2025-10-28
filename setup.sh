#!/bin/bash
# BILK Medical Robotics System - Environment Setup Script
# Run with: bash setup.sh

set -euo pipefail

echo "🏥 BILK Medical Robotics System - Environment Setup"
echo "=================================================="

# Check Python version
echo "📋 Checking Python version..."
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.7+ first."
    exit 1
fi

PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "✅ Python $PYTHON_VERSION found"

# Create virtual environment
echo "🔧 Creating virtual environment..."
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
    echo "✅ Virtual environment created"
else
    echo "✅ Virtual environment already exists"
fi

# Activate virtual environment
echo "🔌 Activating virtual environment..."
source .venv/bin/activate

# Upgrade pip
echo "⬆️  Upgrading pip..."
pip install --upgrade pip

# Install requirements
echo "📦 Installing Python dependencies..."
pip install -r requirements.txt

# Make scripts executable
echo "🔧 Making scripts executable..."
chmod +x tools/run_simulation.sh
chmod +x tools/simulate_with_diagnostics.py

# Create simulation logs directory
echo "📁 Creating simulation logs directory..."
mkdir -p simulation_logs

echo ""
echo "🎉 Setup complete!"
echo ""
echo "Next steps:"
echo "1. Activate the virtual environment:"
echo "   source .venv/bin/activate"
echo ""
echo "2. Test the simulation:"
echo "   ./tools/run_simulation.sh -d 10"
echo ""
echo "3. Configure hardware:"
echo "   - Update WiFi credentials in firmware/leader_esp32/Leader_ESP32_AS5600_UDP.ino"
echo "   - Configure motor pins in firmware/follower_arduino/Follower_Arduino.ino"
echo ""
echo "4. Flash firmware to your hardware:"
echo "   - Upload Leader_ESP32_AS5600_UDP.ino to ESP32"
echo "   - Upload Follower_Arduino.ino to Arduino Mega"
echo ""
echo "5. Run the system:"
echo "   python tools/host_bridge_udp.py /dev/ttyUSB_FOLLOWER /dev/ttyUSB_LEADER"
echo ""
echo "For more information, see README.md"
