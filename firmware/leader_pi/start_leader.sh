#!/bin/bash
# start_leader.sh - Start BILK Leader on Raspberry Pi
# Run with: bash start_leader.sh

set -euo pipefail

echo "🏥 BILK Leader Pi - Starting AS5600 Encoder System"
echo "=================================================="

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo "⚠️  Warning: This script is designed for Raspberry Pi"
fi

# Check Python version
echo "📋 Checking Python version..."
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3.7+ first."
    exit 1
fi

PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "✅ Python $PYTHON_VERSION found"

# Check if I2C is enabled
echo "🔧 Checking I2C configuration..."
if [ ! -e /dev/i2c-1 ]; then
    echo "❌ I2C is not enabled. Please enable I2C in raspi-config:"
    echo "   sudo raspi-config"
    echo "   -> Interface Options -> I2C -> Enable"
    exit 1
fi
echo "✅ I2C interface found"

# Check for required packages
echo "📦 Checking Python dependencies..."
if ! python3 -c "import smbus" 2>/dev/null; then
    echo "Installing required packages..."
    pip3 install -r requirements.txt
fi
echo "✅ Dependencies satisfied"

# Check I2C devices
echo "🔍 Scanning I2C devices..."
i2cdetect -y 1 | grep -E "(70|36)" || echo "⚠️  Warning: PCA9548A (0x70) or AS5600 (0x36) not detected"

# Make script executable
chmod +x leader_pi_as5600.py

# Start the leader
echo "🚀 Starting BILK Leader..."
echo "Press Ctrl+C to stop"
echo ""

python3 leader_pi_as5600.py
