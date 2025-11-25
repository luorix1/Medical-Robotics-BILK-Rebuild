#!/bin/bash
# start_follower.sh - Startup script for BILK Follower Pi Bridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check Python version
if ! command -v python3 &> /dev/null; then
    echo "ERROR: python3 not found"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
REQUIRED_VERSION="3.7"

if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$PYTHON_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
    echo "ERROR: Python 3.7+ required, found $PYTHON_VERSION"
    exit 1
fi

# Check dependencies
echo "Checking dependencies..."
python3 -c "import serial" 2>/dev/null || {
    echo "ERROR: pyserial not installed"
    echo "Install with: pip3 install pyserial"
    exit 1
}

python3 -c "import numpy" 2>/dev/null || {
    echo "ERROR: numpy not installed"
    echo "Install with: pip3 install numpy"
    exit 1
}

# Find Arduino port
ARDUINO_PORT="${1:-/dev/ttyUSB0}"

if [ ! -e "$ARDUINO_PORT" ]; then
    echo "WARNING: Arduino port $ARDUINO_PORT not found"
    echo "Available ports:"
    ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (none found)"
    echo ""
    echo "Usage: $0 [arduino_port]"
    echo "Example: $0 /dev/ttyACM0"
    exit 1
fi

# Check permissions
if [ ! -r "$ARDUINO_PORT" ] || [ ! -w "$ARDUINO_PORT" ]; then
    echo "WARNING: May not have permissions for $ARDUINO_PORT"
    echo "Add user to dialout group: sudo usermod -a -G dialout $USER"
    echo "Then log out and back in"
fi

echo "Starting BILK Follower Pi Bridge..."
echo "  Arduino Port: $ARDUINO_PORT"
echo ""

# Run the bridge
exec python3 follower_pi_bridge.py "$ARDUINO_PORT"

