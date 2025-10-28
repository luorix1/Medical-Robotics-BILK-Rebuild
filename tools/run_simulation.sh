#!/bin/bash
# tools/run_simulation.sh
# Simple wrapper for running BILK simulation with diagnostics

set -euo pipefail

# Default duration
DURATION=30

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -d, --duration SECONDS    Simulation duration (default: 30)"
            echo "  -h, --help               Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                       # Run for 30 seconds"
            echo "  $0 -d 60                 # Run for 60 seconds"
            echo "  $0 --duration 120        # Run for 2 minutes"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Change to project directory
cd "$(dirname "$0")/.."

# Activate virtual environment
if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
else
    echo "Error: Virtual environment not found. Please run setup first:"
    echo "  python3 -m venv .venv"
    echo "  source .venv/bin/activate"
    echo "  pip install numpy matplotlib"
    exit 1
fi

# Run simulation
echo "Starting BILK simulation for $DURATION seconds..."
python tools/simulate_with_diagnostics.py --duration "$DURATION"

echo ""
echo "Simulation complete! Check the simulation_logs_* directory for:"
echo "  - diagnostic_plots.png/pdf: Visual analysis plots"
echo "  - summary_stats.json: Performance statistics"
echo "  - follower_data.json: Raw position data"
echo "  - *_stdout.log: Process output logs"
