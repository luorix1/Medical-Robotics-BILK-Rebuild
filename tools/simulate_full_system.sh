#!/usr/bin/env bash
# tools/simulate_full_system.sh
# Starts the full offline simulation pipeline on macOS/Linux terminals.
# 1) Virtual follower (UDP :9011)
# 2) Host bridge (mirror) (UDP in :9001, out :9011)
# 3) Simulated leader (UDP :9001)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Start virtual follower
python tools/virtual_follower.py &
PID_VF=$!
sleep 0.2

# Start host mirror
python tools/host_bridge_udp_mirror.py &
PID_H=$!
sleep 0.2

# Start simulated leader
python tools/simulate_leader.py &
PID_L=$!

echo "[simulate_full_system] PIDs: virtual_follower=$PID_VF host=$PID_H leader=$PID_L"
echo "[simulate_full_system] Press Ctrl+C to stop"
cleanup() {
    echo "[simulate_full_system] Shutting down gracefully..."
    kill $PID_L $PID_H $PID_VF 2>/dev/null || true
    sleep 0.5
    # Force kill if still running
    kill -9 $PID_L $PID_H $PID_VF 2>/dev/null || true
    echo "[simulate_full_system] Cleanup complete"
    exit 0
}
trap cleanup INT TERM

wait
