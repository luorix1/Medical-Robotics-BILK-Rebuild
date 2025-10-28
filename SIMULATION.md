# Offline Simulation (Mac/Linux)

This lets you test **everything** without hardware:
```
Simulated Leader (UDP) → Host (smoothing, watchdog) → Virtual Follower (console)
```

## Quick Start
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install numpy
# In the project root:
bash tools/simulate_full_system.sh
```

You should see:
- `simulate_leader.py` streaming to UDP :9001
- `host_bridge_udp_mirror.py` forwarding to UDP :9011 (with smoothing + watchdog)
- `virtual_follower.py` printing FOLLOW q=... and MODE=... lines

Stop with **Ctrl+C** (the script will clean up processes).

## Manual Steps (if you prefer)
1. Terminal A:
   ```bash
   python tools/virtual_follower.py
   ```
2. Terminal B:
   ```bash
   python tools/host_bridge_udp_mirror.py
   ```
3. Terminal C:
   ```bash
   python tools/simulate_leader.py
   ```

## What this validates
- BILK frame structure (CRC-16)
- Host smoothing and watchdog → HOLD
- Mode transitions (try killing the leader; watch HOLD)
- Latency/jitter (combine with tools/latency_logger.py if desired)

## Next
When you’re happy, switch to real hardware:
- Replace `host_bridge_udp_mirror.py` with `tools/host_bridge_udp.py`
- Start your real follower on `/dev/ttyUSB*`
- Keep `simulate_leader.py` around for quick tests and demos
