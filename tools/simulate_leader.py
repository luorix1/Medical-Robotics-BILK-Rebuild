# tools/simulate_leader.py
# Simulates an ESP32 Leader by sending BILK LeaderState frames over UDP to localhost:9001
import socket, struct, time, math, signal, sys
import numpy as np

HOST = "127.0.0.1"
PORT = 9001
HZ = 200            # 200 Hz is stable on laptops; raise to 1000 if desired
DT = 1.0 / HZ

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def pack_leader_state(t_us: int, q, qd, buttons=0):
    payload = struct.pack("<I", t_us) \
        + struct.pack("<ffff", *q) \
        + struct.pack("<ffff", *qd) \
        + bytes([buttons]) + b"\x00\x00\x00"
    hdr = bytes([1, 0x01]) + struct.pack("<H", len(payload))
    crc = crc16_ccitt_false(hdr + payload)
    return b"BILK" + hdr + payload + struct.pack("<H", crc)

def signal_handler(signum, frame):
    print(f"\n[simulate_leader] Received signal {signum}, shutting down gracefully...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time()
    print(f"[simulate_leader] Streaming to {HOST}:{PORT} at {HZ} Hz. Ctrl+C to stop.")
    try:
        last_q = None
        while True:
            t = time.time() - t0
            # Smooth reference trajectory
            q = np.array([
                math.sin(2*math.pi*0.30*t),      # J1
                0.6*math.sin(2*math.pi*0.20*t),  # J2
                0.3*math.sin(2*math.pi*0.12*t),  # J3
                0.15                              # J4 const
            ], dtype=float)
            if last_q is None:
                qd = np.zeros_like(q)
            else:
                qd = (q - last_q) / DT
            last_q = q.copy()
            frame = pack_leader_state(int(t*1e6), q.tolist(), qd.tolist(), buttons=0)
            sock.sendto(frame, (HOST, PORT))
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\n[simulate_leader] Interrupted, shutting down...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
