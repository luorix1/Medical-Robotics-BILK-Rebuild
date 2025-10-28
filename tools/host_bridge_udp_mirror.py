# tools/host_bridge_udp_mirror.py
# Host-like bridge for offline simulation:
#  - Receives UDP LeaderState on :9001
#  - Applies min-jerk smoothing & watchdog
#  - Emits follower frames to UDP :9011 instead of serial
import socket, struct, time, select, signal, sys
import numpy as np
from collections import deque

PRE = b"BILK"; MSG_LEADER_STATE=0x01; MSG_SET_MODE=0x10
PORT_IN = 9001      # from simulated ESP32
PORT_OUT = 9011     # to virtual follower

LATENCY_THRESH_MS = 100

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

def pack_frame(msg_type:int, payload:bytes)->bytes:
    hdr = bytes([1, msg_type]) + struct.pack("<H", len(payload))
    crc = crc16_ccitt_false(hdr + payload)
    return PRE + hdr + payload + struct.pack("<H", crc)

def parse_frame(buf:bytes):
    if len(buf) < 10 or buf[0:4] != PRE: return None
    hdr = buf[4:8]; ver, msg, plen = hdr[0], hdr[1], struct.unpack("<H", hdr[2:4])[0]
    if len(buf) != 8 + plen + 2: return None
    payload = buf[8:8+plen]; crc = struct.unpack("<H", buf[8+plen:8+plen+2])[0]
    if crc16_ccitt_false(hdr + payload) != crc: return None
    return (msg, payload)

def minjerk_filter(hist:deque, window=0.050):
    now = time.time()
    while hist and (now - hist[0][0]) > window:
        hist.popleft()
    if not hist: return None
    qs = [x[1] for x in hist]
    return np.mean(np.stack(qs, axis=0), axis=0)

def signal_handler(signum, frame):
    print(f"\n[host_mirror] Received signal {signum}, shutting down gracefully...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", PORT_IN))
    rx.setblocking(False)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    hist = deque()
    last_forward = time.time()

    print(f"[host_mirror] UDP in :{PORT_IN}, UDP out :{PORT_OUT}")
    try:
        while True:
            ready,_,_ = select.select([rx], [], [], 0.02)
            got = False
            if ready:
                data, addr = rx.recvfrom(2048)
                f = parse_frame(data)
                if f and f[0] == MSG_LEADER_STATE:
                    pl = f[1]
                    t_us = struct.unpack("<I", pl[0:4])[0]
                    q  = np.array(struct.unpack("<ffff", pl[4:20]))
                    qd = struct.unpack("<ffff", pl[20:36])
                    buttons = pl[36]
                    hist.append((time.time(), q))
                    q_s = minjerk_filter(hist)
                    if q_s is None:
                        q_s = q
                    # rebuild follower-bound payload
                    out_pl = struct.pack("<I", t_us) \
                           + struct.pack("<ffff", *q_s) \
                           + struct.pack("<ffff", *qd) \
                           + bytes([buttons]) + b"\x00\x00\x00"
                    frame = pack_frame(MSG_LEADER_STATE, out_pl)
                    tx.sendto(frame, ("127.0.0.1", PORT_OUT))
                    last_forward = time.time()
                    got = True
            # Watchdog -> HOLD
            if (time.time() - last_forward)*1000 > LATENCY_THRESH_MS:
                frame = pack_frame(MSG_SET_MODE, bytes([2,0,0,0]))  # HOLD
                tx.sendto(frame, ("127.0.0.1", PORT_OUT))
                last_forward = time.time()
    except KeyboardInterrupt:
        print("\n[host_mirror] Interrupted, shutting down...")
    finally:
        rx.close()
        tx.close()

if __name__ == "__main__":
    main()
