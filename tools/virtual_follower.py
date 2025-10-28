# tools/virtual_follower.py
# Simulated Arduino Mega follower: listens on UDP :9011 and prints parsed commands/modes.
import socket, struct, select, time, signal, sys

PRE = b"BILK"; MSG_LEADER_STATE=0x01; MSG_SET_MODE=0x10
PORT_IN = 9011

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

def parse_frame(buf:bytes):
    if len(buf) < 10 or buf[0:4] != PRE: return None
    hdr = buf[4:8]; ver, msg, plen = hdr[0], hdr[1], struct.unpack("<H", hdr[2:4])[0]
    if len(buf) != 8 + plen + 2: return None
    payload = buf[8:8+plen]; crc = struct.unpack("<H", buf[8+plen:8+plen+2])[0]
    if crc16_ccitt_false(hdr + payload) != crc: return None
    return (msg, payload)

def signal_handler(signum, frame):
    print(f"\n[virtual_follower] Received signal {signum}, shutting down gracefully...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", PORT_IN))
    sock.setblocking(False)

    last = time.time()
    print(f"[virtual_follower] Listening on UDP :{PORT_IN}")
    try:
        while True:
            ready,_,_ = select.select([sock], [], [], 0.1)
            if ready:
                data, addr = sock.recvfrom(2048)
                f = parse_frame(data)
                if not f:
                    continue
                msg, pl = f
                if msg == MSG_LEADER_STATE:
                    t_us = struct.unpack("<I", pl[0:4])[0]
                    q    = struct.unpack("<ffff", pl[4:20])
                    print(f"[virtual_follower] FOLLOW q={tuple(round(x,3) for x in q)} t_us={t_us}")
                elif msg == MSG_SET_MODE:
                    mode = pl[0]
                    name = {0:'IDLE',1:'FOLLOW',2:'HOLD',3:'SHUTDOWN'}.get(mode, f'UNK({mode})')
                    print(f"[virtual_follower] MODE={name}")
            # Optional: print idle heartbeat
            if time.time() - last > 2.0:
                print("[virtual_follower] ...")
                last = time.time()
    except KeyboardInterrupt:
        print("\n[virtual_follower] Interrupted, shutting down...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
