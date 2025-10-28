# tools/host_bridge_udp.py
import serial, time, struct, sys, socket, select, numpy as np
from collections import deque
PRE=b"BILK"; MSG_LEADER_STATE=0x01; MSG_SET_MODE=0x10
UDP_PORT=9001; LATENCY_THRESH_MS=100
def crc16_ccitt_false(data: bytes) -> int:
    crc=0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000: crc = ((crc<<1)&0xFFFF) ^ 0x1021
            else: crc = (crc<<1)&0xFFFF
    return crc
def pack_frame(msg_type:int, payload:bytes)->bytes:
    hdr=bytes([1,msg_type])+struct.pack("<H",len(payload))
    crc=crc16_ccitt_false(hdr+payload)
    return PRE+hdr+payload+struct.pack("<H",crc)
def parse_frame(buf:bytes):
    if len(buf)<10 or buf[0:4]!=PRE: return None
    hdr=buf[4:8]; ver,msg,plen=hdr[0],hdr[1],struct.unpack("<H",hdr[2:4])[0]
    if len(buf)!=8+plen+2: return None
    payload=buf[8:8+plen]; crc=struct.unpack("<H",buf[8+plen:8+plen+2])[0]
    if crc16_ccitt_false(hdr+payload)!=crc: return None
    return (msg,payload)
def minjerk_filter(hist:deque, window=0.050):
    t=time.time()
    while hist and (t-hist[0][0])>window: hist.popleft()
    if not hist: return None
    qs=[x[1] for x in hist]; return np.mean(np.stack(qs,axis=0),axis=0)
def main(follower_port="/dev/ttyUSB1", leader_serial="/dev/ttyUSB0", baud=2000000):
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.bind(("0.0.0.0",UDP_PORT)); sock.setblocking(False)
    ser_follow=serial.Serial(follower_port, baudrate=baud, timeout=0.001)
    try: ser_leader=serial.Serial(leader_serial, baudrate=baud, timeout=0.001)
    except Exception: ser_leader=None
    hist=deque(); last_forward=time.time()
    while True:
        got=False
        ready,_,_=select.select([sock],[],[],0.0)
        if ready:
            try:
                data,addr=sock.recvfrom(512)
                f=parse_frame(data)
                if f and f[0]==MSG_LEADER_STATE:
                    pl=f[1]; t_us=struct.unpack("<I",pl[0:4])[0]
                    q = np.array(struct.unpack("<ffff",pl[4:20]))
                    qd= struct.unpack("<ffff",pl[20:36]); buttons=pl[36]
                    hist.append((time.time(),q)); q_s=minjerk_filter(hist) or q
                    pl2=struct.pack("<I",t_us)+struct.pack("<ffff",*q_s)+struct.pack("<ffff",*qd)+bytes([buttons])+b"\x00\x00\x00"
                    ser_follow.write(pack_frame(MSG_LEADER_STATE,pl2)); last_forward=time.time(); got=True
            except BlockingIOError: pass
        if not got and ser_leader and ser_leader.in_waiting>=10:
            b=ser_leader.read(256)
            for i in range(max(0,len(b)-10)):
                if b[i:i+4]==PRE and i+8<=len(b):
                    plen=struct.unpack("<H",b[i+6:i+8])[0]; end=i+8+plen+2
                    if end<=len(b):
                        f=parse_frame(b[i:end])
                        if f and f[0]==MSG_LEADER_STATE:
                            pl=f[1]; t_us=struct.unpack("<I",pl[0:4])[0]
                            q = np.array(struct.unpack("<ffff",pl[4:20]))
                            qd= struct.unpack("<ffff",pl[20:36]); buttons=pl[36]
                            hist.append((time.time(),q)); q_s=minjerk_filter(hist) or q
                            pl2=struct.pack("<I",t_us)+struct.pack("<ffff",*q_s)+struct.pack("<ffff",*qd)+bytes([buttons])+b"\x00\x00\x00"
                            ser_follow.write(pack_frame(MSG_LEADER_STATE,pl2)); last_forward=time.time(); got=True; break
        if (time.time()-last_forward)*1000 > LATENCY_THRESH_MS:
            ser_follow.write(pack_frame(MSG_SET_MODE, bytes([2,0,0,0])))
if __name__=="__main__":
    args=sys.argv[1:]; kwargs={}
    if len(args)>=1: kwargs["follower_port"]=args[0]
    if len(args)>=2: kwargs["leader_serial"]=args[1]
    main(**kwargs)
