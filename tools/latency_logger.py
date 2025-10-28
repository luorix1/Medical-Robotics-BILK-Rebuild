# tools/latency_logger.py
import time, socket, select, csv
UDP_PORT=9001
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.bind(("0.0.0.0", UDP_PORT)); sock.setblocking(False)
last=None
with open("latency_log.csv","w",newline="") as f:
    w=csv.writer(f); w.writerow(["t_host","dt_arrival_ms"])
    while True:
        ready,_,_=select.select([sock],[],[],0.1)
        if ready:
            data,addr=sock.recvfrom(512); t=time.time()
            if last is not None: w.writerow([t, (t-last)*1000.0]); f.flush()
            last=t
