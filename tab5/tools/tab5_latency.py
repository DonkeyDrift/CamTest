#!/usr/bin/env python3
"""tab5_latency.py — 带 P85 时钟同步的端到端时延测量（WS 帧 + /api/sync）。
用法: tab5_latency.py <host> [秒数=15]"""
import sys, time, socket, base64, os, struct, json, urllib.request, threading

host = sys.argv[1] if len(sys.argv) > 1 else "192.168.3.44"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

def http_json(path):
    req = urllib.request.Request("http://" + host + path)
    return json.loads(urllib.request.urlopen(req, timeout=5).read().decode())

# ---- 时钟同步：Christian + P85 高分位（与网页同语义）----
def sync_once():
    t0 = time.time_ns() // 1000
    j = http_json("/api/sync")
    t1 = time.time_ns() // 1000
    rtt = t1 - t0
    return rtt, j["t_dev_us"] - (t0 + rtt // 2)

hist = []
t_end = time.time() + 10
while time.time() < t_end:
    rtt, off = sync_once()
    hist.append((time.time(), rtt, off))
    time.sleep(0.06 + (os.urandom(1)[0] % 40) / 1000)
hist.sort(key=lambda x: x[1])
min_rtt = hist[0][1]
offs = sorted(h[2] for h in hist)
offset = offs[int(len(offs) * 0.85) - 1] if len(offs) >= 12 else offs[len(offs)//2]
print(f"sync: samples={len(hist)} min_rtt={min_rtt/1000:.1f}ms offset={offset/1e6:.3f}s")

# ---- WS 收帧 ----
key = base64.b64encode(os.urandom(16)).decode()
s = socket.create_connection((host, 81), timeout=5)
req = (f"GET /ws HTTP/1.1\r\nHost: {host}:81\r\nUpgrade: websocket\r\n"
       f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
s.sendall(req.encode())
resp = b""
while b"\r\n\r\n" not in resp:
    resp += s.recv(4096)
buf = bytearray(resp.split(b"\r\n\r\n", 1)[1])

def recv_more(n):
    global buf
    while len(buf) < n:
        d = s.recv(65536)
        if not d:
            raise EOFError
        buf += d

frames = []
t0 = time.time()
while time.time() - t0 < secs:
    recv_more(2)
    opcode = buf[0] & 0x0F
    ln = buf[1] & 0x7F
    idx = 2
    if ln == 126:
        recv_more(4); ln = struct.unpack(">H", buf[2:4])[0]; idx = 4
    elif ln == 127:
        recv_more(10); ln = struct.unpack(">Q", buf[2:10])[0]; idx = 10
    recv_more(idx + ln)
    payload = bytes(buf[idx:idx+ln])
    del buf[:idx+ln]
    if opcode != 2 or len(payload) < 36:
        continue
    fid, cap, enc, w, h, q, src = struct.unpack("<IQQHHBB", payload[4:30])
    plen, keyf = struct.unpack("<IB", payload[30:35])
    arrive = time.time_ns() // 1000
    frames.append({"fid": fid, "cap": cap, "enc": enc, "w": w, "h": h,
                   "len": plen, "key": keyf, "arrive": arrive})
s.close()

if len(frames) < 5:
    print("no frames"); sys.exit(1)
dur = (frames[-1]["arrive"] - frames[0]["arrive"]) / 1e6
fps = len(frames) / dur
lat = [(f["arrive"] - f["cap"] + offset) / 1000 for f in frames]     # e2e(到达时刻, ms)
proc = [(f["enc"] - f["cap"]) / 1000 for f in frames]
lat.sort()
def pct(a, p): return a[min(len(a)-1, int(p*len(a)))]
import statistics
print(f"stream: {len(frames)} frames {dur:.1f}s fps={fps:.2f} res={frames[0]['w']}x{frames[0]['h']} avg={sum(f['len'] for f in frames)/len(frames):.0f}B")
print(f"e2e(到达): mean={statistics.mean(lat):.1f}ms p50={pct(lat,0.5):.1f} p95={pct(lat,0.95):.1f} max={lat[-1]:.1f} std={statistics.stdev(lat):.1f}")
print(f"proc(enc-cap): mean={statistics.mean(proc):.1f}ms")
neg = sum(1 for x in lat if x < 0)
print(f"negative: {neg}/{len(lat)}")
