#!/usr/bin/env python3
"""tab5_ws_probe.py — Tab5 WS v2 协议探针：收 H264/JPEG 帧、校验头、存 Annex-B 裸流。
用法: tab5_ws_probe.py <host> [秒数=10] [outfile=/tmp/tab5.h264] [port=81] [tls=0]
  tls=1 连 :8443（wss，自签证书不校验）"""
import sys, time, socket, base64, os, struct, ssl

host = sys.argv[1] if len(sys.argv) > 1 else "192.168.3.44"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
outpath = sys.argv[3] if len(sys.argv) > 3 else "/tmp/tab5.h264"
port = int(sys.argv[4]) if len(sys.argv) > 4 else 81
use_tls = len(sys.argv) > 5 and sys.argv[5] in ("1", "tls", "wss")

# ---- 极简 RFC6455 客户端（无掩码回显要求：客户端→服务器必须掩码）----
key = base64.b64encode(os.urandom(16)).decode()
s = socket.create_connection((host, port), timeout=5)
if use_tls:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    s = ctx.wrap_socket(s, server_hostname=host)
req = (f"GET /ws HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
       f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
s.sendall(req.encode())
resp = b""
while b"\r\n\r\n" not in resp:
    resp += s.recv(4096)
head, rest = resp.split(b"\r\n\r\n", 1)
assert b"101" in head.split(b"\r\n")[0], head
accept = head.decode().split("Sec-WebSocket-Accept:")[1].split("\r\n")[0].strip()
expect = base64.b64encode(__import__("hashlib").sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
assert accept == expect, "handshake accept mismatch"

buf = bytearray(rest)

def recv_more(n):
    global buf
    while len(buf) < n:
        d = s.recv(65536)
        if not d:
            raise EOFError
        buf += d

def read_message():
    """返回 (opcode, payload)；处理分片与掩码（服务器不掩码）"""
    recv_more(2)
    b0, b1 = buf[0], buf[1]
    opcode = b0 & 0x0F
    ln = b1 & 0x7F
    idx = 2
    if ln == 126:
        recv_more(4); ln = struct.unpack(">H", buf[2:4])[0]; idx = 4
    elif ln == 127:
        recv_more(10); ln = struct.unpack(">Q", buf[2:10])[0]; idx = 10
    recv_more(idx + ln)
    payload = bytes(buf[idx:idx+ln])
    del buf[:idx+ln]
    return opcode, payload

t0 = time.time()
frames = []
out = open(outpath, "wb")
h264_n = jpeg_n = 0
idr_n = 0
while time.time() - t0 < secs:
    try:
        op, payload = read_message()
    except (EOFError, socket.timeout):
        break
    if op != 2 or len(payload) < 36:
        continue
    magic = payload[0:4]
    fid, cap, enc, w, h, q, src = struct.unpack("<IQQHHBB", payload[4:30])
    ln, keyflag = struct.unpack("<IB", payload[30:35])
    data = payload[36:36+ln]
    if magic == b"AVC1":
        h264_n += 1
        idr_n += 1 if keyflag else 0
        out.write(data)
        frames.append((time.time(), fid, cap, enc, w, h, ln, keyflag))
    elif magic == b"MJP1":
        jpeg_n += 1
        frames.append((time.time(), fid, cap, enc, w, h, ln, 0))
out.close()
s.close()

print(f"frames={len(frames)} h264={h264_n} jpeg={jpeg_n} idr={idr_n}")
if len(frames) > 2:
    dur = frames[-1][0] - frames[0][0]
    print(f"duration={dur:.2f}s fps={len(frames)/dur:.2f}")
    caps = [f[2] for f in frames]
    encs = [f[3] for f in frames]
    lats_tx = [(f[0]*1e6 - f[2])/1000 for f in frames]  # 粗略(未做时钟同步)
    print(f"res={frames[0][4]}x{frames[0][5]} avg_size={sum(f[6] for f in frames)/len(frames):.0f}B")
    print(f"proc(enc-cap) mean={sum(e-c for e,c in zip(encs,caps))/len(encs)/1000:.2f}ms")
    print(f"saved={outpath} size={os.path.getsize(outpath)}")
    # Annex-B 结构校验：起始码 + NAL 类型统计
    raw = open(outpath, "rb").read()
    nals = raw.count(b"\x00\x00\x00\x01")
    print(f"annexb start_codes={nals}")
