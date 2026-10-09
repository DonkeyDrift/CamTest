#!/usr/bin/env python3
"""tab5_secure_proxy.py — localhost 成对端口转发（零证书摩擦的 WebCodecs 路径）

http://localhost 本身就是 Secure Context（W3C 豁免），转发后页面里
VideoDecoder 可用——H.264 硬解低延迟路径无需信任任何证书。

用法:
  python3 tab5_secure_proxy.py [设备IP=192.168.3.44]
然后浏览器打开  http://localhost:8080/

端口约定（页面 streamPort() 依赖）:
  本地 8080 → 设备 :80  （页面 + 管理 API）
  本地 8081 → 设备 :81  （WS / MJPEG 流，明文）
"""
import socket
import threading
import sys
import select

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.3.44"
PAIRS = [(8080, 80), (8081, 81)]
BUF = 65536


def pipe(a: socket.socket, b: socket.socket):
    try:
        while True:
            data = a.recv(BUF)
            if not data:
                break
            b.sendall(data)
    except OSError:
        pass
    finally:
        try:
            a.shutdown(socket.SHUT_RD)
            b.shutdown(socket.SHUT_WR)
        except OSError:
            pass


def serve(local_port: int, remote_port: int):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", local_port))
    srv.listen(8)
    print(f"localhost:{local_port}  →  {HOST}:{remote_port}")
    while True:
        c, _ = srv.accept()
        try:
            r = socket.create_connection((HOST, remote_port), timeout=5)
        except OSError:
            c.close()
            continue
        threading.Thread(target=pipe, args=(c, r), daemon=True).start()
        threading.Thread(target=pipe, args=(r, c), daemon=True).start()


def main():
    print(f"Tab5 secure-context 代理 → {HOST}")
    print("浏览器打开  http://localhost:8080/  （WebCodecs 可用，无需证书）\n")
    threads = []
    for lp, rp in PAIRS:
        t = threading.Thread(target=serve, args=(lp, rp), daemon=True)
        t.start()
        threads.append(t)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
