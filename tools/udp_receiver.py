#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
udp_receiver.py — CamTest UDP 分片图传接收端（探索时延下限用，非浏览器方案）

背景：浏览器（JS）无法直接收 UDP，因此 UDP 模式用本脚本接收。
设备端把每帧 JPEG 切成 ≤1200 字节的分片推送（可能花屏：无重传、无 ARQ）。

协议（小端）：
  客户端先向 设备IP:9100 发送一行 "SUBSCRIBE\n"（socket 复用为接收口）；
  停止发送 "UNSUBSCRIBE\n" 或 Ctrl-C 退出。
  设备每个数据报 24 字节头 + 分片负载：
    0  : magic u32 = 0x47504A55 ('UJPG')
    4  : frame_id  u32
    8  : t_capture_us u64（设备 esp_timer）
    16 : frag_idx  u16
    18 : frag_total u16
    20 : frag_len  u16
    22 : flags     u8  （bit0=首片, bit1=末片）
    23 : 保留      u8

用法：
    python3 udp_receiver.py --host korvo-s31.local --duration 20 --csv udp.csv
    python3 udp_receiver.py --host korvo-s31.local --scan   # 扫描模式：常驻并每秒 POST /api/scan/report

输出：每 5 s 一次实时统计；结束打印汇总（fps / 时延 / 丢片率 / 码率）。
时延计算先通过 HTTP /api/sync 做时钟同步（同 pi_compare.py）。
"""
import argparse
import json
import socket
import statistics
import struct
import sys
import time

MAGIC = 0x47504A55
HDR = struct.Struct("<IIQHHHBxxx")  # 24 字节

import requests  # 仅用于时钟同步


def sync_clock(host, http_port, n=8):
    probes = []
    url = f"http://{host}:{http_port}/api/sync"
    for _ in range(n):
        t0 = time.time() * 1e6
        r = requests.get(url, params={"t_client": int(t0)}, timeout=3)
        t1 = time.time() * 1e6
        j = r.json()
        if j.get("t_dev_us") is None:
            continue
        rtt = t1 - t0
        probes.append((rtt, j["t_dev_us"] - (t0 + rtt / 2)))
        time.sleep(0.05)
    if not probes:
        raise RuntimeError("clock sync failed")
    probes.sort()
    return sorted(p[1] for p in probes[:3])[1]


def scan_loop(sock, ip, args):
    """扫描模式：每 1 s 一个统计窗，回传 /api/scan/report（含 udp_loss_rate/udp_incomplete_rate）。
    扫描行切换由设备侧状态机决定；本脚本只按窗口回传，Ctrl-C 退出。"""
    import requests as rq
    sock.settimeout(0.2)
    sock.sendto(b"SUBSCRIBE\n", (ip, args.udp_port))
    print("scan 模式：回传 /api/scan/report（Ctrl-C 退出）", file=sys.stderr)
    frames, win_done, win_lost = {}, 0, 0
    win_lat = []
    t_win = time.time()
    last_sub = time.time()
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                if time.time() - last_sub > 1.0:
                    sock.sendto(b"SUBSCRIBE\n", (ip, args.udp_port))
                    last_sub = time.time()
            else:
                if len(data) >= HDR.size:
                    magic, fid, cap, idx, total, flen, flags = HDR.unpack_from(data)
                    if magic == MAGIC:
                        fr = frames.setdefault(fid, {"total": total, "parts": set(), "cap": cap,
                                                     "t_last_frag": time.time() * 1e6})
                        fr["parts"].add(idx)
                        if flags & 0x2 and len(fr["parts"]) == total:
                            win_done += 1
                            win_lat.append((time.time() * 1e6 - fr["cap"] + offset) / 1000.0)
                            frames.pop(fid, None)
            if time.time() - t_win >= 1.0:
                # 窗口结算：超龄未完成帧计入 incomplete
                stale = [k for k, fr in frames.items() if len(fr["parts"]) < fr["total"]]
                incomplete = len(stale)
                for k in stale:
                    frames.pop(k)
                    win_lost += 1
                total_frames = win_done + incomplete
                body = {
                    "arrival_fps": win_done,
                    "bitrate_mbps": 0,   # 码率由设备侧统计，回传仅作参考
                    "udp_loss_rate": round(win_lost / total_frames * 100, 2) if total_frames else 0,
                    "udp_incomplete_rate": round(incomplete / total_frames * 100, 2) if total_frames else 0,
                }
                if len(win_lat) >= 2:
                    body.update({
                        "latency_mean_ms": round(statistics.mean(win_lat), 2),
                        "latency_p95_ms": round(sorted(win_lat)[min(len(win_lat) - 1, int(0.95 * len(win_lat)))], 2),
                        "latency_max_ms": round(max(win_lat), 2),
                        "latency_std_ms": round(statistics.stdev(win_lat), 2),
                    })
                win_lat = []
                if total_frames:
                    try:
                        rq.post(f"http://{ip}:{args.http_port}/api/scan/report",
                                json=body, timeout=2)
                    except Exception:
                        pass
                win_done, win_lost = 0, 0
                t_win = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        sock.sendto(b"UNSUBSCRIBE\n", (ip, args.udp_port))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="korvo-s31.local", help="设备 IP 或 mDNS 名")
    ap.add_argument("--http-port", type=int, default=80)
    ap.add_argument("--udp-port", type=int, default=9100, help="设备侧 UDP 服务端口")
    ap.add_argument("--local-port", type=int, default=9100, help="本地接收端口（需与设备回推一致，默认同端口）")
    ap.add_argument("--duration", type=int, default=20)
    ap.add_argument("--scan", action="store_true",
                    help="扫描模式：常驻接收，每秒把到达率/时延/丢包率回传 /api/scan/report（UDP 扫描行数据源）")
    ap.add_argument("--csv")
    args = ap.parse_args()

    # 解析主机（支持 mDNS）
    ip = args.host
    try:
        ip = socket.gethostbyname(args.host)
    except OSError:
        pass

    offset = sync_clock(ip, args.http_port)
    print(f"clock offset = {offset/1000:.2f} ms", file=sys.stderr)

    if args.scan:
        scan_loop(sock, ip, args)
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind(("", args.local_port))
    sock.settimeout(1.0)
    sock.sendto(b"SUBSCRIBE\n", (ip, args.udp_port))
    print(f"SUBSCRIBE -> {ip}:{args.udp_port}，接收于 :{args.local_port}", file=sys.stderr)

    frames = {}     # fid -> {total, parts, cap, t_first}
    done = []       # (fid, arrival_us, cap_us, bytes, frags_ok)
    lost = 0
    t_end = time.time() + args.duration
    last_rep = time.time()
    try:
        while time.time() < t_end:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                sock.sendto(b"SUBSCRIBE\n", (ip, args.udp_port))
                continue
            if len(data) < HDR.size:
                continue
            magic, fid, cap, idx, total, flen, flags = HDR.unpack_from(data)
            if magic != MAGIC:
                continue
            fr = frames.setdefault(fid, {"total": total, "parts": {}, "cap": cap,
                                         "t_first": time.time() * 1e6})
            fr["parts"][idx] = data[HDR.size:HDR.size + flen]
            ok_last = flags & 0x2
            if ok_last and len(fr["parts"]) == total:
                done.append((fid, time.time() * 1e6, cap,
                             sum(len(p) for p in fr["parts"].values()), total))
                frames.pop(fid)
            elif len(frames) > 64:  # 防泄漏：太老的未完成帧丢弃
                for k in sorted(frames)[:32]:
                    frames.pop(k)
                    lost += 1
            if time.time() - last_rep >= 5:
                recent = [d for d in done if d[1] >= (time.time() - 5) * 1e6]
                print(f"  ... {args.duration - (t_end - time.time()):5.1f}s "
                      f"fps={len(recent)/5:.1f} complete={len(done)} lost={lost}", file=sys.stderr)
                last_rep = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        sock.sendto(b"UNSUBSCRIBE\n", (ip, args.udp_port))
        sock.close()

    if len(done) < 3:
        print(json.dumps({"ok": False, "error": "收到的完整帧太少", "frames": len(done)}))
        sys.exit(1)
    # offset = 设备钟 − 客户端钟；e2e = arrive − cap + offset（推导见 README）
    lat = [(d[1] - d[2] + offset) / 1000.0 for d in done]
    span = (done[-1][1] - done[0][1]) / 1e6
    fps = (len(done) - 1) / span if span > 0 else 0
    bits = sum(d[3] for d in done) * 8
    p95 = sorted(lat)[min(len(lat) - 1, int(0.95 * len(lat)))]
    out = {
        "transport": "udp", "frames": len(done), "fps": round(fps, 2),
        "bitrate_mbps": round(bits / span / 1e6, 3) if span else 0,
        "latency_mean_ms": round(statistics.mean(lat), 2),
        "latency_p95_ms": round(p95, 2),
        "latency_max_ms": round(max(lat), 2),
        "latency_std_ms": round(statistics.stdev(lat), 2) if len(lat) > 1 else 0,
        "incomplete_frames_dropped": lost,
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    if args.csv:
        import os
        new = not os.path.exists(args.csv)
        with open(args.csv, "a") as f:
            if new:
                f.write("timestamp,transport,frames,fps,bitrate_mbps,latency_mean_ms,"
                        "latency_p95_ms,latency_max_ms,latency_std_ms,lost\n")
            f.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')},udp,{out['frames']},{out['fps']},"
                    f"{out['bitrate_mbps']},{out['latency_mean_ms']},{out['latency_p95_ms']},"
                    f"{out['latency_max_ms']},{out['latency_std_ms']},{lost}\n")


if __name__ == "__main__":
    main()
