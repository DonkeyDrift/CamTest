#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pi_compare.py — CamTest 第三方对照测量脚本（树莓派 / 任意 PC，Python3 + requests）

用途：不依赖浏览器，独立拉取 /stream 并用与网页相同的时钟同步法测量端到端时延，
用于交叉验证 Web UI 测量值的可信度（P2）。

用法：
    python3 pi_compare.py --host korvo-s31.local --duration 30
    python3 pi_compare.py --host 192.168.1.123 --duration 60 --csv out.csv
    python3 pi_compare.py --host 192.168.1.123 --save-dir frames/   # 同时落盘 JPEG（离线分析）

依赖：pip3 install requests（树莓派上无 OpenCV 也能跑；如需显示画面再装 opencv-python）

测量口径与网页版一致：
  - 时钟同步：Christian 算法，8 次探测取 RTT 最小 3 次的 offset 中位数
  - 端到端时延 = 帧头解析完成时刻(客户端钟) − (X-Capture-Us + offset)
  - 注意：requests 路径经过 Python 用户态缓冲，到达时刻略晚于浏览器 fetch 可得值，
    属于"更保守"的估计；与浏览器值差值可视为浏览器/JS 侧开销。
"""
import argparse
import json
import math
import socket
import statistics
import sys
import time

import requests

BOUNDARY_HINT = "frame"


def sync_clock(host, port, n=8):
    """返回 (offset_us, min_rtt_us, wall_diff_ms)。offset: t_dev ≈ t_client + offset"""
    probes = []
    url = f"http://{host}:{port}/api/sync"
    wall_diff = None
    for _ in range(n):
        t0 = time.time() * 1e6  # 客户端钟 us（系统钟，若本机 NTP 同步则与 UTC 对齐）
        r = requests.get(url, params={"t_client": int(t0)}, timeout=3)
        t1 = time.time() * 1e6
        j = r.json()
        if j.get("t_dev_us") is None:
            continue
        rtt = t1 - t0
        probes.append((rtt, j["t_dev_us"] - (t0 + rtt / 2)))
        if j.get("t_wall_ms"):
            wall_diff = j["t_wall_ms"] - t1 / 1e3
        time.sleep(0.05)
    if not probes:
        raise RuntimeError("clock sync failed: /api/sync no response")
    probes.sort()
    best = sorted(p[1] for p in probes[:3])
    return best[1], probes[0][0], wall_diff


def parse_stream(host, stream_port, duration, save_dir=None):
    """流式解析 multipart，返回逐帧 (arrival_client_us, frame_id, capture_us, encode_us, length, quality, res, sensor)"""
    url = f"http://{host}:{stream_port}/stream"
    frames = []
    with requests.get(url, stream=True, timeout=(5, None)) as r:
        r.raise_for_status()
        boundary = None
        ct = r.headers.get("Content-Type", "")
        if "boundary=" in ct:
            boundary = ct.split("boundary=")[1].strip().strip('"').encode()
        buf = b""
        hdr_end = None
        meta = {}
        t_start = time.time()
        last_report = t_start
        for chunk in r.iter_content(chunk_size=4096):
            now = time.time()
            if now - last_report >= 5.0:
                print(f"  ... {now - t_start:5.1f}s, {len(frames)} frames", file=sys.stderr)
                last_report = now
            buf += chunk
            while True:
                if hdr_end is None:
                    # 找 boundary 行 + 头部结束
                    idx = buf.find(b"\r\n\r\n")
                    if idx < 0:
                        break
                    head = buf[:idx].decode("utf-8", "replace")
                    lines = [l for l in head.split("\r\n") if l and not l.startswith("--")]
                    meta = {}
                    for l in lines:
                        if ":" not in l:
                            continue
                        k, v = l.split(":", 1)
                        k = k.strip().lower()
                        v = v.strip()
                        if k == "content-length":
                            meta["len"] = int(v)
                        elif k == "x-frame-id":
                            meta["fid"] = int(v)
                        elif k == "x-capture-us":
                            meta["cap"] = int(v)
                        elif k == "x-encode-us":
                            meta["enc"] = int(v)
                        elif k == "x-jpeg-len":
                            meta["len"] = int(v)
                        elif k == "x-sensor":
                            meta["sensor"] = v
                        elif k == "x-res":
                            meta["res"] = v
                        elif k == "x-quality":
                            meta["q"] = int(v)
                    buf = buf[idx + 4:]
                    hdr_end = time.time() * 1e6  # 头解析完成 ≈ 帧首字节已到达
                else:
                    if len(buf) < meta.get("len", 0) + 2:
                        break
                    frames.append((hdr_end, meta.get("fid", 0), meta.get("cap", 0),
                                   meta.get("enc", 0), meta.get("len", 0),
                                   meta.get("q", 0), meta.get("res", ""),
                                   meta.get("sensor", "")))
                    if save_dir:
                        with open(f"{save_dir}/f{meta.get('fid', 0):08d}_q{meta.get('q', 0)}.jpg", "wb") as f:
                            f.write(buf[:meta["len"]])
                    buf = buf[meta["len"] + 2:]
                    hdr_end = None
                    meta = {}
            if time.time() - t_start > duration:
                break
    return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="korvo-s31.local")
    ap.add_argument("--port", type=int, default=80, help="管理 API 端口")
    ap.add_argument("--stream-port", type=int, default=81, help="MJPEG 流端口（与固件 stream_server 一致）")
    ap.add_argument("--duration", type=int, default=30, help="测量时长（秒），默认 30")
    ap.add_argument("--csv", help="追加一行汇总到该 CSV")
    ap.add_argument("--save-dir", help="保存逐帧 JPEG 到该目录")
    args = ap.parse_args()

    if args.save_dir:
        import os
        os.makedirs(args.save_dir, exist_ok=True)

    print(f"== 时钟同步 {args.host} ...", file=sys.stderr)
    offset, rtt, wall_diff = sync_clock(args.host, args.port)
    print(f"   offset = {offset/1000:.2f} ms（客户端 epoch 钟 − 设备开机钟，恒为大负值属正常，"
          f"相减时抵消）, min RTT = {rtt/1000:.2f} ms, "
          f"wall_diff = {wall_diff if wall_diff is None else round(wall_diff, 2)} ms", file=sys.stderr)

    print(f"== 拉流测量 {args.duration}s ...", file=sys.stderr)
    frames = parse_stream(args.host, args.stream_port, args.duration, args.save_dir)
    if len(frames) < 5:
        print("帧数太少，测量失败", file=sys.stderr)
        sys.exit(1)

    # offset = t_dev − (t0+rtt/2) = 设备钟 − 客户端钟；e2e = arrive − cap + offset（推导见 README）
    lat = [(f[0] - f[2] + offset) / 1000.0 for f in frames if f[2]]      # ms
    enc = [(f[3] - f[2]) / 1000.0 for f in frames if f[2] and f[3]]        # ms，设备侧精确
    span = (frames[-1][0] - frames[0][0]) / 1e6
    fps = (len(frames) - 1) / span if span > 0 else 0
    bits = sum(f[4] for f in frames) * 8
    p95 = sorted(lat)[min(len(lat) - 1, int(0.95 * len(lat)))]
    out = {
        "frames": len(frames), "fps": round(fps, 2),
        "bitrate_mbps": round(bits / span / 1e6, 3) if span else 0,
        "latency_mean_ms": round(statistics.mean(lat), 2),
        "latency_p95_ms": round(p95, 2),
        "latency_max_ms": round(max(lat), 2),
        "latency_std_ms": round(statistics.stdev(lat), 2) if len(lat) > 1 else 0,
        "encode_ms": round(statistics.mean(enc), 2) if enc else None,
        "res": frames[0][6], "quality": frames[0][5], "sensor": frames[0][7],
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))

    if args.csv:
        import os
        new = not os.path.exists(args.csv)
        with open(args.csv, "a") as f:
            if new:
                f.write("timestamp,sensor,resolution,quality,arrival_fps,bitrate_mbps,"
                        "latency_mean_ms,latency_p95_ms,latency_max_ms,latency_std_ms,encode_ms\n")
            f.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')},{out['sensor']},{out['res']},{out['quality']},"
                    f"{out['fps']},{out['bitrate_mbps']},{out['latency_mean_ms']},{out['latency_p95_ms']},"
                    f"{out['latency_max_ms']},{out['latency_std_ms']},{out['encode_ms']}\n")
        print(f"已追加到 {args.csv}", file=sys.stderr)


if __name__ == "__main__":
    main()
