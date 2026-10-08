#!/usr/bin/env python3
"""tab5_uart.py — Tab5 (USB-Serial-JTAG) 串口日志抓取。
用法: tab5_uart.py <port> <秒数> [outfile]"""
import sys, time
import serial

port, secs = sys.argv[1], float(sys.argv[2])
out = open(sys.argv[3], "w", buffering=1) if len(sys.argv) > 3 else None
ser = serial.Serial()
ser.port = port
ser.baudrate = 115200
ser.dtr = False
ser.rts = False
ser.timeout = 0.2
ser.open()
t0 = time.time()
try:
    while time.time() - t0 < secs:
        data = ser.read(4096)
        if data:
            sys.stdout.write(data.decode("utf-8", "replace"))
            sys.stdout.flush()
            if out:
                out.write(data.decode("utf-8", "replace"))
finally:
    ser.close()
    if out:
        out.close()
