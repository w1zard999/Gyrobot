#!/usr/bin/env python3
"""Разговор с гироботом: открыть порт, подождать бут, послать команды, слушать."""
import argparse
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='COM9')
ap.add_argument('--boot', type=float, default=1.8, help='пауза после открытия (бут платы)')
ap.add_argument('--listen', type=float, default=3.0, help='сколько слушать после команд')
ap.add_argument('--cmds', default='', help='команды через ;')
ap.add_argument('--gap', type=float, default=0.15, help='пауза между командами (сек)')
ap.add_argument('--out', default='', help='дописать вывод в файл')
a = ap.parse_args()

ser = serial.Serial(a.port, 115200, timeout=0.05)
t0 = time.time()
time.sleep(a.boot)
ser.reset_input_buffer()
for cmd in [c.strip() for c in a.cmds.split(';') if c.strip()]:
    ser.write((cmd + '\n').encode())
    ser.flush()
    time.sleep(a.gap)
end = time.time() + a.listen
lines = []
while time.time() < end:
    data = ser.readline()
    if data:
        line = f"{time.time()-t0:7.2f}  " + data.decode('utf-8', 'replace').rstrip()
        try:
            print(line)
        except UnicodeEncodeError:
            print(line.encode('ascii', 'replace').decode())
        lines.append(line)
ser.close()
if a.out:
    with open(a.out, 'a', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
