#!/usr/bin/env python3
"""Дрожь стоя по логу телеметрии: разброс угла, гироскопа и ШИМ.

    python tools/standstat.py лог [с_какой_секунды]

Берёт только строки, где робот взведён (A) и стоит (без клавиш, |v| мал).
Меньше — спокойнее. Лог: вывод talk.py --out или tools/logs/wasd-*.log.
"""
import re
import statistics as st
import sys

TELE = re.compile(r'^\s*([\d.]+)\s+(?:\S+\s+)?A a=(-?[\d.]+) r=(-?\d+) v=(-?[\d.]+).*?uL=(-?\d+) uR=(-?\d+)')

t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
a, r, u = [], [], []
for line in open(sys.argv[1], encoding='utf-8', errors='replace'):
    m = TELE.match(line)
    if not m or float(m[1]) < t0 or abs(float(m[4])) > 3:
        continue
    a.append(float(m[2]))
    r.append(int(m[3]))
    u.append((int(m[5]) + int(m[6])) / 2)
if len(a) < 20:
    sys.exit(f'мало строк стойки: {len(a)}')
print(f'строк {len(a)} | угол СКО {st.pstdev(a):.2f}° | гироскоп СКО {st.pstdev(r):.1f} °/с | '
      f'ШИМ СКО {st.pstdev(u):.1f} | средний угол {st.mean(a):+.2f}°')
