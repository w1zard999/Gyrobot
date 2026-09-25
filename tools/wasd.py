#!/usr/bin/env python3
"""WASD-руление гироботом по BT COM: зажал — едет, отпустил — стоит. ESC — выход.

Раскладка клавиатуры должна быть EN: шлются латинские wasd.
Пример: python tools/wasd.py --port COM13
"""
import argparse
import threading
import time

import serial
from pynput import keyboard

ap = argparse.ArgumentParser()
ap.add_argument('--port', required=True, help='исходящий BT COM робота (напр. COM13)')
ap.add_argument('--rate', type=float, default=10.0, help='частота посылки букв, Гц')
a = ap.parse_args()

held = set()
lock = threading.Lock()


def ch_of(key):
    try:
        ch = key.char
    except AttributeError:
        return None
    return ch.lower() if isinstance(ch, str) else None


def on_press(key):
    ch = ch_of(key)
    if ch and ch in 'wasd':
        with lock:
            held.add(ch)
    elif key == keyboard.Key.esc:
        listener.stop()


def on_release(key):
    ch = ch_of(key)
    if ch and ch in 'wasd':
        with lock:
            held.discard(ch)


ser = serial.Serial(a.port, 115200, timeout=0.1)
print('WASD ->', a.port, '| ESC — выход')

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()
try:
    period = 1.0 / a.rate
    while listener.is_alive():
        with lock:
            keys = sorted(held)
        for ch in keys:
            ser.write((ch + '\n').encode())
        time.sleep(period)
finally:
    listener.stop()
    ser.close()
    print('Стоп: буквы больше не шлются, робот затормозит по watchdog.')
