#!/usr/bin/env python3
"""WASD-руление гироботом по BT COM: зажал — едет, отпустил — стоит. ESC — выход.

Раскладка клавиатуры должна быть EN: шлются латинские wasd.
Пример: python tools/wasd.py --port COM13
"""
import argparse
import os
import threading
import time

import serial
from pynput import keyboard

ap = argparse.ArgumentParser()
ap.add_argument('--port', required=True, help='исходящий BT COM робота (напр. COM13)')
ap.add_argument('--rate', type=float, default=10.0, help='частота посылки букв, Гц')
ap.add_argument('--log', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs'),
                help='папка для лога телеметрии робота (пусто — не писать)')
a = ap.parse_args()

held = set()
lock = threading.Lock()


VK = {87: 'w', 65: 'a', 83: 's', 68: 'd'}   # виртуальные коды: не зависят от раскладки


def ch_of(key):
    vk = getattr(key, 'vk', None)
    if vk in VK:
        return VK[vk]
    try:
        ch = key.char
    except AttributeError:
        return None
    ch = ch.lower() if isinstance(ch, str) else None
    return ch if ch in VK.values() else None


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

log = None
if a.log:
    os.makedirs(a.log, exist_ok=True)
    log_path = os.path.join(a.log, time.strftime('wasd-%Y%m%d-%H%M%S.log'))
    log = open(log_path, 'w', encoding='utf-8')
    print('лог телеметрии:', log_path)
t0 = time.time()


def reader():                       # телеметрия робота -> файл, с отметкой нажатых клавиш
    while log and not log.closed:
        try:
            data = ser.readline()
        except (serial.SerialException, TypeError):
            break
        if data:
            with lock:
                keys = ''.join(sorted(held)) or '.'
            line = data.decode('utf-8', 'replace').rstrip()
            log.write(f'{time.time() - t0:8.2f} {keys:4s} {line}\n')
            log.flush()


threading.Thread(target=reader, daemon=True).start()
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
    if log:
        log.close()
    ser.close()
    print('Стоп: буквы больше не шлются, робот затормозит по watchdog.')
