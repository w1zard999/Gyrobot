# Проверка только Wi-Fi (без камеры и сервера): подключиться и 60 с писать «жив».
# Если камера падает и здесь — дело в Wi-Fi/шилде/прошивке; если нет — в сервере.
import time

import network
from machine import LED

import wifi_config

G = LED("LED_GREEN")
print("1. включаю Wi-Fi")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("2. подключаюсь к", wifi_config.SSID)
wlan.connect(wifi_config.SSID, wifi_config.PASSWORD)
t0 = time.ticks_ms()
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), t0) > 20000:
        raise SystemExit("сеть не найдена за 20 с")
    time.sleep_ms(200)
print("3. подключено, IP:", wlan.ifconfig()[0])
for i in range(120):
    G.toggle()
    print("4. жив", i, "rssi", wlan.status("rssi") if hasattr(wlan, "status") else "")
    time.sleep_ms(500)
print("5. тест окончен — Wi-Fi держится")
