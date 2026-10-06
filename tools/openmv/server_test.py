# Поэтапная проверка стримера: Wi-Fi -> камера -> сервер. Печатает этапы,
# чтобы было видно, где падает. Клиентов ждёт кусками по 1 с (accept с таймаутом).
import socket
import time

import csi
import network
from machine import LED

import wifi_config

G = LED("LED_GREEN")
print("A. Wi-Fi...")
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(wifi_config.SSID, wifi_config.PASSWORD)
t0 = time.ticks_ms()
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), t0) > 20000:
        raise SystemExit("сеть не найдена")
    time.sleep_ms(200)
IP = wlan.ifconfig()[0]
print("A. ok, IP:", IP)

print("B. камера...")
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
for i in range(20):
    img = csi0.snapshot()
print("B. ok, 20 кадров при включённом Wi-Fi")

print("C. сервер...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 80))
s.listen(1)
s.settimeout(1.0)
print("C. ok, открой http://%s/  (тест 60 с)" % IP)

for sec in range(60):
    G.toggle()
    csi0.snapshot()                    # IDE видит живую картинку
    try:
        conn, addr = s.accept()
    except OSError:
        print("D. жду клиента", sec)
        continue
    print("D. клиент", addr)
    try:
        conn.settimeout(2.0)
        conn.recv(512)
        conn.send(b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOpenMV server_test OK\r\n")
        print("D. ответ отправлен")
    except OSError as e:
        print("D. ошибка:", e)
    finally:
        conn.close()
print("E. тест окончен")
