# Видеопоток с OpenMV (WiFi-шилд) в браузер: http://<IP>/ , статус: /status
# Сеть и пароль — в wifi_config.py на камере (образец: wifi_config_example.py).
# Светодиод: синий мигает — ищет сеть, зелёный — поток работает, красный — ошибка.
# Если сеть не нашлась за 20 с — сдаётся и отпускает камеру (можно подключить IDE).
import socket
import time

import csi
import network
from machine import LED

import wifi_config

try:
    import hand_detect                 # обводка рук рамкой, если файл есть на камере
except ImportError:
    hand_detect = None

R, G, B = LED("LED_RED"), LED("LED_GREEN"), LED("LED_BLUE")


def leds(r=0, g=0, b=0):
    (R.on if r else R.off)(); (G.on if g else G.off)(); (B.on if b else B.off)()


def fail(msg):
    print("ОШИБКА:", msg)
    for _ in range(10):                # 5 с мигать красным и выйти — IDE сможет подключиться
        leds(r=1); time.sleep_ms(250); leds(); time.sleep_ms(250)
    raise SystemExit


# --- Wi-Fi ---
print("Wi-Fi: подключаюсь к", wifi_config.SSID)
try:
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(wifi_config.SSID, wifi_config.PASSWORD)
except Exception as e:                 # шилда нет или он не отвечает
    fail("WiFi-шилд не отвечает: %s" % e)
t0 = time.ticks_ms()
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), t0) > 20000:
        fail("сеть %s не найдена или неверный пароль" % wifi_config.SSID)
    leds(b=1); time.sleep_ms(150); leds(); time.sleep_ms(350)
IP = wlan.ifconfig()[0]
print("WiFi OK, IP:", IP)
print("Поток: http://%s/   статус: http://%s/status" % (IP, IP))
leds(g=1)

# --- Камера ---
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
csi0.snapshot(time=2000)               # автоэкспозиция и баланс белого

BOOT_MS = time.ticks_ms()
FRAMES = 0
CLIENT = "-"


def send_all(conn, data):
    """Wi-Fi-модуль за раз отправляет только часть — досылать, пока не уйдёт всё.
    Иначе кадр обрезается и браузер замирает на последнем целом кадре."""
    mv = memoryview(data)
    while mv:
        n = conn.send(mv)
        if not n:
            raise OSError("send вернул 0")
        mv = mv[n:]


def http_ok(body, ctype=b"text/html; charset=utf-8"):
    return b"HTTP/1.1 200 OK\r\nContent-Type: " + ctype + b"\r\n\r\n" + body


PAGE = http_ok(b"<!DOCTYPE html><html><head><title>OpenMV Cam</title></head>"
               b"<body style='text-align:center'><h2>OpenMV Cam</h2>"
               b"<img src='/stream.jpg' style='max-width:100%' "
               b"onerror=\"setTimeout(()=>this.src='/stream.jpg?'+Date.now(),1000)\">"
               b"</body></html>")                # поток оборвался — переподключиться


def status_page():
    up = time.ticks_diff(time.ticks_ms(), BOOT_MS) // 1000
    html = ("<!DOCTYPE html><html><head><title>OpenMV status</title>"
            "<meta http-equiv='refresh' content='2'></head>"
            "<body style='font-family:monospace;text-align:center'>"
            "<h2>OpenMV Cam - Status</h2><p>IP: %s</p><p>Uptime: %02d:%02d:%02d</p>"
            "<p>Frames sent: %d</p><p>Last client: %s</p>"
            "<p><a href='/'>Open stream</a></p></body></html>"
            ) % (IP, up // 3600, (up // 60) % 60, up % 60, FRAMES, CLIENT)
    return http_ok(html.encode())


HDR = (b"HTTP/1.1 200 OK\r\n"
       b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 80))
s.listen(1)
s.settimeout(1.0)                      # ждать клиентов кусками по 1 с, а не вечно

while True:
    try:
        conn, addr = s.accept()
    except OSError:                    # за секунду никого — ждём дальше
        continue
    CLIENT = str(addr[0])
    print("Клиент:", addr)
    try:
        # Без таймаута один молчащий клиент вешал сервер навсегда: браузеры открывают
        # «запасные» соединения и ничего не шлют; пропавший клиент блокировал отправку
        conn.settimeout(3.0)
        req = conn.recv(1024)
        if b"/stream" in req:
            send_all(conn, HDR)
            while True:
                img = csi0.snapshot()
                if hand_detect:
                    hand_detect.draw(img, hand_detect.detect(img))
                jpeg = bytes(img.compress(quality=60).bytearray())   # 85 — кадры крупнее, чаще рвётся
                send_all(conn, b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                         + str(len(jpeg)).encode() + b"\r\n\r\n")
                send_all(conn, jpeg)
                send_all(conn, b"\r\n")
                FRAMES += 1
        elif b"/status" in req:
            send_all(conn, status_page())
        else:
            send_all(conn, PAGE)
    except OSError as e:
        print("Клиент отключился:", e)
    finally:
        conn.close()
