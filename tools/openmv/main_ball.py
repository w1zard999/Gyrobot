# Камера гиробота: ищет красный мяч и сообщает роботу по UART; видео — по Wi-Fi.
#
# Роботу (UART3, вывод P4 -> D12 Nano, 19200 бод) 10 раз в секунду уходит короткая строка:
#   b<x>,<размер>   мяч виден: x — смещение центра от середины кадра, пикс (+ вправо),
#                   размер — большая сторона рамки, пикс (кадр 320x240)
#   n               мяча нет
# Поиск идёт всегда — со зрителем и без, с Wi-Fi и без него.
#
# Страницы (http://<IP>/...):
#   /            видео с рамкой на мяче          /status   состояние
#   /color       цвет в центре кадра (LAB) и текущий порог — поставь мяч в центр
#   /thr?v=Lmin,Lmax,Amin,Amax,Bmin,Bmax         задать порог и запомнить (ball_thr.txt)
# Клиент у Wi-Fi-модуля один: пока кто-то смотрит видео, страницы не открываются.
# Сеть и пароль — в wifi_config.py на камере (образец: wifi_config_example.py).
# Светодиод: синий — сети нет (мяч всё равно ищется), зелёный — сеть есть, красный — сбой.
import gc
import socket
import time

import csi
import machine
import network
from machine import LED, UART

import wifi_config

W, H = 320, 240
THR_FILE = "ball_thr.txt"
THR = [10, 70, 25, 80, -5, 50]         # красный мяч, LAB; подобрано по кадру 2026-10-08:
                                       # мяч A 34..45, стол/стена/тень — A не выше 21
SEND_MS = 100                          # роботу — 10 раз/с: каждый байт стоит ему 0.5 мс

R, G, B = LED("LED_RED"), LED("LED_GREEN"), LED("LED_BLUE")


def leds(r=0, g=0, b=0):
    (R.on if r else R.off)(); (G.on if g else G.off)(); (B.on if b else B.off)()


def load_thr():
    try:
        with open(THR_FILE) as f:
            v = [int(x) for x in f.read().split(",")]
        if len(v) == 6:
            THR[:] = v
    except (OSError, ValueError):
        pass                           # файла нет или испорчен — порог из кода


load_thr()
try:                                   # причина прошлого сбоя — видна на /status без USB
    with open("last_error.txt") as f:
        LAST_ERROR = f.read().strip()
except OSError:
    LAST_ERROR = "-"
leds(b=1)
uart = UART(3, 19200)

# --- Камера ---
csi0 = csi.CSI()
csi0.reset()
csi0.pixformat(csi.RGB565)
csi0.framesize(csi.QVGA)
csi0.snapshot(time=2000)               # автоэкспозиция и баланс белого

# --- Wi-Fi: не подключился — работаем без видео ---
wlan = None
try:
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(wifi_config.SSID, wifi_config.PASSWORD)
except Exception as e:                 # шилда нет или он не отвечает
    print("WiFi-шилд не отвечает:", e)
    wlan = None

BOOT_MS = time.ticks_ms()
FRAMES = 0
FPS = 0.0
CLIENT = "-"
IP = "-"
server = None                          # слушающий сокет, когда сеть поднялась
viewer = None                          # кто смотрит видео (один)
ball = None                            # (x от центра, размер) последнего кадра


def find_ball(img):
    """Самое крупное пятно цвета мяча. Поля пятна — по номерам: в разных прошивках
    pixels/rect то методы, то значения. [0..3] рамка, [4] пикселей, [5] центр x."""
    best = None
    for b in img.find_blobs([tuple(THR)], pixels_threshold=40, area_threshold=40, merge=True):
        if best is None or b[4] > best[4]:
            best = b
    return best


def send_all(conn, data):
    """Wi-Fi-модуль за раз отправляет только часть — досылать, пока не уйдёт всё.
    Иначе кадр обрезается и браузер замирает на последнем целом кадре."""
    mv = memoryview(data)
    while mv:
        n = conn.send(mv)
        if not n:
            raise OSError("send вернул 0")
        mv = mv[n:]


def close(conn):
    try:
        conn.close()
    except Exception:
        pass


def sock_fd(conn):
    """Номер соединения в Wi-Fi-модуле; другого способа узнать его нет — из описания
    вида <socket fd=1 timeout=...>."""
    try:
        return str(conn).split("fd=")[1].split()[0]
    except IndexError:
        return None


def http_ok(body, ctype=b"text/html; charset=utf-8"):
    return b"HTTP/1.1 200 OK\r\nContent-Type: " + ctype + b"\r\n\r\n" + body


PAGE = http_ok(b"<!DOCTYPE html><html><head><title>OpenMV Cam</title></head>"
               b"<body style='text-align:center'><h2>OpenMV Cam</h2>"
               b"<img src='/stream.jpg' style='max-width:100%' "
               b"onerror=\"setTimeout(()=>this.src='/stream.jpg?'+Date.now(),1000)\">"
               b"</body></html>")                # поток оборвался — переподключиться

HDR = (b"HTTP/1.1 200 OK\r\n"
       b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n")


def ball_text():
    return "x=%d size=%d" % ball if ball else "not seen"


def status_page():
    up = time.ticks_diff(time.ticks_ms(), BOOT_MS) // 1000
    html = ("<!DOCTYPE html><html><head><title>OpenMV status</title>"
            "<meta http-equiv='refresh' content='2'></head>"
            "<body style='font-family:monospace;text-align:center'>"
            "<h2>OpenMV Cam - Status</h2><p>IP: %s</p><p>Uptime: %02d:%02d:%02d</p>"
            "<p>Frames: %d (%.0f fps)</p><p>Ball: %s</p><p>Last client: %s</p>"
            "<p>Last error: %s</p><p><a href='/'>Open stream</a></p></body></html>"
            ) % (IP, up // 3600, (up // 60) % 60, up % 60, FRAMES, FPS, ball_text(), CLIENT,
                 LAST_ERROR)
    return http_ok(html.encode())


def color_page():
    """Цвет квадрата 40x40 в центре кадра."""
    st = csi0.snapshot().get_statistics(roi=(W // 2 - 20, H // 2 - 20, 40, 40))

    def val(name):                     # в разных прошивках поля — то методы, то значения
        v = getattr(st, name)
        return v() if callable(v) else v

    out = "center 40x40, LAB\n"
    for c in "lab":
        out += "%s: min %d  max %d  quartiles %d..%d  mean %d\n" % (
            c.upper(), val(c + "_min"), val(c + "_max"), val(c + "_lq"), val(c + "_uq"),
            val(c + "_mean"))
    out += "threshold: %s\nball: %s\nfps: %.0f\n" % (
        ",".join(str(v) for v in THR), ball_text(), FPS)
    return http_ok(out.encode(), b"text/plain; charset=utf-8")


def set_thr(path):
    try:
        v = [int(x) for x in path.split(b"v=")[1].split(b"&")[0].split(b",")]
        if len(v) != 6:
            raise ValueError
    except (IndexError, ValueError):
        return http_ok(b"need /thr?v=Lmin,Lmax,Amin,Amax,Bmin,Bmax\n", b"text/plain")
    THR[:] = v
    text = ",".join(str(x) for x in v)
    try:
        with open(THR_FILE, "w") as f:
            f.write(text)
        saved = "saved"
    except OSError as e:
        saved = "NOT saved: %s" % e
    return http_ok(("threshold %s, %s\n" % (text, saved)).encode(), b"text/plain")


def poll_clients():
    """Принять одного клиента, если стучится. Запрос потока — он становится зрителем
    (прежний отключается: браузеры любят держать мёртвые соединения)."""
    global viewer, CLIENT
    try:
        conn, addr = server.accept()
    except OSError:                    # никого
        return
    CLIENT = str(addr[0])
    # Wi-Fi-модуль отдаёт новому клиенту номер соединения ушедшего зрителя, а отправка
    # в закрытое зрителем соединение ошибки не даёт — видео уходило бы новому клиенту
    if viewer and sock_fd(viewer) == sock_fd(conn):
        viewer = None                  # не закрывать: номер уже принадлежит новому
    try:
        conn.settimeout(2.0)           # молчащий клиент не останавливает поиск мяча надолго
        req = conn.recv(1024)
        parts = req.split(b" ")
        path = parts[1] if len(parts) > 1 else b"/"
        if path.startswith(b"/stream"):
            send_all(conn, HDR)
            if viewer:
                close(viewer)
            viewer = conn
            return
        if path.startswith(b"/status"):
            send_all(conn, status_page())
        elif path.startswith(b"/color"):
            send_all(conn, color_page())
        elif path.startswith(b"/thr"):
            send_all(conn, set_thr(path))
        else:
            send_all(conn, PAGE)
    except OSError as e:
        print("Клиент отключился:", e)
    close(conn)


def send_frame(img):
    jpeg = bytes(img.compress(quality=60).bytearray())   # 85 — кадры крупнее, чаще рвётся
    send_all(viewer, b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
             + str(len(jpeg)).encode() + b"\r\n\r\n")
    send_all(viewer, jpeg)
    send_all(viewer, b"\r\n")


def die(why):
    """Неожиданный сбой: записать причину и перезагрузить камеру — через несколько
    секунд она снова ищет мяч и выходит в сеть."""
    print("СБОЙ:", why)
    try:
        with open("last_error.txt", "w") as f:
            f.write("uptime %d s, frames %d: %s\n"
                    % (time.ticks_diff(time.ticks_ms(), BOOT_MS) // 1000, FRAMES, why))
    except Exception:
        pass
    leds(r=1)
    time.sleep_ms(500)
    machine.reset()


def net_check():
    """Раз в секунду: сеть поднялась — открыть сервер; пропала — перезагрузка."""
    global server, IP
    if wlan is None:
        return
    up = wlan.isconnected()
    if server is None and up:
        IP = wlan.ifconfig()[0]
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", 80))
        server.listen(1)
        server.settimeout(0.01)        # заглянуть, не стучится ли кто, и дальше искать мяч
        print("WiFi OK: http://%s/" % IP)
        leds(g=1)
    elif server is not None and not up:
        die("Wi-Fi пропал")


def main():
    global FRAMES, FPS, viewer, ball
    last_tx = last_net = fps_t = time.ticks_ms()
    fps_n = 0
    while True:
        img = csi0.snapshot()
        b = find_ball(img)
        ball = (b[5] - W // 2, max(b[2], b[3])) if b else None
        now = time.ticks_ms()
        if time.ticks_diff(now, last_tx) >= SEND_MS:
            last_tx = now
            uart.write("b%d,%d\n" % ball if ball else "n\n")
        if time.ticks_diff(now, last_net) >= 1000:
            last_net = now
            net_check()
        FRAMES += 1
        fps_n += 1
        if time.ticks_diff(now, fps_t) >= 2000:
            FPS = fps_n * 1000 / time.ticks_diff(now, fps_t)
            fps_t, fps_n = now, 0
        if viewer:
            if b:
                # координаты — одним набором: по отдельности эта прошивка не принимает
                img.draw_rectangle((b[0], b[1], b[2], b[3]), color=(0, 255, 0), thickness=2)
                img.draw_cross((b[5], b[6]), color=(0, 255, 0))
            try:
                send_frame(img)
            except OSError as e:       # зритель ушёл или замолчал — обычное дело
                print("Зритель отключился:", e)
                close(viewer)
                viewer = None
        # Пока есть зритель, второго клиента Wi-Fi-модуль не принимает (проверено:
        # страницы ждут и не открываются) — новый войдёт, когда зритель уйдёт
        if server:
            poll_clients()
        if FRAMES % 20 == 0:
            gc.collect()


try:
    main()
except Exception as e:                 # сенсор, память, что угодно — перезапуск
    die("%s: %s" % (type(e).__name__, e))
