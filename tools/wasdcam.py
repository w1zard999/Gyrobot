"""Видео с камеры OpenMV для клиента WASD: поиск камеры в сети, чтение MJPEG-потока.

Камера (tools/openmv/main_stream.py) отдаёт http://<ip>/stream.jpg — поток частей
"--frame ... Content-Length: N" + JPEG. Обслуживает одного зрителя за раз.
"""
import concurrent.futures
import os
import re
import socket
import threading
import time

PORT = 80
LENGTH = re.compile(rb"Content-Length:\s*(\d+)", re.I)


class FrameParser:
    """Собирает JPEG-кадры из кусков потока в том виде, как они приходят из сокета."""

    def __init__(self, max_frame=500_000):
        self.buf = b""
        self.max_frame = max_frame

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(b"--frame")
            if start < 0:
                self.buf = self.buf[-8:]                 # хвост: вдруг граница разорвана
                break
            head_end = self.buf.find(b"\r\n\r\n", start)
            if head_end < 0:
                self.buf = self.buf[start:]
                break
            m = LENGTH.search(self.buf, start, head_end)
            if not m:                                    # граница без длины — пропустить её
                self.buf = self.buf[head_end + 4:]
                continue
            if int(m[1]) > self.max_frame:               # невозможная длина — начать заново,
                self.buf = b""                           # а не копить мегабайты в буфере
                break
            body = head_end + 4
            end = body + int(m[1])
            if len(self.buf) < end:
                self.buf = self.buf[start:]
                break
            frames.append(self.buf[body:end])
            self.buf = self.buf[end:]
        return frames


def looks_like_camera(response):
    """Ответ на /status — от нашего стримера?"""
    return b"OpenMV" in response


def _probe(ip, timeout=0.7):
    try:
        with socket.create_connection((ip, PORT), timeout=timeout) as s:
            s.settimeout(2.0)
            s.sendall(b"GET /status HTTP/1.0\r\n\r\n")
            data = b""
            while len(data) < 1500:
                chunk = s.recv(512)
                if not chunk:
                    break
                data += chunk
    except OSError:
        return None
    return ip if looks_like_camera(data) else None


def local_subnets():
    """Префиксы /24 сетей, в которых находится этот компьютер."""
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    try:                                                 # адрес «наружу» — основная сеть
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.168.255.255", 9))
            ips.add(s.getsockname()[0])
    except OSError:
        pass
    return sorted({ip.rsplit(".", 1)[0] for ip in ips if not ip.startswith(("127.", "169.254."))})


def find_camera(cached=None):
    """IP камеры: сначала запомненный, потом перебор локальных сетей. None — не нашли."""
    if cached and _probe(cached, timeout=1.5):
        return cached
    for net in local_subnets():
        with concurrent.futures.ThreadPoolExecutor(64) as pool:
            for ip in pool.map(_probe, (f"{net}.{i}" for i in range(1, 255))):
                if ip:
                    return ip
    return None


class CamStream:
    """Фоновое чтение потока. .frame — последний JPEG (bytes), .seq растёт с каждым кадром,
    .state — строка для показа пользователю."""

    def __init__(self, ip=None, cache_path=None):
        self.ip, self.cache_path = ip, cache_path
        self.frame, self.seq, self.fps = None, 0, 0.0
        self.state = "ищу камеру..."
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop = True

    def _cached(self):
        try:
            return open(self.cache_path, encoding="utf-8").read().strip() or None
        except (OSError, TypeError):
            return None

    def _run(self):
        while not self._stop:
            if not self.ip:
                self.state = "ищу камеру..."
                self.ip = find_camera(self._cached())
                if not self.ip:
                    self.state = "камера не найдена (ПК в сети 2.4 ГГц?)"
                    self._sleep(10)
                    continue
                if self.cache_path:
                    try:
                        open(self.cache_path, "w", encoding="utf-8").write(self.ip)
                    except OSError:
                        pass
            try:
                self._stream()
            except OSError:
                pass
            self.frame = None
            if not self._stop:
                self.state = f"нет видео с {self.ip}, переподключаюсь..."
                self.fails += 1
                if self.fails >= 5:                      # адрес мог смениться — искать заново
                    self.ip, self.fails = None, 0
                self._sleep(2)

    fails = 0

    def _sleep(self, sec):
        end = time.time() + sec
        while not self._stop and time.time() < end:
            time.sleep(0.1)

    def _stream(self):
        parser = FrameParser()
        t0, n = time.time(), 0
        with socket.create_connection((self.ip, PORT), timeout=3) as s:
            s.settimeout(5.0)                            # 5 с без данных — камера пропала
            s.sendall(b"GET /stream.jpg HTTP/1.0\r\n\r\n")
            while not self._stop:
                chunk = s.recv(16384)
                if not chunk:
                    return
                for jpeg in parser.feed(chunk):
                    self.frame, self.seq = jpeg, self.seq + 1
                    self.fails = 0
                    n += 1
                    if time.time() - t0 >= 1.0:
                        self.fps, t0, n = n / (time.time() - t0), time.time(), 0
                    self.state = f"{self.ip}  {self.fps:.0f} кадр/с"


if __name__ == "__main__":                               # быстрая проверка: python wasdcam.py
    cam = CamStream(cache_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".wasd_cam"))
    for _ in range(15):
        time.sleep(1)
        print(cam.state, "| кадров:", cam.seq, "| размер:", len(cam.frame or b""))
