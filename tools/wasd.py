#!/usr/bin/env python3
"""Управление гироботом с клавиатуры по Bluetooth — Windows, macOS, Linux.

Открывается окно: пока оно в фокусе, W/A/S/D (или стрелки) — ехать, отпустил —
стоит. ESC или закрыть окно — выход. Клавиши берутся по физическому положению,
раскладка не важна. Порт HC-05 ищется сам: клиент спрашивает у Bluetooth-портов
параметры ("p") и берёт тот, что ответил как робот. Можно задать --port.

Зависимости (pygame, pyserial) ставятся сами при первом запуске.
Телеметрия пишется в tools/logs/wasd-*.log (разбор: turnstat.py, stopstat.py).
"""
import argparse
import os
import re
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
VENV = os.path.join(HERE, '.venv')
VENV_PY = os.path.join(VENV, 'Scripts', 'python.exe') if os.name == 'nt' else os.path.join(VENV, 'bin', 'python')
DEPS = ['pygame', 'pyserial']


def have_deps():
    import importlib
    importlib.invalidate_caches()            # увидеть пакеты, поставленные только что
    try:
        import pygame, serial  # noqa: F401
        return True
    except ImportError:
        return False


def rerun_in(python):
    """Перезапустить клиент другим интерпретатором с теми же аргументами."""
    sys.exit(subprocess.call([python, os.path.abspath(__file__)] + sys.argv[1:]))


def ensure_deps():
    """Нет pygame/pyserial — поставить. Сначала обычным pip; если система не
    даёт (Linux/Homebrew, PEP 668) — в своё окружение tools/.venv."""
    if have_deps():
        return
    in_venv = os.path.abspath(sys.executable) == os.path.abspath(VENV_PY)
    if not in_venv and os.path.exists(VENV_PY):
        rerun_in(VENV_PY)                    # окружение уже есть с прошлого раза
    print(f'Не хватает пакетов, ставлю: {" ".join(DEPS)}...')
    pip = [sys.executable, '-m', 'pip', 'install', '--disable-pip-version-check', *DEPS]
    if subprocess.call(pip) == 0 and have_deps():
        return
    if in_venv:
        sys.exit(f'Не удалось поставить пакеты. Вручную: {sys.executable} -m pip install {" ".join(DEPS)}')
    print(f'Обычная установка не прошла — создаю окружение {VENV}...')
    if subprocess.call([sys.executable, '-m', 'venv', VENV]) != 0:
        sys.exit('Не удалось создать окружение. На Debian/Ubuntu: sudo apt install python3-venv, '
                 f'или поставь пакеты сам: pip install {" ".join(DEPS)}')
    if subprocess.call([VENV_PY, '-m', 'pip', 'install', '--disable-pip-version-check', *DEPS]) != 0:
        sys.exit(f'Не удалось поставить пакеты в {VENV}. Проверь интернет.')
    rerun_in(VENV_PY)


ensure_deps()
import pygame  # noqa: E402
import serial  # noqa: E402
from serial.tools import list_ports  # noqa: E402

PORT_CACHE = os.path.join(HERE, '.wasd_port')   # последний рабочий порт — пробуем первым
RATE = 10.0                                      # букв в секунду (прошивка ждёт 250 мс)

# физические клавиши (SDL scancode) -> буква протокола
KEYMAP = {
    pygame.KSCAN_W: 'w', pygame.KSCAN_S: 's', pygame.KSCAN_A: 'a', pygame.KSCAN_D: 'd',
    getattr(pygame, 'KSCAN_UP', 82): 'w', getattr(pygame, 'KSCAN_DOWN', 81): 's',
    getattr(pygame, 'KSCAN_LEFT', 80): 'a', getattr(pygame, 'KSCAN_RIGHT', 79): 'd',
}


# ---------- поиск порта ----------
def bluetooth_ports():
    """Порты, похожие на исходящий Bluetooth SPP, для текущей ОС."""
    out = []
    for p in list_ports.comports():
        dev, hwid = p.device, (p.hwid or '').upper()
        if sys.platform == 'win32':
            # у входящего порта в hwid нулевой адрес устройства
            if 'BTHENUM' in hwid and '000000000000' not in hwid:
                out.append(dev)
        elif sys.platform == 'darwin':
            if dev.startswith('/dev/cu.') and 'usb' not in dev.lower() and not any(
                    x in dev for x in ('Bluetooth-Incoming-Port', 'debug-console', 'wlan')):
                out.append(dev)
        else:
            if 'rfcomm' in dev:
                out.append(dev)
    return out


def probe(port, timeout=4.0):
    """Открыть порт и спросить параметры. Вернуть открытый порт, если ответил робот."""
    try:
        ser = serial.Serial(port, 115200, timeout=0.2, write_timeout=2)
    except (serial.SerialException, OSError):
        return None
    try:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.write(b'p\n')
        end = time.time() + timeout
        while time.time() < end:
            if b'kp=' in ser.readline():
                return ser
    except (serial.SerialException, OSError):
        pass
    ser.close()
    return None


def find_robot(port_arg):
    if port_arg:
        print(f'Порт {port_arg}...')
        return serial.Serial(port_arg, 115200, timeout=0.2, write_timeout=2), port_arg
    cands = bluetooth_ports()
    try:
        cached = open(PORT_CACHE, encoding='utf-8').read().strip()
        if cached in cands:
            cands.remove(cached)
            cands.insert(0, cached)
    except OSError:
        pass
    for port in cands:
        print(f'Ищу робота на {port}...')
        ser = probe(port)
        if ser:
            try:
                open(PORT_CACHE, 'w', encoding='utf-8').write(port)
            except OSError:
                pass
            return ser, port
    print('\nРобот не найден. Робот включён, HC-05 спарен с компьютером?')
    print('Порты в системе:')
    for p in list_ports.comports():
        print(f'  {p.device}  —  {p.description}')
    print('Можно указать порт вручную: --port COM13  |  /dev/cu.HC-05  |  /dev/rfcomm0')
    sys.exit(1)


# ---------- связь ----------
class Link:
    """Порт + поток чтения телеметрии в лог и в последнее состояние."""
    TELE = re.compile(r'^(A|-) a=(-?[\d.]+) r=\S+ v=(-?[\d.]+).*?yr=(-?\d+)')

    def __init__(self, ser, port, held, log):
        self.ser, self.port, self.held, self.log = ser, port, held, log
        self.lock = threading.Lock()
        self.last_rx = 0.0
        self.tele = None          # (взведён, наклон, скорость, рыскание)
        self.error = ''
        self.t0 = time.time()
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while True:
            ser = self.ser
            if ser is None:
                time.sleep(0.2)
                continue
            try:
                data = ser.readline()
            except (serial.SerialException, OSError, TypeError):
                self._drop('порт отвалился')
                continue
            if not data:
                continue
            self.last_rx = time.time()
            line = data.decode('utf-8', 'replace').rstrip()
            m = self.TELE.match(line)
            if m:
                self.tele = (m[1] == 'A', float(m[2]), float(m[3]), int(m[4]))
            if self.log:
                with self.lock:
                    keys = ''.join(sorted(self.held)) or '.'
                self.log.write(f'{time.time() - self.t0:8.2f} {keys:4s} {line}\n')
                self.log.flush()

    def _drop(self, why):
        self.error = why
        try:
            if self.ser:
                self.ser.close()
        except (serial.SerialException, OSError):
            pass
        self.ser = None

    def send(self, letters):
        if self.ser is None:
            return
        try:
            for ch in letters:
                self.ser.write((ch + '\n').encode())
        except (serial.SerialException, OSError):
            self._drop('не удалось отправить')

    def reconnect(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.2, write_timeout=2)
            self.error = ''
        except (serial.SerialException, OSError):
            pass


# ---------- окно ----------
def draw(screen, fonts, link, held):
    big, small = fonts
    screen.fill((24, 26, 30))
    online = link.ser is not None and time.time() - link.last_rx < 1.5
    status = 'связь есть' if online else (link.error or 'нет данных от робота')
    screen.blit(small.render(f'{link.port}: {status}', True,
                             (110, 200, 120) if online else (220, 110, 100)), (16, 12))

    # клавиши: W сверху, A S D снизу
    for ch, (x, y) in {'w': (80, 50), 'a': (20, 110), 's': (80, 110), 'd': (140, 110)}.items():
        on = ch in held
        pygame.draw.rect(screen, (90, 160, 250) if on else (55, 60, 70), (x, y, 52, 52), border_radius=8)
        t = big.render(ch.upper(), True, (255, 255, 255))
        screen.blit(t, (x + 26 - t.get_width() // 2, y + 26 - t.get_height() // 2))

    if link.tele:
        armed, a, v, yr = link.tele
        lines = [('балансирует' if armed else 'лежит / ждёт калибровки'),
                 f'наклон  {a:+5.1f}°',
                 f'скорость  {v:+5.1f} см/с',
                 f'поворот  {yr:+4d} °/с']
    else:
        lines = ['телеметрии пока нет']
    for i, s in enumerate(lines):
        screen.blit(small.render(s, True, (220, 220, 220)), (230, 55 + i * 30))
    screen.blit(small.render('ESC — выход', True, (120, 120, 130)), (16, 200))
    pygame.display.flip()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', help='порт робота; без него ищется сам')
    ap.add_argument('--no-log', action='store_true', help='не писать телеметрию в tools/logs')
    args = ap.parse_args()

    ser, port = find_robot(args.port)
    print(f'Робот на {port}. Окно управления открыто — ESC для выхода.')

    log = None
    if not args.no_log:
        os.makedirs(os.path.join(HERE, 'logs'), exist_ok=True)
        log = open(os.path.join(HERE, 'logs', time.strftime('wasd-%Y%m%d-%H%M%S.log')),
                   'w', encoding='utf-8')

    held = set()
    link = Link(ser, port, held, log)

    pygame.init()
    screen = pygame.display.set_mode((460, 240))
    pygame.display.set_caption('Gyrobot WASD')
    face = 'arial,helvetica,dejavusans,liberationsans,notosans'   # с кириллицей
    fonts = (pygame.font.SysFont(face, 30, bold=True), pygame.font.SysFont(face, 20))
    clock = pygame.time.Clock()
    next_send = next_retry = 0.0

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.scancode in KEYMAP:
                    with link.lock:
                        held.add(KEYMAP[ev.scancode])
            elif ev.type == pygame.KEYUP and ev.scancode in KEYMAP:
                with link.lock:
                    held.discard(KEYMAP[ev.scancode])
            elif ev.type == getattr(pygame, 'WINDOWFOCUSLOST', -1):
                with link.lock:
                    held.clear()          # ушёл из окна — робот останавливается

        now = time.time()
        if now >= next_send:
            next_send = now + 1.0 / RATE
            with link.lock:
                keys = sorted(held)
            link.send(keys)
        if link.ser is None and now >= next_retry:
            next_retry = now + 2.0
            link.reconnect()

        draw(screen, fonts, link, held)
        clock.tick(30)

    pygame.quit()
    if link.ser:
        link.ser.close()
    if log:
        log.close()
    print('Выход: буквы больше не шлются, робот остановится и будет стоять.')


if __name__ == '__main__':
    main()
