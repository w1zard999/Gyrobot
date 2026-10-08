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


def missing_deps():
    import importlib
    importlib.invalidate_caches()
    out = []
    for mod, pkg in (('pygame', 'pygame'), ('serial', 'pyserial')):
        try:
            __import__(mod)
        except ImportError:
            out.append(pkg)
    return out


def linux_install_hint(missing):
    """Команда установки для дистрибутива (по /etc/os-release)."""
    ids = ''
    try:
        for line in open('/etc/os-release', encoding='utf-8'):
            if line.startswith(('ID=', 'ID_LIKE=')):
                ids += ' ' + line.split('=', 1)[1].strip().strip('"').lower()
    except OSError:
        pass
    names = {'pygame': 'pygame', 'pyserial': 'serial'}
    if any(d in ids for d in ('debian', 'ubuntu')):
        return 'sudo apt install ' + ' '.join(f'python3-{names[p]}' for p in missing)
    if any(d in ids for d in ('fedora', 'rhel', 'centos')):
        return 'sudo dnf install ' + ' '.join(f'python3-{p}' for p in missing)
    if any(d in ids for d in ('arch', 'manjaro')):
        return 'sudo pacman -S ' + ' '.join(f'python-{p}' for p in missing)
    return 'через пакетный менеджер дистрибутива (пакеты python3-pygame, python3-serial)'


def wait_for_deps_linux():
    """Linux: сами ничего не ставим — сообщаем и ждём, пока пользователь поставит."""
    while True:
        missing = missing_deps()
        if not missing:
            return
        print(f'\nНе установлены пакеты Python: {", ".join(missing)}')
        print(f'  Установи:  {linux_install_hint(missing)}')
        print(f'  или:       {sys.executable} -m pip install --user {" ".join(missing)}')
        try:
            input('После установки нажми Enter для проверки (Ctrl+C — выход)... ')
        except (KeyboardInterrupt, EOFError):
            print()
            sys.exit(1)


def ensure_deps():
    """Нет pygame/pyserial — поставить. Сначала обычным pip; если система не
    даёт (Homebrew на маке, PEP 668) — в своё окружение tools/.venv.
    На Linux ничего не ставим, только просим установить (wait_for_deps_linux)."""
    if have_deps():
        return
    if sys.platform.startswith('linux'):
        wait_for_deps_linux()
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
import math  # noqa: E402
import io  # noqa: E402
from wasdcam import CamStream  # noqa: E402
from wasdmap import MapView, Trail, parse_pose  # noqa: E402

CAM_CACHE = os.path.join(HERE, '.wasd_cam')     # последний адрес камеры
PORT_CACHE = os.path.join(HERE, '.wasd_port')   # последний рабочий порт — пробуем первым
RATE = 10.0                                      # букв в секунду (прошивка ждёт 250 мс)
CMS_PER_UNIT = 0.694 / 0.4                      # v в телеметрии — имп/40мс; 0.694 мм/имп (рулетка)
NAV_BALL = 4                                     # nav= в телеметрии: едет за мячом

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


def open_error(e):
    """Понятная причина, почему порт не открылся."""
    msg = str(e).lower()
    if any(x in msg for x in ('busy', 'denied', 'permission', 'отказано', 'errno 16', 'errno 13')):
        return 'занят другой программой (или другим компьютером)'
    return f'не открылся ({e})'


def probe(port, timeout=4.0):
    """Открыть порт и спросить параметры. Вернуть (порт, None), если ответил робот,
    иначе (None, причина)."""
    try:
        ser = serial.Serial(port, 115200, timeout=0.2, write_timeout=2)
    except (serial.SerialException, OSError) as e:
        return None, open_error(e)
    try:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.write(b'p\n')
        end = time.time() + timeout
        while time.time() < end:
            if b'kp=' in ser.readline():
                return ser, None
        why = 'открылся, но робот не отвечает'
    except (serial.SerialException, OSError) as e:
        why = f'связь оборвалась ({e})'
    ser.close()
    return None, why


# ---------- один клиент на компьютере ----------
# Клиент, закрытый нештатно (упал Терминал, завис на Bluetooth), может остаться
# жить и держать порт — тогда следующий запуск робота не находит. Поэтому при
# старте закрываем предыдущий клиент, если он ещё жив.
LOCK = os.path.join(HERE, '.wasd.lock')


def is_our_client(pid):
    """Жив ли процесс pid и наш ли это wasd.py (номер мог достаться чужой программе)."""
    if pid <= 0 or pid == os.getpid():
        return False
    try:
        if os.name == 'nt':
            cmd = f"(Get-CimInstance Win32_Process -Filter 'ProcessId={pid}').CommandLine"
            out = subprocess.run(['powershell', '-NoProfile', '-Command', cmd],
                                 capture_output=True, text=True, timeout=15).stdout
        else:
            out = subprocess.run(['ps', '-p', str(pid), '-o', 'command='],
                                 capture_output=True, text=True, timeout=5).stdout
        return 'wasd.py' in out
    except (OSError, subprocess.SubprocessError):
        return False


def kill_client(pid):
    if os.name == 'nt':
        subprocess.run(['taskkill', '/PID', str(pid), '/F'], capture_output=True)
        return
    import signal
    try:
        os.kill(pid, signal.SIGTERM)
        for _ in range(20):                   # до 2 с на аккуратный выход
            time.sleep(0.1)
            os.kill(pid, 0)
        os.kill(pid, signal.SIGKILL)
    except OSError:
        pass                                  # процесса уже нет


def take_lock():
    try:
        pid = int(open(LOCK, encoding='utf-8').read().strip())
    except (OSError, ValueError):
        pid = 0
    if is_our_client(pid):
        print(f'Найден незакрытый клиент (процесс {pid}) — закрываю его, чтобы освободить порт.')
        kill_client(pid)
        time.sleep(1.5)                       # Bluetooth-порту нужно время освободиться
    try:
        open(LOCK, 'w', encoding='utf-8').write(str(os.getpid()))
    except OSError:
        pass


def release_lock():
    try:
        if int(open(LOCK, encoding='utf-8').read().strip()) == os.getpid():
            os.remove(LOCK)
    except (OSError, ValueError):
        pass


def find_robot(port_arg):
    if port_arg:
        print(f'Порт {port_arg}...')
        try:
            return serial.Serial(port_arg, 115200, timeout=0.2, write_timeout=2), port_arg
        except (serial.SerialException, OSError) as e:
            sys.exit(f'{port_arg}: {open_error(e)}')
    cands = bluetooth_ports()
    try:
        cached = open(PORT_CACHE, encoding='utf-8').read().strip()
        if cached in cands:
            cands.remove(cached)
            cands.insert(0, cached)
    except OSError:
        pass
    for port in cands:
        print(f'Ищу робота на {port}...', end=' ', flush=True)
        ser, why = probe(port)
        print('найден' if ser else why)
        if ser:
            try:
                open(PORT_CACHE, 'w', encoding='utf-8').write(port)
            except OSError:
                pass
            return ser, port
    print('\nРобот не найден. Что проверить:')
    print('  - робот включён, HC-05 спарен с этим компьютером (PIN 1234);')
    print('  - к роботу не подключён другой компьютер или телефон — у HC-05 одно подключение;')
    print('  - если порт «занят», а других подключений нет — выключи и включи робота.')
    print('Порты в системе:')
    for p in list_ports.comports():
        print(f'  {p.device}  —  {p.description}')
    print('Можно указать порт вручную: --port COM13  |  /dev/cu.HC-05  |  /dev/rfcomm0')
    sys.exit(1)


# ---------- связь ----------
class Link:
    """Порт + поток чтения телеметрии в лог и в последнее состояние."""
    TELE = re.compile(r'^(A|-) a=(-?[\d.]+) r=\S+ v=(-?[\d.]+).*?yr=(-?\d+)')
    BALL = re.compile(r' b=(-?\d+)/(\d+)')      # мяч в кадре: смещение/размер

    def __init__(self, ser, port, held, log):
        self.ser, self.port, self.held, self.log = ser, port, held, log
        self.lock = threading.Lock()
        self.last_rx = 0.0
        self.tele = None          # (взведён, наклон, скорость, рыскание)
        self.pose = None          # (x см, y см, курс °, фаза возврата)
        self.ball = None          # (смещение, размер) мяча в кадре, пикс; None — не виден
        self.trail = Trail()
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
            if m:
                b = self.BALL.search(line)
                self.ball = (int(b[1]), int(b[2])) if b else None
            pose = parse_pose(line)
            if pose:
                self.pose = pose
                self.trail.add(pose[0], pose[1])
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

    def send_cmd(self, cmd):
        """Одна команда роботу (h, x, home, g)."""
        if self.ser is None:
            return
        try:
            self.ser.write((cmd + '\n').encode())
        except (serial.SerialException, OSError):
            self._drop('не удалось отправить')

    def reconnect(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.2, write_timeout=2)
            self.error = ''
        except (serial.SerialException, OSError):
            pass


# ---------- окно ----------
def draw(screen, fonts, link, held, cam=None):
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
                 f'скорость  {v * CMS_PER_UNIT:+5.1f} см/с',
                 f'поворот  {yr:+4d} °/с']
    else:
        lines = ['телеметрии пока нет']
    for i, s in enumerate(lines):
        screen.blit(small.render(s, True, (220, 220, 220)), (230, 55 + i * 30))
    nav = link.pose[3] if link.pose else 0
    if nav == NAV_BALL:
        seen = 'вижу, %+d / %d пикс' % link.ball if link.ball else 'не вижу, жду'
        screen.blit(small.render(f'за мячом: {seen}', True, (240, 120, 100)), (230, 175))
    elif nav:
        x, y = link.pose[0], link.pose[1]
        phase = {1: 'разворот', 2: 'едет', 3: 'доворот'}.get(nav, '')
        screen.blit(small.render(f'домой: {phase}, {math.hypot(x, y):.0f} см', True, (240, 200, 90)),
                    (230, 175))
    draw_video(screen, small, cam, VIDEO)
    for i, s in enumerate(('W A S D — ехать    H — домой    ESC — выход',
                           'F — за мячом    R — дом здесь    C — стереть след')):
        screen.blit(small.render(s, True, (120, 120, 130)), (16, 472 + i * 24))
    draw_map(screen, small, link, MAP)
    pygame.display.flip()


VIDEO = pygame.Rect(16, 215, 320, 240)    # видео с камеры под клавишами
_video = {'seq': -1, 'surf': None}        # последний декодированный кадр


def draw_video(screen, font, cam, rect):
    """Кадр с камеры; JPEG декодируется только когда пришёл новый."""
    pygame.draw.rect(screen, (32, 35, 40), rect, border_radius=8)
    if cam is None:
        msg = 'видео выключено (--no-cam)'
    else:
        jpeg, seq = cam.frame, cam.seq
        if jpeg is None:
            _video['surf'] = None
        elif seq != _video['seq']:
            try:
                surf = pygame.image.load(io.BytesIO(jpeg))
                if surf.get_size() != rect.size:
                    surf = pygame.transform.scale(surf, rect.size)
                _video['surf'], _video['seq'] = surf, seq
            except pygame.error:              # битый кадр — оставить прошлый
                pass
        msg = cam.state
    if cam is not None and _video['surf'] is not None:
        screen.blit(_video['surf'], rect.topleft)
        label = font.render(msg, True, (230, 230, 230))
        bg = pygame.Surface((label.get_width() + 8, label.get_height()), pygame.SRCALPHA)
        bg.fill((0, 0, 0, 120))
        screen.blit(bg, (rect.x, rect.bottom - label.get_height()))
        screen.blit(label, (rect.x + 4, rect.bottom - label.get_height()))
    else:
        label = font.render(msg, True, (120, 120, 130))
        screen.blit(label, (rect.centerx - label.get_width() // 2, rect.centery - 10))


MAP = pygame.Rect(470, 30, 410, 410)      # квадрат карты справа


def draw_map(screen, font, link, rect):
    """След, дом (крестик) и робот (стрелка по курсу); дом в центре."""
    pygame.draw.rect(screen, (32, 35, 40), rect, border_radius=8)
    pose = link.pose
    pts = list(link.trail.points)
    view = MapView(rect.width, min_extent=50)
    view.fit(pts, (pose[0], pose[1]) if pose else (0, 0))

    def scr(x, y):
        px, py = view.to_screen(x, y)
        return rect.x + px, rect.y + py

    step = 10 if view.scale * 10 >= 6 else 50           # сетка: 10 см, если не слишком густо
    n = int(view.extent // step) + 1
    for i in range(-n, n + 1):
        col = (58, 62, 70) if (i * step) % 50 else (78, 84, 94)
        a, b = scr(i * step, -view.extent), scr(i * step, view.extent)
        c, d = scr(-view.extent, i * step), scr(view.extent, i * step)
        screen.set_clip(rect)
        pygame.draw.line(screen, col, a, b)
        pygame.draw.line(screen, col, c, d)
        screen.set_clip(None)
    if len(pts) > 1:
        pygame.draw.lines(screen, (90, 160, 250), False, [scr(x, y) for x, y in pts], 2)
    hx, hy = scr(0, 0)                                    # дом
    pygame.draw.line(screen, (240, 200, 90), (hx - 8, hy - 8), (hx + 8, hy + 8), 3)
    pygame.draw.line(screen, (240, 200, 90), (hx - 8, hy + 8), (hx + 8, hy - 8), 3)
    if pose:                                              # робот: треугольник по курсу
        rx, ry = scr(pose[0], pose[1])
        th = math.radians(pose[2])                        # 0° — вверх, + — влево
        tip = (rx - 14 * math.sin(th), ry - 14 * math.cos(th))
        l = (rx - 8 * math.sin(th + 2.5), ry - 8 * math.cos(th + 2.5))
        r = (rx - 8 * math.sin(th - 2.5), ry - 8 * math.cos(th - 2.5))
        pygame.draw.polygon(screen, (110, 200, 120), (tip, l, r))
    scale_cm = step if step == 50 else 10
    screen.blit(font.render(f'клетка {scale_cm} см', True, (120, 120, 130)),
                (rect.x + 8, rect.bottom - 26))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', help='порт робота; без него ищется сам')
    ap.add_argument('--no-log', action='store_true', help='не писать телеметрию в tools/logs')
    ap.add_argument('--cam', help='IP камеры OpenMV; без него ищется сама в локальной сети')
    ap.add_argument('--no-cam', action='store_true', help='не показывать видео с камеры')
    args = ap.parse_args()

    take_lock()
    import signal

    def stop(signum, frame):                  # закрыли Терминал / kill — выйти чисто
        raise SystemExit(0)
    for name in ('SIGTERM', 'SIGHUP'):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), stop)
    try:
        run(args)
    except KeyboardInterrupt:
        pass
    finally:
        release_lock()


def run(args):
    ser, port = find_robot(args.port)
    print(f'Робот на {port}. Окно управления открыто — ESC для выхода.')

    log = None
    if not args.no_log:
        os.makedirs(os.path.join(HERE, 'logs'), exist_ok=True)
        log = open(os.path.join(HERE, 'logs', time.strftime('wasd-%Y%m%d-%H%M%S.log')),
                   'w', encoding='utf-8')

    held = set()
    link = Link(ser, port, held, log)
    # видео идёт отдельным потоком и на управление не влияет: нет камеры — заглушка
    cam = None if args.no_cam else CamStream(ip=args.cam, cache_path=CAM_CACHE)

    pygame.init()
    screen = pygame.display.set_mode((895, 530))
    pygame.display.set_caption('Gyrobot WASD')
    face = 'arial,helvetica,dejavusans,liberationsans,notosans'   # с кириллицей
    fonts = (pygame.font.SysFont(face, 30, bold=True), pygame.font.SysFont(face, 20))
    clock = pygame.time.Clock()

    try:
        loop(screen, fonts, clock, link, held, cam)
    finally:                                  # любой выход: ESC, окно, сигнал, ошибка
        pygame.quit()
        link.send_cmd('x')                    # остановить возврат домой, если шёл
        if cam:
            cam.stop()
        if link.ser:
            try:
                link.ser.close()
            except (serial.SerialException, OSError):
                pass
        if log:
            log.close()
    print('Выход: буквы больше не шлются, робот остановится и будет стоять.')


def loop(screen, fonts, clock, link, held, cam=None):
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
                elif ev.scancode == pygame.KSCAN_H:
                    link.send_cmd('h')                    # домой
                elif ev.scancode == pygame.KSCAN_F:       # за мячом: вкл / выкл
                    following = link.pose and link.pose[3] == NAV_BALL
                    link.send_cmd('x' if following else 'g')
                elif ev.scancode == pygame.KSCAN_R:
                    link.send_cmd('home')                 # дом здесь
                    link.trail.clear()
                elif ev.scancode == pygame.KSCAN_C:
                    link.trail.clear()
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

        draw(screen, fonts, link, held, cam)
        clock.tick(30)


if __name__ == '__main__':
    main()
