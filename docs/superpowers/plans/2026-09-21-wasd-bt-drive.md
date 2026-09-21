# WASD-управление по Bluetooth — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Езда гироботом с клавиатуры ПК: WASD по Bluetooth (HC-05 на D0/D1), робот непрерывно балансирует.

**Architecture:** W/S плавно (TAU 0.3 с) сдвигают точку равновесия (наклон = движение), A/D дают дифференциал на моторы поверх контура курса. Watchdog 300 мс глушит цели без потока букв. ПК-клиент шлёт буквы зажатых клавиш 10 Гц.

**Tech Stack:** Arduino Nano C++ (arduino-cli, ядро arduino:avr), Python 3 + pyserial + pynput, HC-05 (SPP, 115200).

## Global Constraints

- Скетч: `firmware/gyrobot_mov` (config.h + imu.h + motors.h + speed.h + gyrobot_mov.ino), собирается `arduino-cli compile -b arduino:avr:nano`
- Сериал 115200, парсер строк уже есть (`paramsPoll`/`applyParam` в gyrobot_mov.ino) — буквы ДОБАВЛЯЮТСЯ туда, ничего не ломая
- Компиляция не должна превысить RAM 70% (сейчас 618/2048 Б)
- Прошивка на плату: COM10, при отсоединённом/обесточенном HC-05
- Знак «вперёд» (drv) и знак поворота (trn) — живые параметры, физический знак проверяется на столе/полу, в коде не хардкодится
- Спека: `docs/superpowers/specs/2026-09-21-wasd-bt-drive-design.md`

---

### Task 1: Прошивка — состояние WASD, парсер букв, watchdog

**Files:**
- Modify: `firmware/gyrobot_mov/gyrobot_mov.ino` (состояние, applyParam, loop-watchdog)
- Modify: `firmware/gyrobot_mov/config.h` (extern drv/trn)

**Interfaces:**
- Consumes: существующие `paramsPoll()`/`applyParam()`, `millis()`
- Produces: глобальные `float drvTarget, drvNow, trnTarget, trnNow; uint32_t cmdDeadline; bool kw, ks, ka, kd;` и живые параметры `drv`, `trn` (Task 2 использует drvNow/trnNow)

- [ ] **Step 1: Добавить состояние WASD** — в gyrobot_mov.ino после блока «Живая настройка»:

```cpp
// ---------- WASD (BT/USB) ----------
float drv = 2.5f, trn = 35.0f;      // целевой наклон, °; дифференциал поворота, ШИМ
float drvTarget = 0, drvNow = 0;    // сглаженный наклон добавляется к нолю равновесия
float trnTarget = 0, trnNow = 0;    // дифференциал: A +, B −
bool kw = false, ks = false, ka = false, kd = false;
uint32_t cmdDeadline = 0;
```

В config.h в блок «живые параметры» добавить:

```cpp
extern float drv, trn;                       // WASD: наклон и поворот (сериал)
extern float trnTarget;                      // угасание yawInt в speed.h
```

- [ ] **Step 2: Буквы в applyParam** — перед закрывающей `}` applyParam добавить ветку (после esb):

```cpp
  } else if (name[1] == 0 && strchr("wsad", name[0])) {
    if (name[0] == 'w') kw = true;
    if (name[0] == 's') ks = true;
    if (name[0] == 'a') ka = true;
    if (name[0] == 'd') kd = true;
    cmdDeadline = millis() + 300;
    drvTarget = (kw ? drv : 0) - (ks ? drv : 0);
    trnTarget = (kd ? trn : 0) - (ka ? trn : 0);
    Serial.println(F("ok"));
  }
```

- [ ] **Step 3: Команды drv/trn в applyParam** — рядом с остальными:

```cpp
  } else if (!strcmp(name, "drv") && v >= -10 && v <= 10) { drv = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "trn") && v >= -100 && v <= 100) { trn = v; Serial.println(F("ok"));
```

- [ ] **Step 4: Watchdog в начале loop()** (сразу после `paramsPoll();`):

```cpp
  if (millis() > cmdDeadline && (kw || ks || ka || kd)) {   // поток букв кончился — стоим
    kw = ks = ka = kd = false;
    drvTarget = 0; trnTarget = 0;
  }
```

- [ ] **Step 5: Компиляция**

Run: `& "$env:LOCALAPPDATA\arduino-cli\arduino-cli.exe" compile -b arduino:avr:nano "C:\Users\kyoto\Documents\Gyrobot\firmware\gyrobot_mov"`
Expected: SUCCESS, RAM ≤ 70%

- [ ] **Step 6: Коммит**

```bash
git add firmware/gyrobot_mov
git commit -m "mov: WASD-состояние, буквы в парсере, watchdog 300мс"
```

---

### Task 2: Прошивка — применение наклона и поворота

**Files:**
- Modify: `firmware/gyrobot_mov/gyrobot_mov.ino:2` (сглаживание после dt; фильтр угла; телеметрия d=; сброс при падении)
- Modify: `firmware/gyrobot_mov/motors.h` (trnNow в дифференциал drive)
- Modify: `firmware/gyrobot_mov/speed.h` (угасание yawInt при повороте)

**Interfaces:**
- Consumes: `drvTarget/trnTarget/drvNow/trnNow` из Task 1, `yawInt` из speed.h, `dt` из loop
- Produces: наклон входит в комплементарный фильтр; дифференциал в drive(); телеметрия `d=`

- [ ] **Step 1: Сглаживание + вход наклона в фильтр** — в loop(), ветка armed, после строки `if (dt > 0.02f) dt = 0.02f;`:

```cpp
  drvNow += (drvTarget - drvNow) * (dt / 0.3f);   // TAU 0.3 c: ступенька = качели
  trnNow += (trnTarget - trnNow) * (dt / 0.3f);
```

и строку фильтра заменить на:

```cpp
  GyYsum += rate * dt + (accAngle() + balancing_zerro + drvNow - GyYsum) * (dt / TAU_ACC);
```

- [ ] **Step 2: Дифференциал в drive()** — motors.h, заменить вычисление c и analogWrite:

```cpp
  int c = (int)constrain(yawTerm + trnNow, -60.0f, 60.0f);
  analogWrite(PWMA, constrain(p + (int)(fwd ? atr : atrb) + c, 0, 255));
  analogWrite(PWMB, constrain(p - c, 0, 255));
```

(ворота `(p > 0)` убрать: при p=0 дифференциал = поворот на месте — это штатно)

- [ ] **Step 3: Угасание yawInt при повороте** — speed.h, в конец speedTick():

```cpp
  if (trnTarget != 0) yawInt *= 0.9f;   // водитель рулит — курс-холд молчит
```

- [ ] **Step 4: Телеметрия и сброс при падении** — в armed-телеметрию после `zr=`:

```cpp
    Serial.print(F(" d=")); Serial.print(drvNow, 2);
```

в ветку падения (`armed = false; uprightSince = 0;`):

```cpp
    drvNow = 0; trnNow = 0; drvTarget = 0; trnTarget = 0;
    kw = ks = ka = kd = false;
```

- [ ] **Step 5: Компиляция**

Run: `& "$env:LOCALAPPDATA\arduino-cli\arduino-cli.exe" compile -b arduino:avr:nano "C:\Users\kyoto\Documents\Gyrobot\firmware\gyrobot_mov"`
Expected: SUCCESS, RAM ≤ 70%

- [ ] **Step 6: Залить и стол-тест (колёса в воздухе)**

```bash
& "$env:LOCALAPPDATA\arduino-cli\arduino-cli.exe" upload -p COM10 -b arduino:avr:nano "C:\Users\kyoto\Documents\Gyrobot\firmware\gyrobot_mov"
python tools/talk.py --port COM10 --boot 3 --cmds "w;w;w" --listen 6
```
Expected: держа робота в руках, видим в телеметрии `d=` вырос до ~2.5, через ~3 с после конца команд `d=` -> 0. Колёса крутятся «вперёд».

- [ ] **Step 7: Коммит**

```bash
git add firmware/gyrobot_mov
git commit -m "mov: WASD едет — наклон в фильтр, дифференциал в drive, угасание yawInt"
```

---

### Task 3: HC-05 — подключение, 115200, сопряжение (ручные шаги)

**Files:**
- Нет кода. Физические действия + проверка.

- [ ] **Step 1: Подключить модуль**: HC-05 TX -> D0 (RX Nano), HC-05 RX -> D1 через делитель 1к/2к (с D1), VCC/GND — 5V/GND
- [ ] **Step 2: Скорость 115200**: подать питание с зажатой кнопкой EN (AT-режим, светодиод медленный), с USB-UART (или через сам Nano при снятом TX модуля с D0) послать: `AT+UART=115200,0,0`, ответ `OK`. Проверить `AT+UART?`
- [ ] **Step 3: Сопряжение с Windows**: Параметры -> Bluetooth -> добавить HC-05, PIN `1234` (или `0000`). Диспетчер устройств -> Порты -> исходящий «Standard Serial over Bluetooth (COMx)» — номер запомнить
- [ ] **Step 4: Проверка канала**: USB воткнут, HC-05 запитан, робот лежит:

```bash
python -c "import serial; s=serial.Serial('COMx',115200); s.write(b'w\n'); s.close()"
```
и параллельно `python tools/talk.py --port COM10 --listen 5` — Expected: в телеметрии мелькнуло `ok`/`d=` дёрнулся (буква с BT дошла через общий UART)

- [ ] **Step 5: Коммит** (если было что менять в docs/STATUS)

```bash
git add STATUS.md; git commit -m "док: HC-05 на D0/D1, 115200, BT-COM номер"
```

---

### Task 4: ПК-клиент tools/wasd.py

**Files:**
- Create: `tools/wasd.py`

**Interfaces:**
- Consumes: протокол букв (Task 1) по BT COM на 115200
- Produces: ручное управление роботом с клавиатуры

- [ ] **Step 1: Установить зависимость**

Run: `pip install pynput`
Expected: Successfully installed

- [ ] **Step 2: Написать tools/wasd.py**

```python
#!/usr/bin/env python3
"""WASD-руление гироботом по BT COM: зажал — едет, отпустил — стоит. ESC — выход.

Раскладка клавиатуры должна be EN: шлются латинские wasd.
"""
import argparse
import threading
import time

import serial
from pynput import keyboard

ap = argparse.ArgumentParser()
ap.add_argument('--port', required=True, help='исходящий BT COM робота')
ap.add_argument('--rate', type=float, default=10.0, help='частота посылки, Гц')
a = ap.parse_args()

held = set()
lock = threading.Lock()

def ch_of(key):
    try:
        return (key.char or '').lower()
    except AttributeError:
        return None

def on_press(key):
    ch = ch_of(key)
    if ch in 'wasd':
        with lock:
            held.add(ch)
    elif key == keyboard.Key.esc:
        listener.stop()

def on_release(key):
    ch = ch_of(key)
    if ch in 'wasd':
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
```

- [ ] **Step 3: Проверка на столе** (колёса в воздухе, USB для телеметрии вынут, питание от батареи; либо BT+USB вместе — см. Task 3 Step 4)

Run: `python tools/wasd.py --port COMx`
Expected: зажатие W — колёса вперёд и `d=` растёт (если слушать через USB), отпускание — всё к нулю за ~1 с

- [ ] **Step 4: Коммит**

```bash
git add tools/wasd.py
git commit -m "tools: wasd.py — руление с клавиатуры по BT COM"
```

---

### Task 5: Полевой тест + итоги в STATUS.md

**Files:**
- Modify: `STATUS.md`

- [ ] **Step 1: Напольный тест**: батарея, BT без USB; `drv 1.5` — направление вперёд (перепутано -> `drv -1.5`); `drv 2.5`; A/D на месте и в движении; толчок во время езды — робот не падает и доезжает
- [ ] **Step 2: Watchdog-тест**: на ходу закрыть wasd.py (или вырубить BT) — робот останавливается сам за ~1 с
- [ ] **Step 3: Записать в STATUS.md** секцию «WASD/BT»: схема, COM-номер, подобранные drv/trn, известные особенности (поворот при движении назад может инвертироваться — эмпирия; HC-05 обесточивать при прошивке)
- [ ] **Step 4: Коммит**

```bash
git add STATUS.md
git commit -m "док: полевой тест WASD/BT, подобранные параметры"
```

---

## Self-Review

- Спека покрыта: модель управления (T1-T2), протокол/watchdog (T1), HC-05 (T3), клиент (T4), тест-план (T5). Открытые пункты спеки («на потом») — осознанно вне плана.
- Placeholders: нет (все шаги с кодом/командами).
- Типы/имена: drvTarget/drvNow/trnTarget/trnNow/kw-ks-ka-kd/cmdDeadline согласованы между T1, T2 и speed.h/motors.h (trnTarget extern в config.h — добавить в T1 вместе с drv/trn: строка `extern float drv, trn, trnTarget;` — учесть: trnTarget используется в speed.h Task 2 Step 3!). Исправлено ниже в задаче T1 (добавить `extern float trnTarget;` в config.h).
