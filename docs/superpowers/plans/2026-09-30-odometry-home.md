# Одометрия, карта и возврат домой — план реализации

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** робот ведёт одометрию, клиент рисует карту пути, по кнопке H робот
сам возвращается домой по прямой и встаёт в исходное направление.

**Architecture:** одометрия и автомат возврата — в прошивке
`gyrobot_tumbller.ino` (40-мс блок скорости); автомат выдаёт те же цели
`moveSet`/`turnSet`, что клавиши, поэтому баланс, рампы и поворот на месте
работают без изменений. Клиент `tools/wasd.py` получает `px/py/ph/nav` в
телеметрии и рисует карту; чистая логика карты — в `tools/wasdmap.py` с
юнит-тестами.

**Tech Stack:** Arduino Nano (arduino-cli, `arduino:avr:nano`), Python 3 +
pygame + pyserial, unittest.

**Spec:** `docs/superpowers/specs/2026-09-30-odometry-home-design.md`

## Global Constraints

- Возврат только по прямой: разворот к дому → езда → разворот в исходный курс.
- Курс — по гироскопу Z, путь — по энкодерам (1 имп ≈ 0.40 мм).
- Дом: автоматически при взводе; кнопка R (`home`) — текущие положение и курс.
- Отмена возврата: любая W/A/S/D, команда `x` (клиент шлёт при выходе), падение.
- Допуски: разворот к дому < 10°, прибытие < 5 см, финальный курс < 5°.
- Параметры навигации живые и сохраняются командой `save`.
- Однобуквенные команды `w s a d` заняты WASD; новые: `h`, `x`, `home`.
- Без строк `Co-Authored-By` в коммитах.

## Review Focus

- Команда `h`, когда робот уже дома (< 5 см): не ездить, только довернуть курс.
- Проскок дома во время езды (дом оказался сзади): не разворачиваться
  бесконечно — перейти к финальному развороту.
- Нажатие W/A/S/D посреди возврата: возврат сразу отменяется, управление у человека.
- Падение посреди возврата: автомат сбрасывается, после взвода — новый дом.
- Курс через ±180° (робот развернулся задом к дому): ошибка курса считается
  по кратчайшему углу, без рывка на 360°.

---

### Task 1: Одометрия в прошивке

**Files:**
- Modify: `firmware/gyrobot_tumbller/gyrobot_tumbller.ino`

**Interfaces:**
- Produces: глобальные `float odoX, odoY` (мм), `float odoTh` (° , + влево,
  −180…180); `void odoReset()`; `float wrap180(float)`; команда `home`;
  телеметрия `px=<см> py=<см> ph=<°>`.

- [ ] **Step 1: Глобальные и функции** — после блока «Состояние»:

```cpp
// ---------- Одометрия ----------
// Путь по энкодерам, курс по гироскопу Z (колёса на поворотах проскальзывают).
// Дом — (0,0), курс 0: ставится при взводе и командой home.
#define MM_PER_CNT 0.3998f            // π·63 мм / 495 имп; уточнить по рулетке
float odoX = 0, odoY = 0;             // мм: x — вперёд от дома, y — влево
float odoTh = 0;                      // курс, °, + влево, −180…180

float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void odoReset() { odoX = odoY = odoTh = 0; }
```

- [ ] **Step 2: Курс каждые 5 мс** — в `controlTick()` сразу после строки
  `heading += yawR * dt;`:

```cpp
  odoTh = wrap180(odoTh + yawR * dt);
```

- [ ] **Step 3: Путь каждые 40 мс** — в блоке скорости после
  `vF = 0.7f * vF + 0.3f * v;`:

```cpp
    float ds = (cA + cB) * 0.5f * MM_PER_CNT;          // мм за 40 мс, + вперёд
    odoX += ds * cos(odoTh * 0.0174533f);
    odoY += ds * sin(odoTh * 0.0174533f);
```

- [ ] **Step 4: Сброс при взводе** — в `resetLoops()` добавить `odoReset();`
  (вызывается при взводе и падении; новый взвод = новый дом).

- [ ] **Step 5: Команда `home`** — в `applyLine()` рядом с `save`:

```cpp
  if (!strcmp(line, "home")) { odoReset(); Serial.println(F("дом здесь")); return; }
```

- [ ] **Step 6: Телеметрия** — в `telemetry()` перед `hz=`:

```cpp
  Serial.print(F(" px=")); Serial.print(odoX * 0.1f, 0);
  Serial.print(F(" py=")); Serial.print(odoY * 0.1f, 0);
  Serial.print(F(" ph=")); Serial.print(odoTh, 0);
```

- [ ] **Step 7: Компиляция**
  Run: `arduino-cli compile -b arduino:avr:nano firmware/gyrobot_tumbller`
  Expected: без ошибок, ОЗУ < 60 %.

- [ ] **Step 8: Проверка на роботе** — залить (HC-05 вынуть), робот стоит:
  `px/py/ph` ≈ 0. Проехать W ≈ 1 м по рулетке: `px` ≈ 100 (±10); иначе
  поправить `MM_PER_CNT` пропорционально. Развернуть A на ~360°: `ph`
  возвращается к ≈ 0 (±10). `home` обнуляет.

- [ ] **Step 9: Commit** `git commit -am "odo: одометрия (энкодеры + гироскоп), команда home, px/py/ph в телеметрии"`

---

### Task 2: Автомат возврата домой

**Files:**
- Modify: `firmware/gyrobot_tumbller/gyrobot_tumbller.ino`

**Interfaces:**
- Consumes: `odoX, odoY, odoTh, wrap180()` (Task 1); `moveSet, turnSet,
  MOVE, YMAX`, `updateTargets()`.
- Produces: `uint8_t nav` (0 IDLE, 1 TURN, 2 DRIVE, 3 FACE); команды `h`, `x`;
  параметры `knt knd ntol` (в `save`); телеметрия `nav=`.

- [ ] **Step 1: Параметры и состояние** — рядом с параметрами поворота:

```cpp
// Возврат домой по прямой: TURN — к дому, DRIVE — ехать, FACE — в исходный курс
enum { NAV_IDLE, NAV_TURN, NAV_DRIVE, NAV_FACE };
uint8_t nav = NAV_IDLE;
float KNT  = 2.0f;    // °/с поворота на ° ошибки курса
float KND  = 0.5f;    // имп/40мс скорости на см до дома (замедление на подходе)
float NTOL = 5;       // см: дома
#define NAV_YMIN 25   // °/с — меньше тугое колесо не сдвинет
```

  Добавить `&KNT, &KND, &NTOL` в конец массива `CFG[]`.

- [ ] **Step 2: Цели автомата** — перед `updateTargets()`:

```cpp
float navTurn(float err, float lim) {         // поворот с мин. скоростью, не больше lim
  float y = constrain(KNT * err, -lim, lim);
  if (fabs(y) < NAV_YMIN) y = (err >= 0) ? NAV_YMIN : -NAV_YMIN;
  return y;
}

// Раз в 40 мс: цели скорости (имп/40мс) и поворота (°/с) для возврата
void navTargets(float& mt, float& yt) {
  mt = 0; yt = 0;
  float dx = -odoX * 0.1f, dy = -odoY * 0.1f;           // см до дома
  float dist = sqrt(dx * dx + dy * dy);
  float err = wrap180(atan2(dy, dx) * 57.2958f - odoTh); // куда повернуть к дому
  if (nav == NAV_TURN) {
    if (dist < NTOL) nav = NAV_FACE;
    else if (fabs(err) < 10) nav = NAV_DRIVE;
    else yt = navTurn(err, YMAX);
  }
  if (nav == NAV_DRIVE) {
    if (dist < NTOL || fabs(err) > 90) nav = NAV_FACE;    // дома или проскочили
    else if (fabs(err) > 45) nav = NAV_TURN;              // сильно сбились — довернуть
    else {
      mt = constrain(KND * dist, 4.0f, MOVE);
      yt = constrain(KNT * err, -40.0f, 40.0f);           // подруливание на ходу
    }
  }
  if (nav == NAV_FACE && moveSet == 0) {                  // довернуть, когда встал
    float e = wrap180(-odoTh);
    if (fabs(e) < 5) nav = NAV_IDLE;
    else yt = navTurn(e, YMAX);
  }
}
```

- [ ] **Step 3: Встроить в `updateTargets()`** — заменить начало функции до
  рампы `moveSet` на:

```cpp
  int fwd = (int)held(tW) - (int)held(tS);
  int lr  = (int)held(tA) - (int)held(tD);
  if (fwd || lr) nav = NAV_IDLE;                          // человек перехватил управление
  float mt, yt;
  if (nav) navTargets(mt, yt);
  else { mt = fwd * MOVE; yt = lr * YMAX; }
  int8_t drv = fwd ? fwd : (nav == NAV_DRIVE ? 1 : 0);   // езда: клавишей или автоматом
  if (drv && !lastFwd && !stopping) posRest = posI;     // начало езды: запомнить стоянку
  if (drv) stopping = false;
  else if (lastFwd) stopping = true;                    // конец езды
  lastFwd = drv;
```

  и поворот:

```cpp
  if (yt == 0) turnSet = 0;                              // отпустил — сразу ноль, тормозит KT
  else turnSet += constrain(yt - turnSet, -15.0f, 15.0f);
```

  (строка `float mt = fwd * MOVE;` и `float yt = lr * YMAX;` удаляются.)

- [ ] **Step 4: Сброс** — в `resetLoops()` добавить `nav = NAV_IDLE;`.

- [ ] **Step 5: Команды** — в `applyLine()` до проверки WASD:

```cpp
  if (!strcmp(line, "h")) { if (armed) nav = NAV_TURN; return; }
  if (!strcmp(line, "x")) { nav = NAV_IDLE; return; }
```

  параметры (в блоке `else if`):

```cpp
  else if (!strcmp(line, "knt") && v > 0) KNT = v;
  else if (!strcmp(line, "knd") && v > 0) KND = v;
  else if (!strcmp(line, "ntol") && v > 0) NTOL = v;
```

  в `printParams()`: ` knt= knd= ntol=`; в `telemetry()`: ` nav=<nav>` **сразу после `ph=`**
  (клиент разбирает `px= py= ph= nav=` одной группой).

- [ ] **Step 6: Компиляция** — как в Task 1.

- [ ] **Step 7: Проверка на роботе (по talk.py на COM13)**
  - `h` дома: сразу FACE→IDLE, робот не едет.
  - Отъехать W ~50 см, `h`: разворот ~180°, езда, стоп в < 10 см, разворот
    в исходный курс.
  - Посреди возврата нажать W: `nav=0`, управление у человека.
  - Повернуть на ~170°, отъехать, `h`: разворот по кратчайшему углу.

- [ ] **Step 8: Commit** `git commit -am "nav: возврат домой по прямой (TURN/DRIVE/FACE), команды h/x, knt/knd/ntol"`

---

### Task 3: Логика карты в клиенте (чистый модуль + тесты)

**Files:**
- Create: `tools/wasdmap.py`
- Create: `tools/test_wasdmap.py`

**Interfaces:**
- Produces: `parse_pose(line: str) -> tuple | None` → `(x_cm, y_cm, h_deg, nav)`;
  `class Trail(min_step=1.0, cap=3000)` с `add(x, y)`, `clear()`, `points`;
  `class MapView(size_px, min_extent=50)` с `fit(points, robot_xy)` и
  `to_screen(x, y) -> (px, py)` (вперёд = вверх, влево = влево, дом в центре).

- [ ] **Step 1: Тесты (падают)** — `tools/test_wasdmap.py`:

```python
import unittest
from wasdmap import parse_pose, Trail, MapView


class T(unittest.TestCase):
    def test_parse(self):
        line = 'A a=0.1 r=2 v=0.0 vA=0 vB=0 x=5 m=0 yr=0 h=0.0 uL=1 uR=1 px=12 py=-3 ph=-170 nav=2 hz=200 ie=0'
        self.assertEqual(parse_pose(line), (12.0, -3.0, -170.0, 2))
        self.assertIsNone(parse_pose('- a=1 r=0 v=0'))

    def test_trail_thins_and_caps(self):
        t = Trail(min_step=1.0, cap=3)
        for x in (0, 0.5, 2, 4, 6):
            t.add(x, 0)
        self.assertEqual(t.points, [(2, 0), (4, 0), (6, 0)])
        t.clear()
        self.assertEqual(t.points, [])

    def test_view_axes(self):
        m = MapView(200, min_extent=50)
        m.fit([], (0, 0))
        cx, cy = m.to_screen(0, 0)
        fx, fy = m.to_screen(10, 0)     # вперёд — вверх
        lx, ly = m.to_screen(0, 10)     # влево — влево
        self.assertEqual((cx, cy), (100, 100))
        self.assertTrue(fy < cy and fx == cx)
        self.assertTrue(lx < cx and ly == cy)

    def test_view_fits_far_point(self):
        m = MapView(200, min_extent=50)
        m.fit([(300, 0)], (0, 0))
        px, py = m.to_screen(300, 0)
        self.assertTrue(0 <= py <= 200)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: Запуск — падает**
  Run: `cd tools && python -m unittest test_wasdmap -v`
  Expected: `ModuleNotFoundError: No module named 'wasdmap'`

- [ ] **Step 3: Реализация** — `tools/wasdmap.py`:

```python
"""Карта для клиента WASD: разбор позы из телеметрии, след, пересчёт в экран."""
import re

POSE = re.compile(r' px=(-?\d+) py=(-?\d+) ph=(-?\d+) nav=(\d)')


def parse_pose(line):
    m = POSE.search(line)
    return (float(m[1]), float(m[2]), float(m[3]), int(m[4])) if m else None


class Trail:
    """След робота: точка добавляется, если отошли на min_step см; не больше cap."""
    def __init__(self, min_step=1.0, cap=3000):
        self.min_step, self.cap, self.points = min_step, cap, []

    def add(self, x, y):
        if self.points:
            lx, ly = self.points[-1]
            if (x - lx) ** 2 + (y - ly) ** 2 < self.min_step ** 2:
                return
        self.points.append((x, y))
        if len(self.points) > self.cap:
            del self.points[0]

    def clear(self):
        self.points = []


class MapView:
    """Квадрат size_px, дом в центре; вперёд (x) — вверх, влево (y) — влево."""
    def __init__(self, size_px, min_extent=50):
        self.size, self.min_extent, self.extent = size_px, min_extent, min_extent

    def fit(self, points, robot_xy):
        far = max([abs(c) for p in list(points) + [robot_xy] for c in p] + [0])
        self.extent = max(self.min_extent, far * 1.2)

    @property
    def scale(self):                      # пикселей на см
        return (self.size / 2) / self.extent

    def to_screen(self, x, y):
        c = self.size / 2
        return round(c - y * self.scale), round(c - x * self.scale)
```

- [ ] **Step 4: Тесты проходят**
  Run: `cd tools && python -m unittest test_wasdmap -v`
  Expected: 4 tests OK.

- [ ] **Step 5: Commit** `git add tools/wasdmap.py tools/test_wasdmap.py && git commit -m "wasd: модуль карты (поза, след, пересчёт в экран) с тестами"`

---

### Task 4: Карта и клавиши в клиенте

**Files:**
- Modify: `tools/wasd.py`

**Interfaces:**
- Consumes: `parse_pose, Trail, MapView` (Task 3); команды робота `h`, `x`,
  `home` (Task 1–2).

- [ ] **Step 1: Приём позы** — в `Link.__init__`: `self.pose = None;
  self.trail = Trail()`; в `_reader` после разбора телеметрии:

```python
            pose = parse_pose(line)
            if pose:
                self.pose = pose
                self.trail.add(pose[0], pose[1])
```

- [ ] **Step 2: Клавиши** — физические коды: H (`pygame.KSCAN_H`), R
  (`KSCAN_R`), C (`KSCAN_C`). В обработке `KEYDOWN`:

```python
                elif ev.scancode == pygame.KSCAN_H:
                    link.send_cmd('h')
                elif ev.scancode == pygame.KSCAN_R:
                    link.send_cmd('home'); link.trail.clear()
                elif ev.scancode == pygame.KSCAN_C:
                    link.trail.clear()
```

  В `Link` метод `send_cmd(self, s)` — как `send`, но одна строка. При выходе
  (в `finally` перед закрытием порта) — `link.send_cmd('x')`.

- [ ] **Step 3: Окно и карта** — окно `900×430`; текущая панель слева без
  изменений; справа квадрат 400×400 с отступом (x=480, y=15):
  сетка 10 см (тонкая) и 50 см (толще) — если шаг ≥ 6 px; след линией;
  дом — крестик в центре; робот — треугольник по курсу `ph` (0° — вверх,
  + — против часовой). Внизу слева подсказка
  `H — домой   R — дом здесь   C — стереть след   ESC — выход`, в статусе
  `едет домой: N см` при `nav > 0` (фазы: разворот / едет / доворот).

- [ ] **Step 4: Проверка** — синтаксис (`python -c "import ast; ..."`),
  затем с роботом: след рисуется, H/R/C работают, W отменяет возврат,
  после выхода по ESC робот не продолжает возврат.

- [ ] **Step 5: Commit** `git commit -am "wasd: карта пути, H — домой, R — дом здесь, C — стереть след"`

---

### Task 5: Настройка на роботе и документация

**Files:**
- Modify: `README.md`, `docs/STATUS.md`; при необходимости дефолты в прошивке.

- [ ] **Step 1: Подбор** — 3–5 возвратов с поворотами; записать промах до
  старта и ошибку курса; при необходимости поправить `knt`, `knd`, `ntol`,
  `MM_PER_CNT`; удачное вписать в дефолты (или `save`).
- [ ] **Step 2: README** — клавиши H/R/C, как задаётся дом, точность и что
  препятствия не объезжаются; строки `knt knd ntol` в таблицу параметров.
- [ ] **Step 3: STATUS.md** — итоги замеров одометрии и возврата.
- [ ] **Step 4: Commit + push** — влить в master, `git push origin master`.
