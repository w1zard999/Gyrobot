# DIY-балансиры на Arduino + MPU-6050 + DC-моторы: проверенные проекты и уроки (research notes, 2026-09-28)

Контекст нашего робота: Nano, MPU-6050, TB6612 на 3.9 кГц, JGA25-370 45:1, 11 CPR (495 имп/об колеса), колеса 63 мм, 2S, HC-05. Центр масс ~18 мм над осью, неустойчивый полюс ~19 рад/с (удвоение наклона за 36 мс), мертвая зона PWM ~12–16/255.

## Q1. Какие проекты доказанно стоят и ездят по пульту, и как устроен Balanduino

### Takeaway
Самый полный и проверенный DC+энкодер эталон — TKJ Electronics Balanduino (открытый код, коммерческий кит, последний коммит в январе 2022). Архитектура: один PID по углу (Kalman), а **всё управление движением идёт через уставку угла** (restAngle): в покое — позиционная и скоростная обратная связь по энкодерам сдвигают уставку, при езде — пульт задает сдвиг уставки до ±7°. Уставка ограничена по скорости изменения (не больше 1° за итерацию). B-robot EVO2 и YABR (Brokking) — шаговые моторы, но тоже управляют через уставку угла.

### Cited Findings
**Balanduino (TKJElectronics/Balanduino, файлы `Firmware/Balanduino/{Balanduino.ino, Motor.ino, Balanduino.h, EEPROM.ino, Bluetooth.ino}`; последний коммит 7 Jan 2022)**
- Коэффициенты по умолчанию (`restoreEEPROMValues`): `P = 9.0f, I = 2.0f, D = 3.0f, targetAngle = 180.0f, backToSpot = 1, controlAngleLimit = 7, turningLimit = 25, Qangle = 0.001f, Qbias = 0.003f, Rmeasure = 0.03f` — [EEPROM.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/EEPROM.ino)
- Ядро PID (`updatePID(restAngle, offset, turning, dt)` в Motor.ino), дословно:
  ```c
  float error = restAngle - pitch;
  float pTerm = cfg.P * error;
  iTerm += cfg.I * 100.0f * error * dt;
  iTerm = constrain(iTerm, -100.0f, 100.0f);
  float dTerm = (cfg.D / 100.0f) * (error - lastError) / dt;
  ```
  Выход в процентах (0..100) → `moveMotor()` делает `setPWM(motor, speedRaw * PWMVALUE / 100)`; углы в градусах. Т.е. P=9 означает 9 % скважности на градус, D = 0.03 %/(°/с), I = 200 %/(°·с) с ограничением ±100 % — [Motor.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Motor.ino)
- Режим стоп (`steerStop`): уставка сдвигается по ошибке позиции колес с зонным расписанием коэффициентов, плюс по скорости колес:
  ```c
  if (abs(positionError) > zoneA) restAngle -= (float)positionError / positionScaleA;
  else if (abs(positionError) > zoneB) restAngle -= positionError / positionScaleB;
  else if (abs(positionError) > zoneC) restAngle -= positionError / positionScaleC;
  else restAngle -= positionError / positionScaleD;
  ...
  restAngle -= (float)wheelVelocity / velocityScaleStop;
  restAngle = constrain(restAngle, cfg.targetAngle - 10, cfg.targetAngle + 10);
  ```
  Константы (без pin-change): `zoneA = 8000, zoneB = 4000, zoneC = 1000` (с pin-change — ×2: 16000/8000/2000), `positionScaleA = 600, B = 800, C = 1000, D = 500` (×2 при pin-change: 1200/1600/2000/1000), `velocityScaleMove = 70, velocityScaleStop = 60, velocityScaleTurning = 70` (×2) — [Balanduino.h](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Balanduino.h) (значения для варианта без pin-change выведены из ×2-формы в коде `8000 * 2`, `70.0f * 2.0f`)
- Режим езды: 
  ```c
  if ((offset > 0 && wheelVelocity < 0) || (offset < 0 && wheelVelocity > 0) || offset == 0)
    offset += (float)wheelVelocity / velocityScaleMove;
  restAngle -= offset;
  ```
  затем для всех режимов: `restAngle = constrain(restAngle, lastRestAngle - 1, lastRestAngle + 1);` — ограничение скорости изменения уставки 1° за итерацию. — [Motor.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Motor.ino)
- Пульт: `targetOffset = scale(stick, 0, 1, 0, cfg.controlAngleLimit)` (пропорционально стику до 7°), `turningOffset = scale(..., cfg.turningLimit)`; при отпускании `steer(stop)` → `steerStop = true`, при первом входе в стоп `targetPosition = getWheelsPosition(); stopped = false;`. — [Bluetooth.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Bluetooth.ino)
- Поворот ослабляется со скоростью: `turning -= abs(wheelVelocity / velocityScaleTurning)` (не меняя знак); `PIDLeft = PIDValue + turning; PIDRight = PIDValue - turning;` + `leftMotorScaler/rightMotorScaler` для разных моторов. — [Motor.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Motor.ino)
- Скорость колес считается раз в 100 мс как разность позиций: `if (timer - encoderTimer >= 100) { wheelVelocity = wheelPosition - lastWheelPosition; ... if (abs(wheelVelocity) <= 40 && !stopped) { targetPosition = wheelPosition; stopped = true; } }` — т.е. точка удержания фиксируется только когда робот почти затормозил. — [Balanduino.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Balanduino.ino)
- Цикл PID крутится «как можно быстрее» (без фиксированного периода), dt меряется `micros()`; при падении за ±45° (и до подъема в ±10°) — `stopAndReset()`: `lastError = 0; iTerm = 0; targetPosition = getWheelsPosition(); lastRestAngle = cfg.targetAngle;` — [Balanduino.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Balanduino.ino), [Motor.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Motor.ino)
- ШИМ: `#define PWM_FREQUENCY 20000`, `PWMVALUE = F_CPU / PWM_FREQUENCY / 2` (Timer1 phase-correct); драйверы VNH5180 — [Balanduino.h](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Balanduino.h), [README](https://github.com/TKJElectronics/Balanduino)
- В Balanduino **нет** компенсации мертвой зоны моторов в коде (moveMotor просто масштабирует 0..100 % в PWM). — [Motor.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Motor.ino)

**B-robot EVO2 (jjrobots, шаговые — только идеи управления)**
- Каскад: `target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr)` → `control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd)`; `KP 0.32, KD 0.050, KP_THROTTLE 0.080, KI_THROTTLE 0.1`, `max_target_angle` 14° (26° pro). Выход PD интегрируется (это ускорение, а не скорость). Оценка скорости: `estimated_speed = -actual_robot_speed + angular_velocity` где `angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0` — т.е. из скорости колес вычитается вклад качания корпуса; далее фильтр `0.9*old + 0.1*new`. Цикл 100 Гц по готовности данных MPU. — [BROBOT_EVO2.ino](https://github.com/jjrobots/B-ROBOT_EVO2/blob/master/Arduino/BROBOT_EVO2/BROBOT_EVO2.ino)

**YABR (Joop Brokking, шаговые — идеи)**
- Цикл строго 4 мс (250 Гц) `loop_timer += 4000`; MPU: гиро ±250 °/с (0x1B=0x00), DLPF 0x1A=0x03 (~43 Гц); `pid_p_gain = 34, pid_i_gain = 0.90, pid_d_gain = 20`, `max_target_speed = 200`; зона нечувствительности выхода `if(pid_output < 5 && pid_output > -5) pid_output = 0`; езда — плавное изменение уставки угла по 0.05 за цикл; самоподстройка `self_balance_pid_setpoint += / -= 0.0015` только когда пульт не нажат (pid_setpoint == 0). — [siredmar/yabr копия Brokking-кода](https://github.com/siredmar/yabr/blob/master/Balancing_robot_ownPID/Balancing_robot_ownPID.ino), [brokking.net YABR](http://www.brokking.net/yabr_main.html)

**Franko (lukagabric, 2014, DC без энкодеров, L298N, Uno)**
- Только угловой PID (библиотека PID_v1) по углу DMP, `PID pid(&input,&output,&setpoint, 70, 240, 1.9, DIRECT)`, `SetSampleTime(10)`, `SetOutputLimits(-255,255)`, `#define MIN_ABS_SPEED 30` (компенсация мертвой зоны через `motorController.move(output, MIN_ABS_SPEED)`), `originalSetpoint = 174.29` (уставка подобрана руками), езда — `setpoint = originalSetpoint ± movingAngleOffset`. — [lukagabric/Franko](https://github.com/lukagabric/Franko) (последний коммит 14 Apr 2014; локально проверено)

**Прочие**
- nathannegron1/self-balancing-robot: Uno + MPU6050 + DC с энкодерами, «outer velocity-control loop that modifies the desired balance angle» — [GitHub](https://github.com/nathannegron1/self-balancing-robot)
- EduBal (ETH Zürich, 2020): DC 30:1 с энкодерами, каскад «внутренний PI скорости колеса + LQR сверху», LQR `K = (−0.1, −2.04, −90.23, −14.97)` по (φ, φ̇, θ, θ̇) — [arXiv 2005.09304](https://arxiv.org/pdf/2005.09304)

### Inferences
- Balanduino по сути реализует стратегию (a): «скорость/позиция → уставка угла → PID угла». Причем I-член угла у него тоже есть (I=2), но драйв и стояние на месте обеспечивает позиционная петля, а не адаптация нуля.
- Масштаб Balanduino для нашего железа: у них скорость = отсчеты за 100 мс, а разрешение прописано в комментарии Balanduino.h: «One resolution is 928 pulses per encoder» (без pin-change) / «1856 pulses» (с pin-change) — [Balanduino.h](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Balanduino.h). У нас 495 имп/об (≈0.4 мм на импульс при 63 мм), т.е. примерно в 1.9 раза меньше, чем в варианте 928. Значит zone*/positionScale*/velocityScale* для нас надо делить примерно на 1.9 (плюс поправка на диаметр колес), а не копировать.
- P=9 %/° в Balanduino ≈ 23 PWM-единиц/° на 8-битной шкале; D=0.03 %/(°/с) ≈ 0.077 PWM/(°/с). Для нашего низкого ЦМ нужна своя подстройка, но это порядок величин для DC-робота ~1 кг.

### Gaps
- Модель моторов Balanduino точно не подтверждена (разрешение 928/1856 имп/об — из кода); не найдено измерение реальной частоты цикла (free-running; обычно называют ~ сотни Гц, но источника нет).
- Шаги remote-drive у nathannegron не прочитаны в коде (только README).

## Q2. Две стратегии езды: (a) скорость → целевой угол (каскад) vs (b) PWM скорости параллельно (китайская схема)

### Takeaway
Обе рабочие и проверены массово. (a) — Balanduino/B-robot/YABR/EduBal: движение задается наклоном, робот «сам себя» разгоняет, рывков нет, если уставка угла слева ограничена по скорости. (b) — китайские «平衡小车»: PWM = 直立环(PD) + 速度环(PI) + 转向环; скоростная петля имеет **положительный** знак, фильтр 0.7/0.3 на скорость и сильно ограниченный интеграл; пульт вводится как смещение цели скорости/интеграла, а не как сырой PWM. Прямое добавление сырого PWM (как у нас) не используется ни одной из схем.

### Cited Findings
- Китайская параллельная схема: «Motor PWM = Upright Loop + Speed Loop», с поворотом «Left = Upright + Speed + Turn/2, Right = Upright + Speed − Turn/2»; скоростная петля в балансире должна давать положительную обратную связь (при правильном знаке оба колеса ускоряются в сторону движения — это и есть нужный эффект) — [сводка поиска по CSDN/miaowlabs](https://c.miaowlabs.com/E03.html), [CSDN mayuxin1314](https://blog.csdn.net/mayuxin1314/article/details/125114464) (страницы не удалось открыть напрямую — ECONNRESET/SSL; данные из поискового сниппета)
- Пример (STM32 + MPU6050 DMP, 100 Гц, PWM 10 кГц): upright PD `Kp = -420, Kd = -2000`, speed PI `VKp = +190, VKi = 0.95`, фильтр `filtered_velocity = 0.3*velocity + 0.7*last`, интеграл ограничен `I_xianfu(3000)`, выход ±7000; порядок настройки: сначала только upright (Kp до колебаний → Kd для гашения), потом speed loop; сброс интеграла и энкодеров при |roll|>30°. Автор прямо пишет, что без пульта. — [Luoyu132/STM32-BalanceCar](https://github.com/Luoyu132/STM32-BalanceCar)
- B-robot делает (a) и специально вычитает из скорости колес вклад угловой скорости корпуса (`estimated_speed = -actual_robot_speed + angular_velocity`), чтобы качание корпуса не выглядело как езда — [BROBOT_EVO2.ino](https://github.com/jjrobots/B-ROBOT_EVO2/blob/master/Arduino/BROBOT_EVO2/BROBOT_EVO2.ino)
- Balanduino (a): уставка = targetAngle − offset(пульт, до 7°) − velocity/scale, и уставка ограничена ±1°/итерацию, ±10° от нуля в стопе — [Motor.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Motor.ino)
- EduBal: внутренний PI по скорости колеса (полюса с ts=0.5 с, f≤30 1/с) введен специально, чтобы «уменьшить полосу актуатора и не возбуждать критичные высокие частоты» и бороться с люфтом — [arXiv 2005.09304](https://arxiv.org/pdf/2005.09304)

### Inferences
- Для DC-моторов (a) и (b) математически близки (PI скорости, умноженный на Kp угла, эквивалентен добавке в PWM), но (a) имеет физически осмысленное ограничение — лимит наклона (7° / 14°) и лимит скорости изменения уставки, — что автоматически убирает рывки при старте езды. В (b) то же приходится делать ограничением интеграла и плавным изменением цели скорости.
- Для нашего быстрого полюса (19 рад/с) важно, что внешняя петля медленная (Balanduino: скорость раз в 100 мс; китайцы: фильтр 0.7/0.3; B-robot: 0.9/0.1). Внешняя петля должна быть в 5–10 раз медленнее внутренней, иначе она «дерется» с балансом. Рекомендация: (a) по образцу Balanduino/B-robot с команд. скорости с клавиатуры → PI скорости → целевой угол с ограничением амплитуды (несколько градусов) и скорости изменения.
- Причина рывка при «добавлении сырого PWM»: добавка PWM без изменения уставки угла воспринимается угловым PID как возмущение — он сначала гасит его (рывок в обратную сторону), и только потом робот наклоняется. В (a) робот сначала наклоняется вперед, потом едет — это физически правильный порядок (неминимально-фазовая система).

### Gaps
- Прямые цитаты из miaowlabs/CSDN (Ki = Kp/200 и т.п.) не получены — сайты не открылись.

## Q3. Дрейф точки баланса: внешняя петля (интеграл скорости/позиции) vs адаптивная уставка

### Takeaway
Проверенные DC-проекты с энкодерами (Balanduino, китайские машинки, EduBal, nathannegron) убирают дрейф ноля через энкодерную петлю: интеграл скорости = позиция. Смещение ЦМ тогда компенсируется автоматически — робот стоит на месте при любом нуле ±несколько градусов. «Адаптивный ноль по выходу PID» — это прием из шаговых проектов (YABR), где выход PID = скорость колес; для DC он хрупок.

### Cited Findings
- Balanduino: в стопе уставка угла корректируется по позиции колес (зонные коэффициенты) и скорости, ограничена ±10°; `targetPosition` фиксируется, когда `|wheelVelocity| <= 40`; при подъеме/падении `stopAndReset()` обнуляет iTerm, lastError и ставит `lastRestAngle = cfg.targetAngle` — [Motor.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Motor.ino), [Balanduino.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Balanduino.ino)
- Китайская схема: дрейф ликвидирует интеграл скоростной петли (накопление скорости), ограниченный `I_xianfu(3000)`, сбрасываемый при падении — [Luoyu132/STM32-BalanceCar](https://github.com/Luoyu132/STM32-BalanceCar)
- YABR: `self_balance_pid_setpoint` двигается на ±0.0015 за цикл 4 мс по знаку pid_output, только когда пульт отпущен; при этом выход PID для шаговиков — это скорость вращения (период шагов) — [yabr code](https://github.com/siredmar/yabr/blob/master/Balancing_robot_ownPID/Balancing_robot_ownPID.ino)
- Franko без энкодеров: уставка `174.29` подобрана вручную, адаптации нет — [Franko.ino](https://github.com/lukagabric/Franko)

### Inferences
- Почему адаптивный ноль хрупок у нас: (1) у DC-моторов выход PID ≈ напряжение/момент, а не скорость; ненулевой средний PWM может быть из-за трения, мертвой зоны, разницы моторов, разряда батареи — адаптация «чинит» не ту причину; (2) порог |PID|>0.7 и скорость дрейфа образуют еще один интегратор, не согласованный с I-членом угла → два интегратора на одну ошибку → медленные колебания/дрейф; (3) без сброса при повторном взводе накопленное смещение остается (мы и видим дрейф после re-arm); (4) адаптация не знает про позицию, поэтому робот медленно уезжает.
- Если оставить адаптацию, минимум — делать ее только по энкодерной скорости (как YABR фактически делает для шаговиков), с медленной постоянной, с ограничением, и сбрасывать при взводе. Лучше заменить на Balanduino-подобную позиционную+скоростную петлю в уставку.

### Gaps
- Не найдено публикаций, прямо сравнивающих адаптивную уставку с энкодерной петлей на DC-роботах.

## Q4. Фильтры и тайминг: Kalman vs комплементарный vs DMP; частота цикла; DLPF; диапазон гиро

### Takeaway
Сходятся на: гиро ±250 °/с, DLPF 0x03 (~42–44 Гц), цикл 100–250 Гц. Balanduino — TKJ Kalman (Q_angle 0.001, Q_bias 0.003, R_measure 0.03) с free-running циклом; YABR — комплементарный с жестким 250 Гц; Franko и китайцы — DMP 100 Гц. Для быстрого полюса важно суммарное запаздывание, а не тип фильтра.

### Cited Findings
- Balanduino setup: `i2cBuffer[0] = 1; // sample rate 500Hz`, `i2cBuffer[1] = 0x03; // 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling`, `i2cBuffer[2] = 0x00; // Gyro ±250deg/s`, `i2cBuffer[3] = 0x00; // Acc ±2g`; угол: `pitch = kalman.getAngle(accAngle, gyroRate, dt)`, `gyroRate = (gyroX - gyroXzero) / 131.0f` — [Balanduino.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Balanduino.ino)
- Калибровка гиро в Balanduino: 25 выборок, проверка неподвижности (разброс < 2000 LSB) через `checkMinMax()` — [Tools.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Tools.ino)
- YABR: 250 Гц фиксированно (`loop_timer += 4000`), ±250 °/с, DLPF 0x03 — [yabr code](https://github.com/siredmar/yabr/blob/master/Balancing_robot_ownPID/Balancing_robot_ownPID.ino)
- B-robot EVO2: 100 Гц по новым данным MPU — [BROBOT_EVO2.ino](https://github.com/jjrobots/B-ROBOT_EVO2/blob/master/Arduino/BROBOT_EVO2/BROBOT_EVO2.ino)
- Franko: DMP + PID SampleTime 10 мс — [Franko](https://github.com/lukagabric/Franko); STM32 машинка: DMP 100 Гц — [Luoyu132](https://github.com/Luoyu132/STM32-BalanceCar)
- EduBal предлагает студентам и комплементарный, и Kalman — [arXiv 2005.09304](https://arxiv.org/pdf/2005.09304)
- Задержки DLPF MPU-6050 (регистр 0x1A, гиро): CFG=0: 256 Гц/0.98 мс; 1: 188 Гц/1.9 мс; 2: 98 Гц/2.8 мс; 3: 42 Гц/4.8 мс; 4: 20 Гц/8.3 мс; 5: 10 Гц/13.4 мс; 6: 5 Гц/18.6 мс — [MPU-6000/6050 Register Map, reg 26](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) (значения по памяти из этого документа, проверить по PDF)

### Inferences
- При удвоении наклона за 36 мс DLPF 20 Гц (8.3 мс) + DMP (фиксированные 100 Гц, свое сглаживание) — заметная доля запаса по фазе. Рекомендация: DLPF 0x03 (42 Гц) или даже 0x02 (98 Гц), цикл 200–250 Гц жестко по micros (как YABR), D-член брать прямо от гиро (rate), а не дифференцировать угол — меньше шума и задержки.
- Тип фильтра вторичен: Kalman TKJ и комплементарный с α≈0.98–0.996 дают почти одинаковый угол; DMP удобен, но фиксирует 100–200 Гц и добавляет задержку FIFO.

### Gaps
- Нет измерений реальной частоты цикла Balanduino.

## Q5. Моторы: мертвая зона, частота ШИМ, люфт и джиттер

### Takeaway
Люфт редуктора — признанная главная причина мелких колебаний (limit cycle) у DC-балансиров (подтверждено даже в ETH EduBal). Лечат: внутренним PI по скорости колеса (EduBal), снижением Kp и достаточным Kd, D от гироскопа, компенсацией мертвой зоны небольшим фиксированным смещением (Franko MIN_ABS_SPEED=30) или зоной нечувствительности выхода (YABR ±5). Balanduino компенсацию мертвой зоны не делает, ШИМ 20 кГц.

### Cited Findings
- EduBal: «the motor gearbox showed significant backlash. This nonlinearity could not directly be incorporated into the linear model and complicated stabilization control. In order to mitigate this effect, we designed an underlying feedback proportional-integral controller (PI) for the wheel velocity»; мотор идентифицирован как `ω/U = 2.6/(0.038 s + 1)` (τ=38 мс); замкнутая скоростная петля ~ `K=1, tEM=0.0994 s`; остаточные мелкие колебания θ «can be attributed to the gearbox backlash» — [arXiv 2005.09304](https://arxiv.org/pdf/2005.09304)
- Советы по настройке: сначала только P до колебаний, «large enough for movement but not so large that movement becomes jerky», Kd начать с ~1 % от Kp и увеличивать до исчезновения колебаний; слишком малый Kd — «shake» — [Instructables PID](https://www.instructables.com/Self-Balancing-Robot-PID-Control-Algorithm/)
- Малые DC-моторы: асимметрия вперед/назад и разброс моторов на малом моменте; люфт требует учета — [поисковый сниппет Hackaday 2021](https://hackaday.com/2021/09/20/self-balancing-robot-needs-a-little-work/)
- Franko: `MIN_ABS_SPEED 30` для L298N (мертвая зона) — [Franko](https://github.com/lukagabric/Franko); YABR: выход |pid|<5 → 0 — [yabr](https://github.com/siredmar/yabr/blob/master/Balancing_robot_ownPID/Balancing_robot_ownPID.ino)
- Balanduino: `leftMotorScaler/rightMotorScaler` для выравнивания моторов, ШИМ 20 кГц — [Motor.ino](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Motor.ino), [Balanduino.h](https://github.com/TKJElectronics/Balanduino/blob/master/Firmware/Balanduino/Balanduino.h)
- Китайский пример: ШИМ 10 кГц — [Luoyu132](https://github.com/Luoyu132/STM32-BalanceCar)

### Inferences
- Наш τ мотора, вероятно, того же порядка (20–40 мс), что и 1/19 рад/с = 53 мс; то есть актуатор ненамного быстрее неустойчивого полюса — это делает робот требовательным к запаздыванию и объясняет джиттер. Компенсация мертвой зоны у нас должна быть чуть меньше измеренной (например 10–12 при 12–16), иначе возникает релейный limit cycle вокруг нуля.
- 3.9 кГц на TB6612 допустимо (слышно, но на малых скважностях мотор ведет себя линейнее, чем на 20 кГц из-за индуктивности); переходить на 20 кГц не обязательно — это не главная причина джиттера.
- Люфт при низком ЦМ: мертвый ход колеса ±Δ° почти напрямую становится ошибкой положения корпуса относительно колес, и с большим Kp угол-петля «звенит» в пределах люфта. Лечение — меньше Kp, больше D от гиро, скоростная внутренняя петля.

### Gaps
- Измерений люфта JGA25-370 45:1 не найдено.

## Q6. Влияние очень низкого центра масс

### Takeaway
Для перевернутого маятника темп расходимости ≈ √(g/ℓ): чем ниже ЦМ, тем быстрее падает и тем сложнее стабилизировать с учетом задержек. Проверенные проекты специально поднимают батарею (EduBal: «purposely placed high above the ground ... to raise the center of mass»). Рекомендация — поднять батарею/массу.

### Cited Findings
- «the acceleration is inversely proportional to the length. Tall pendulums fall more slowly than short ones»; ω_p = √(g/ℓ); пример с метлой на пальце — [Wikipedia: Inverted pendulum](https://en.wikipedia.org/wiki/Inverted_pendulum)
- EduBal (0.99 кг, 235 мм высоты): «The 9.9 V 2100 mAh ... battery is purposely placed high above the ground in a cage in order to raise the center of mass» — [arXiv 2005.09304](https://arxiv.org/pdf/2005.09304)
- Противоречие: один AI-сниппет поиска утверждал, что низкий ЦМ «increases passive stability (slower divergence)» — это противоречит физике и первоисточникам выше; не использовать — [Quora (поисковый сниппет)](https://www.quora.com/What-effect-would-the-structural-height-of-a-2-wheeled-self-balancing-robot-have-on-controlling-it)

### Inferences
- Наш полюс 19 рад/с ↔ эффективная длина ℓ_eff = g/19² ≈ 27 мм. Подняв ℓ_eff до 60 мм: √(9.81/0.06) ≈ 12.8 рад/с (удвоение ~54 мс); до 100 мм: ≈ 9.9 рад/с (удвоение ~70 мс); до 150 мм: ≈ 8.1 рад/с (~86 мс). Т.е. подъем батареи на 5–10 см дает в 1.5–2 раза больше времени на реакцию и снижает чувствительность к люфту и задержкам DLPF/цикла.
- Цена: нужен больший момент для той же коррекции (m·g·ℓ·θ растет с ℓ), но 45:1 редуктор это покрывает; также наклон при езде дает большее ускорение на градус — лимит целевого угла (Balanduino 7°) нужно будет уменьшить.

### Gaps
- Нет публичного количественного эксперимента «один робот, разные высоты ЦМ» на DC-моторах.

## Итоговая рекомендация (выводы для синтеза)
- Эталон: Balanduino-структура на нашем железе: жесткий цикл 200–250 Гц (YABR), ±250 °/с, DLPF 0x03, Kalman TKJ или комплементарный; PID угла с D от гиро; уставка = ноль − сдвиг_от_пульта − (позиция/скорость энкодеров), с ограничением амплитуды (±5–7°) и скорости изменения уставки; скорость колес по окну 50–100 мс с фильтром; сброс всех интеграторов и targetPosition при взводе. Удалить «адаптивный ноль» и сырое добавление PWM. Поднять батарею. — выводы на основе источников выше.
