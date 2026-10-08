// ============================================================
// Гиробот — порт схемы Elegoo Tumbller (она же Keyestudio/Yahboom/平衡小车).
// Эталон и обоснование: docs/research/Балансирующий робот Arduino эталон.md
//
// Три контура, выходы складываются:
//   u = KP*(угол-ноль) + KD*гироскоп          — вертикаль, каждые 5 мс
//     + KSP*скорость + KSI*(положение-цель)   — энкодеры, каждые 40 мс
//     ± KT*(цель_рыскания - гироскоп_Z)        — поворот, каждые 5 мс
// Ноль угла ФИКСИРОВАН. Смещение центра тяжести (груз) выбирает интеграл
// положения колёс; он обнуляется при взводе и падении. Езда W/S — сдвиг
// цели внутри этого интеграла, а не сырой ШИМ. Никакого адаптивного ноля.
//
// Знаки: угол + = наклон вперёд; u + = колёса вперёд (под падение);
// энкодеры + = вперёд; рыскание + = поворот влево.
// ============================================================
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// Камера OpenMV: её UART3 TX (P4) -> D12, RX (P5) <- D11, 9600 бод. Аппаратный UART
// занят HC-05, поэтому программный: на каждый принятый байт ~1 мс без прерываний —
// энкодеры не теряются (на нашей скорости импульс реже раза в 2.5 мс), но камера
// должна слать короткие строки, а не поток данных. Сейчас — проверка провода:
// принятые строки уходят в телеметрию как "cam> ...".
SoftwareSerial cam(12, 11);           // RX, TX

// ---------- Пины (docs/STATUS.md, прозвонено 2026-09-07) ----------
#define PWMA 9      // A = левое колесо
#define AIN1 7
#define AIN2 8
#define PWMB 10     // B = правое
#define BIN1 5
#define BIN2 4
#define STBY 6
#define PIN_ECA_A 2   // INT0, фаза B на A2
#define PIN_ECB_A 3   // INT1, фаза B на A3
#define MPU_ADDR 0x68

// ---------- MPU-6050 ----------
#define ABX 234.0f          // смещения акселя, LSB (docs/STATUS.md)
#define ABY (-14808.0f)
#define ABZ 426.0f
#define GYR_LSB 65.5f       // ±500 °/с, пишется в 0x1B
#define K_ACC 0.02f         // комплементарный фильтр: доля акселя за тик (τ≈0.25 с)

// ---------- Тайминг ----------
#define TICK_US 5000UL      // контур вертикали 200 Гц
#define SPD_TICKS 8         // контур скорости раз в 8 тиков = 40 мс
#define KEY_MS 250          // клавиша «нажата», пока буква моложе 250 мс (клиент шлёт 10 Гц)

// ---------- Живые параметры (сериал: "kp 25", "p" — показать) ----------
float KP   = 20.0f;   // ШИМ/°        (Tumbller 55; у нас 26 уже хуже)
float KD   = 0.50f;   // ШИМ/(°/с)    подбор 2026-09-28: 0.3 тряска 5 Гц, 0.7 раскачка
float KSP  = 8.0f;    // ШИМ/(имп/40мс); 4.6 (пересчёт Tumbller) — уезжал после толчка
float KSI  = 0.25f;   // ШИМ/имп
float ILIM = 2400;    // потолок интеграла положения, имп
float KT   = 0.5f;    // ШИМ/(°/с) демпфер рыскания (Tumbller kd_turn; 1.0 — «танец» стоя)
float TFF  = 0.3f;    // прямая подача поворота, ШИМ на °/с цели (Tumbller: 80 ШИМ при повороте)
float KH   = 1.0f;    // удержание курса, ШИМ/° (0.5 — 10° ошибки = 5 ШИМ, в мёртвой зоне)
float YMAX = 90;      // цель поворота при A/D, °/с
float MOVE = 15;      // цель езды при W/S, имп/40мс (1 ≈ 10 мм/с)
float AZ   = -10.5f;  // ноль угла: при нём интеграл положения стоя ≈ 0. С камерой OpenMV
                      // (центр тяжести ушёл назад); без камеры было −2.8
float DBA  = 6, DBB = 4;     // компенсация мёртвой зоны (измерено 16/12; 12/9 дёргало, 3/2 хуже)
float FALL = 30;      // угол отключения моторов
int8_t YS  = 1;       // знак гироскопа Z (проверка: поворот рукой влево → yr > 0)
float RAMP_UP = 0.7f, RAMP_DN = 0.45f; // рампа цели езды, имп/40мс за тик: цель не должна
                             // убегать от робота (при 2 отставал на 16 см и проскакивал стоп);
                             // торможение мягче (0.7 — качка 3 Гц на остановке после S)
float BLEED = 20;     // после отпускания W/S интеграл положения возвращается к значению до
                      // езды, имп за 40 мс: робот «забывает» отставание от цели и тормозит
                      // там, где отпустили (приём jfboy). 0 — выключено; 40 — рывок и тряска
// Поворот на месте (A/D без W/S). Подбор 2026-09-28, история — docs/STATUS.md. Отставало
// колесо, крутящееся назад (левое на полу туже: слева HC-05). Решение: PI скорости
// колёс по периоду импульсов (200 Гц), только по РАЗНОСТИ колёс — общей скоростью
// владеет баланс (прямое управление ей = «неверный» знак, разгон через ~1 с);
// прямая подача трения левого; усиленный демпфер KSP·v (иначе центр качается — восьмёрка).
float KW   = 4.0f;    // П: ШИМ на имп/40мс ошибки разности колёс; 8 — рывки
float KWI  = 0.5f;    // И: ШИМ за 40 мс на имп — П оставляет тугое колесо стоять, интеграл докручивает
#define WI_LIM 80     // потолок интегральной добавки, ШИМ
#define WPD 0.095f    // имп/40мс колеса на °/с рыскания: из логов, 80 °/с ≈ 7.5 имп/40мс
float FLA  = 10;      // прямая подача лишнего трения левого колеса, ШИМ (в воздухе моторы равны)
float PKS  = 3.0f;    // множитель KSP в повороте: 0.5 хуже, 1.5/2.5 лучше, 3 — выбрано, 4 качка
// Возврат домой по прямой: TURN — к дому, DRIVE — ехать, FACE — в исходный курс
enum { NAV_IDLE, NAV_TURN, NAV_DRIVE, NAV_FACE };
uint8_t nav = NAV_IDLE;
bool faceGo = false;                  // доворот дома начался
uint32_t navStart = 0;                // когда начат возврат
#define NAV_MAX_MS 30000UL            // дольше — сдаёмся: упёрся в препятствие или качается у цели
float KNT  = 2.0f;    // °/с поворота на ° ошибки курса
float KND  = 0.3f;    // имп/40мс скорости на см до дома (0.5 — тормозил поздно, подъезжал на 20)
float NTOL = 5;       // см: дома
float FTOL = 2;       // °: доворот в исходный курс (5 — недокручивал ~5°)
#define NAV_YMIN 25   // °/с — меньше тугое колесо не сдвинет
#define PIV_V 6       // имп/40мс: к этой скорости усиленный демпфер поворота гаснет до обычного
#define FACE_V 3      // имп/40мс: доворот дома — только когда робот почти встал
float TBA  = 1.6f;    // доля поворота левого колеса в повороте на ходу (W+A): левое на полу туже

// ---------- Состояние ----------
int16_t AcX, AcY, AcZ, GyX, GyZ;
float gxOff = 0, gzOff = 0;
float angle = 0, rate = 0, yawR = 0;
bool armed = false;
uint32_t uprightSince = 0;

volatile long encA = 0, encB = 0;          // для контура скорости (обнуляются каждые 40 мс)
volatile long totA = 0, totB = 0;          // накопительные, для теста 't'
float vF = 0, posI = 0, speedOut = 0;
float moveSet = 0, turnSet = 0;
float heading = 0, headTgt = 0;           // курс по гироскопу Z и его цель, °
float posRest = 0;                         // интеграл положения до начала езды
bool stopping = false;                     // идёт «забывание» отставания после W/S
bool pivot = false;                        // сейчас поворот на месте (A/D без W/S)
float wIA = 0, wIB = 0;                    // интегралы скорости колёс в повороте, ШИМ
float wA = 0, wB = 0;                      // скорость колёс по периоду импульсов, имп/40мс, каждые 5 мс
int8_t lastFwd = 0;
uint8_t spdCnt = 0;
uint32_t tW = 0, tS = 0, tA = 0, tD = 0;   // время последней буквы
float uL = 0, uR = 0;

// ---------- Одометрия ----------
// Путь по энкодерам, курс по гироскопу Z (колёса на поворотах проскальзывают).
// Дом — (0,0), курс 0: ставится при взводе и командой home.
#define MM_PER_CNT 0.694f             // по рулетке 2026-09-30: 85 см реально при 49 по 0.400 (π·63/495);
                                      // выходит ~285 имп/оборот колеса, а не 495 — редуктор не 45:1
float odoX = 0, odoY = 0;             // мм: x — вперёд от дома, y — влево
float odoTh = 0;                      // курс, °, + влево, −180…180

float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void odoReset() { odoX = odoY = odoTh = 0; }

uint32_t tNext = 0, tickCnt = 0, hzMark = 0;
uint16_t hz = 0, hzCnt = 0, imuErr = 0;
bool stream = true;

// ---------- Энкодеры: RISING на A, направление по B ----------
volatile uint32_t edgeA = 0, edgeB = 0;   // время последнего фронта, мкс
volatile uint16_t perA = 0, perB = 0;      // период между фронтами, мкс
volatile int8_t dirA = 0, dirB = 0;        // направление последнего фронта
void isrA() {
  long d = (PINC & _BV(PC2)) ? 1 : -1; encA += d; totA += d; dirA = d;
  uint32_t t = micros(); uint32_t p = t - edgeA; edgeA = t; perA = p > 65535 ? 65535 : p;
}
void isrB() {
  long d = (PINC & _BV(PC3)) ? -1 : 1; encB += d; totB += d; dirB = d;   // esB = -1
  uint32_t t = micros(); uint32_t p = t - edgeB; edgeB = t; perB = p > 65535 ? 65535 : p;
}

// ---------- MPU ----------
void mpuWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(true);
}

bool mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  uint8_t n = Wire.requestFrom(MPU_ADDR, 14, true);
  if (n < 14 || Wire.getWireTimeoutFlag()) { Wire.clearWireTimeoutFlag(); return false; }
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                              // температура
  GyX = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                              // GyY не нужен
  GyZ = Wire.read() << 8 | Wire.read();
  return true;
}

float accAngle() { return atan2(AcY - ABY, AcZ - ABZ) * 57.29578f; }

// Калибровка смещений гироскопа по окнам 2 с (400 тиков). Окно засчитывается,
// только если робот не шевелился: размах GyX и GyZ меньше CAL_SPREAD. Работает
// и на старте, и всё время, пока робот лежит (не взведён): смещение от нагрева
// и от сбитой стартовой калибровки (робота двигали при включении) уходит само.
#define CAL_N 400
#define CAL_SPREAD 100      // LSB ≈ 1.5 °/с: больше — робот двигали, окно выбрасываем
long calSx = 0, calSz = 0;
int16_t calMinX, calMaxX, calMinZ, calMaxZ;
uint16_t calN = 0;
bool calGood = false;                      // была честная калибровка; без неё не взводимся
uint32_t twitchUntil = 0;                  // «готов»: колёса дёргаются вперёд после первой калибровки
#define TWITCH_MS 400
#define TWITCH_PWM 60
float calPrevX = 1e9, calPrevZ = 1e9;       // среднее прошлого тихого окна
#define CAL_AGREE 30        // LSB ≈ 0.3 °/с: два окна подряд обязаны совпасть. Иначе
                            // плавный перенос робота (малый разброс, но сдвинутое
                            // среднее) проходил проверку и сбивал ноль на 25 °/с

bool calFeed() {                                         // true — смещения обновлены
  if (calN == 0) { calMinX = calMaxX = GyX; calMinZ = calMaxZ = GyZ; calSx = calSz = 0; }
  calSx += GyX; calSz += GyZ;
  if (GyX < calMinX) calMinX = GyX; if (GyX > calMaxX) calMaxX = GyX;
  if (GyZ < calMinZ) calMinZ = GyZ; if (GyZ > calMaxZ) calMaxZ = GyZ;
  if (++calN < CAL_N) return false;
  calN = 0;
  if (calMaxX - calMinX > CAL_SPREAD || calMaxZ - calMinZ > CAL_SPREAD) {
    calPrevX = calPrevZ = 1e9;                           // шевелили — начать заново
    return false;
  }
  float mx = (float)calSx / CAL_N, mz = (float)calSz / CAL_N;
  bool agree = fabs(mx - calPrevX) < CAL_AGREE && fabs(mz - calPrevZ) < CAL_AGREE;
  calPrevX = mx; calPrevZ = mz;
  if (!agree) return false;
  gxOff = mx;
  gzOff = mz;
  if (!calGood) twitchUntil = millis() + TWITCH_MS;   // первая — показать, что готов
  calGood = true;
  return true;
}

void gyroCalib(uint16_t msMax) {                         // старт: ждём неподвижности
  uint32_t t0 = millis();
  calN = 0; calPrevX = calPrevZ = 1e9;
  while (millis() - t0 < msMax) {
    if (mpuRead() && calFeed()) return;
    delay(5);
  }
  long sx = 0, sz = 0; int n = 0;                        // не дождались — грубое среднее за 1 с,
  for (uint32_t t1 = millis(); millis() - t1 < 1000; delay(5))   // уточнится, когда робот ляжет
    if (mpuRead()) { sx += GyX; sz += GyZ; n++; }
  gxOff = n ? (float)sx / n : 0;
  gzOff = n ? (float)sz / n : 0;
  calN = 0;
  Serial.println(F("гироскоп прогревается — лежи, колёса дёрнутся, когда будет готов"));
}

// ---------- Моторы ----------
// «Вперёд» = IN1 LOW / IN2 HIGH: так робот ловил наклон в старой gyrobot_mov (DRIVE_SIGN -1)
void motorOut(uint8_t pwmPin, uint8_t in1, uint8_t in2, float u, float db) {
  int p = 0;
  if (u > 0.5f || u < -0.5f) p = (int)(fabs(u) + db);   // мёртвая зона: прибавка на старте
  p = constrain(p, 0, 255);
  digitalWrite(in1, u < 0);
  digitalWrite(in2, u >= 0);
  analogWrite(pwmPin, p);
}

void motorsOff() { analogWrite(PWMA, 0); analogWrite(PWMB, 0); uL = uR = 0; }

void resetLoops() {                                      // взвод и падение: всё с нуля
  noInterrupts(); encA = encB = 0; interrupts();
  vF = posI = speedOut = 0;
  moveSet = turnSet = 0;
  spdCnt = 0;
  tW = tS = tA = tD = 0;
  stopping = false; lastFwd = 0; pivot = false; wIA = wIB = 0;
  odoReset();                                            // новый взвод — новый дом
  nav = NAV_IDLE;
  heading = headTgt = 0;
}

// ---------- Скорость колёс по периоду импульсов ----------
// Счёт импульсов за 5 мс даёт только 0/1/2/3 (замер step 2026-09-28), а период
// между фронтами — гладкую скорость. Если колесо стоит дольше последнего периода,
// берём прошедшее время: скорость плавно уходит к нулю, а не залипает.
float periodSpeed(uint16_t per, uint32_t since, int8_t dir) {
  if (since > 100000UL || per == 0) return 0;            // >0.1 с без импульса — стоит
  uint32_t p = (since > per) ? since : per;
  return dir * 40000.0f / p;                             // имп/40мс
}

void updateWheelSpeeds() {                               // каждые 5 мс
  noInterrupts();
  uint32_t eA = edgeA, eB = edgeB; uint16_t pA = perA, pB = perB;
  int8_t dA = dirA, dB = dirB;
  interrupts();
  uint32_t now = micros();
  wA = 0.5f * wA + 0.5f * periodSpeed(pA, now - eA, dA);
  wB = 0.5f * wB + 0.5f * periodSpeed(pB, now - eB, dB);
}

// ---------- WASD: цели с рампой ----------
bool held(uint32_t t) { return t && millis() - t < KEY_MS; }

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
    if (dist < NTOL + 1.2f * fabs(vF) || fabs(err) > 90) nav = NAV_FACE;   // тормозной путь / проскочили
    else if (fabs(err) > 45) nav = NAV_TURN;              // сильно сбились — довернуть
    else {
      mt = constrain(KND * dist, 4.0f, MOVE);
      yt = constrain(KNT * err, -40.0f, 40.0f);           // подруливание на ходу
    }
  }
  if (nav == NAV_FACE && moveSet == 0 && fabs(vF) < FACE_V) faceGo = true;   // встал — можно
  if (nav == NAV_FACE && faceGo) {                        // дальше без проверки скорости: иначе
                                                          // доворот замирал при v≈3 — рывками
    float e = wrap180(-odoTh);
    if (fabs(e) < FTOL) nav = NAV_IDLE;
    else yt = navTurn(e, YMAX);
  }
}

void updateTargets() {                                   // раз в 40 мс
  int fwd = (int)held(tW) - (int)held(tS);
  int lr  = (int)held(tA) - (int)held(tD);
  if (fwd || lr) nav = NAV_IDLE;                          // человек перехватил управление
  if (nav && millis() - navStart > NAV_MAX_MS) nav = NAV_IDLE;   // не бесконечно (стена, BT пропал)
  float mt, yt;
  if (nav) navTargets(mt, yt);
  else { mt = fwd * MOVE; yt = lr * YMAX; }
  int8_t drv = fwd ? fwd : (nav == NAV_DRIVE ? 1 : 0);   // езда: клавишей или автоматом
  if (drv && !lastFwd && !stopping) posRest = posI;     // начало езды: запомнить стоянку
  if (drv) stopping = false;
  else if (lastFwd) stopping = true;                    // конец езды
  lastFwd = drv;
  // рампа RAMP_UP/RAMP_DN имп/40мс за тик (2 ≈ 0.5 м/с², наклон ≈3°)
  float step = (fabs(mt) > fabs(moveSet)) ? RAMP_UP : RAMP_DN;
  moveSet += constrain(mt - moveSet, -step, step);
  if (yt == 0) turnSet = 0;                              // отпустил — сразу ноль, тормозит KT
  else turnSet += constrain(yt - turnSet, -15.0f, 15.0f);
}

// ---------- Тик 5 мс ----------
void controlTick() {
  if (!mpuRead()) { imuErr++; return; }
  const float dt = TICK_US * 1e-6f;
  rate = (GyX - gxOff) / GYR_LSB;
  yawR = YS * (GyZ - gzOff) / GYR_LSB;
  angle = (1.0f - K_ACC) * (angle + rate * dt) + K_ACC * accAngle();
  float th = angle - AZ;

  if (!armed) {                                          // ждём стойку: |θ|<5° полсекунды
    if (twitchUntil && (long)(millis() - twitchUntil) < 0) {   // сигнал «откалиброван»
      motorOut(PWMA, AIN1, AIN2, TWITCH_PWM, 0);
      motorOut(PWMB, BIN1, BIN2, TWITCH_PWM, 0);
      return;
    }
    twitchUntil = 0;
    motorsOff();
    if (calFeed()) {                                     // лежит неподвижно — уточнить смещения
      Serial.print(F("cal gx0=")); Serial.print(gxOff, 0);
      Serial.print(F(" gz0=")); Serial.println(gzOff, 0);
    }
    if (fabs(th) < 5 && calGood) {
      if (!uprightSince) uprightSince = millis();
      if (millis() - uprightSince > 500) {
        resetLoops(); armed = true; uprightSince = 0; calN = 0; calPrevX = calPrevZ = 1e9;
        Serial.println(F("armed"));
      }
    } else uprightSince = 0;
    return;
  }
  if (fabs(th) > FALL) {
    armed = false; motorsOff(); resetLoops();
    Serial.println(F("fall"));
    return;
  }

  float balance = KP * th + KD * rate;
  updateWheelSpeeds();

  if (++spdCnt >= SPD_TICKS) {                           // 40 мс: скорость и положение
    spdCnt = 0;
    noInterrupts(); long cA = encA, cB = encB; encA = encB = 0; interrupts();
    float v = (cA + cB) * 0.5f;                          // имп/40мс, + вперёд
    vF = 0.7f * vF + 0.3f * v;                           // фильтр Tumbller
    float ds = (cA + cB) * 0.5f * MM_PER_CNT;          // мм за 40 мс, + вперёд
    odoX += ds * cos(odoTh * 0.0174533f);
    odoY += ds * sin(odoTh * 0.0174533f);
    updateTargets();
    // поворот на месте = A/D без W/S (и без езды автоматом). Не по moveSet: после
    // отпускания W цель скорости гаснет ещё ~1.3 с, и поворот шёл одним колесом
    pivot = turnSet != 0 && lastFwd == 0;
    posI = constrain(posI + vF - moveSet, -ILIM, ILIM);    // положение минус цель
    if (stopping) {                                        // забываем отставание от цели
      float d = posRest - posI;
      posI += constrain(d, -BLEED, BLEED);
      if (fabs(d) <= BLEED && moveSet == 0) stopping = false;
    }
    // усиленный демпфер поворота на месте плавно гаснет со скоростью: PKS на месте,
    // 1 при |v| ≥ PIV_V. На ходу PKS×KSP×v давал удар до ШИМ 255 и наклон ±28°,
    // а жёсткий порог по скорости дёргал поворот (режим то вкл, то выкл)
    float pk = pivot ? 1.0f + (PKS - 1.0f) * max(0.0f, 1.0f - fabs(vF) / PIV_V) : 1.0f;
    speedOut = pk * KSP * vF + KSI * posI;                 // тот же знак, что у balance
  }

  heading += yawR * dt;
  odoTh = wrap180(odoTh + yawR * dt);
  if (turnSet != 0) headTgt = heading;                  // в повороте цель курса едет с роботом
  float hErr = constrain(headTgt - heading, -30.0f, 30.0f);
  float turn = TFF * turnSet + KT * (turnSet - yawR) + KH * hErr;   // >0 → правое быстрее → влево
  float tL = -TBA * turn, tR = turn;
  if (pivot) {                                           // поворот по скоростям колёс
    float w = WPD * turnSet;                             // цель: левое −w, правое +w
    float eA = -w - wA, eB = w - wB;                     // скорость по периоду, каждые 5 мс
    float ec = 0.5f * (eA + eB);                         // общая часть (−скорость робота) —
    eA -= ec; eB -= ec;                                  // дело баланса, берём только разность
    wIA = constrain(wIA + KWI / SPD_TICKS * eA, -WI_LIM, WI_LIM);   // KWI — за 40 мс
    wIB = constrain(wIB + KWI / SPD_TICKS * eB, -WI_LIM, WI_LIM);
    tL = -TFF * turnSet + KW * eA + wIA + (turnSet > 0 ? -FLA : FLA);   // левое: назад при A
    tR =  TFF * turnSet + KW * eB + wIB;
  } else wIA = wIB = 0;                                  // вне поворота — с нуля
  uL = constrain(balance + speedOut + tL, -255.0f, 255.0f);
  uR = constrain(balance + speedOut + tR, -255.0f, 255.0f);
  motorOut(PWMA, AIN1, AIN2, uL, DBA);
  motorOut(PWMB, BIN1, BIN2, uR, DBB);
}

// ---------- Сериал: WASD, параметры, тесты ----------
// ---------- Настройки в EEPROM: save / defaults ----------
// Сохраняются все настраиваемые параметры (не калибровка — она при каждом старте).
// Вместе с ними — «отпечаток» заводских значений прошивки: если после перепрошивки
// заводские в коде другие, сохранённое игнорируется (иначе старое молча перебило бы
// новые значения из кода). Повреждённая или пустая память — тоже заводские.
float* const CFG[] = {&KP, &KD, &KSP, &KSI, &ILIM, &KT, &TFF, &KH, &YMAX, &MOVE, &AZ,
                      &DBA, &DBB, &FALL, &RAMP_UP, &RAMP_DN, &BLEED, &KW, &KWI, &FLA,
                      &PKS, &TBA, &KNT, &KND, &NTOL, &FTOL};
const uint8_t CFG_N = sizeof(CFG) / sizeof(CFG[0]);
#define CFG_MAGIC 0x4731              // «G1»
float cfgFactory[CFG_N];              // заводские значения этой прошивки (копия при старте)
uint32_t cfgFactoryHash = 0;

uint32_t fnv(const uint8_t* p, uint16_t n, uint32_t h = 2166136261UL) {
  while (n--) { h ^= *p++; h *= 16777619UL; }
  return h;
}

// Раскладка: magic(2) | N(1) | отпечаток заводских(4) | N float | контрольная сумма(4)
uint32_t cfgStoredSum(uint16_t len) {                     // сумма байтов 0..len-1 в EEPROM
  uint32_t h = 2166136261UL;
  for (uint16_t i = 0; i < len; i++) { uint8_t b = EEPROM.read(i); h = fnv(&b, 1, h); }
  return h;
}

void cfgSave() {
  uint16_t a = 0;
  uint16_t magic = CFG_MAGIC;
  EEPROM.put(a, magic); a += 2;
  EEPROM.update(a, CFG_N); a += 1;
  EEPROM.put(a, cfgFactoryHash); a += 4;
  for (uint8_t i = 0; i < CFG_N; i++) { EEPROM.put(a, *CFG[i]); a += 4; }
  EEPROM.put(a, cfgStoredSum(a));
  Serial.println(F("сохранено в память робота"));
}

// 0 — загружено; 1 — памяти нет/повреждена; 2 — прошивка с другими заводскими
uint8_t cfgLoad() {
  uint16_t magic; uint32_t fh, sum;
  EEPROM.get(0, magic);
  if (magic != CFG_MAGIC || EEPROM.read(2) != CFG_N) return 1;
  uint16_t end = 7 + 4 * CFG_N;
  EEPROM.get(end, sum);
  if (sum != cfgStoredSum(end)) return 1;
  EEPROM.get(3, fh);
  if (fh != cfgFactoryHash) return 2;
  for (uint8_t i = 0; i < CFG_N; i++) EEPROM.get(7 + 4 * i, *CFG[i]);
  return 0;
}

void cfgInit() {                                          // в setup, до первого printParams
  for (uint8_t i = 0; i < CFG_N; i++) cfgFactory[i] = *CFG[i];
  cfgFactoryHash = fnv((const uint8_t*)cfgFactory, sizeof(cfgFactory));
  uint8_t r = cfgLoad();
  if (r == 0) Serial.println(F("настройки: из памяти робота (save)"));
  else if (r == 2) Serial.println(F("настройки: заводские — прошивка новая, сохранённые не подходят"));
  else Serial.println(F("настройки: заводские"));
}

void cfgDefaults() {
  for (uint8_t i = 0; i < CFG_N; i++) *CFG[i] = cfgFactory[i];
  uint16_t zero = 0;
  EEPROM.put(0, zero);                                    // стереть метку — при старте заводские
  Serial.println(F("заводские настройки, память очищена"));
}

void printParams() {
  Serial.print(F("kp=")); Serial.print(KP, 2);
  Serial.print(F(" kd=")); Serial.print(KD, 3);
  Serial.print(F(" ksp=")); Serial.print(KSP, 3);
  Serial.print(F(" ksi=")); Serial.print(KSI, 4);
  Serial.print(F(" ilim=")); Serial.print(ILIM, 0);
  Serial.print(F(" kt=")); Serial.print(KT, 3);
  Serial.print(F(" ymax=")); Serial.print(YMAX, 0);
  Serial.print(F(" move=")); Serial.print(MOVE, 1);
  Serial.print(F(" az=")); Serial.print(AZ, 2);
  Serial.print(F(" dba=")); Serial.print(DBA, 0);
  Serial.print(F(" dbb=")); Serial.print(DBB, 0);
  Serial.print(F(" fall=")); Serial.print(FALL, 0);
  Serial.print(F(" ys=")); Serial.print(YS);
  Serial.print(F(" acc=")); Serial.print(RAMP_UP, 1);
  Serial.print(F(" dec=")); Serial.print(RAMP_DN, 1);
  Serial.print(F(" tba=")); Serial.print(TBA, 2);
  Serial.print(F(" bleed=")); Serial.print(BLEED, 0);
  Serial.print(F(" tff=")); Serial.print(TFF, 2);
  Serial.print(F(" kh=")); Serial.print(KH, 2);
  Serial.print(F(" kw=")); Serial.print(KW, 2);
  Serial.print(F(" kwi=")); Serial.print(KWI, 2);
  Serial.print(F(" fla=")); Serial.print(FLA, 0);
  Serial.print(F(" pks=")); Serial.print(PKS, 2);
  Serial.print(F(" knt=")); Serial.print(KNT, 2);
  Serial.print(F(" knd=")); Serial.print(KND, 2);
  Serial.print(F(" ntol=")); Serial.print(NTOL, 0);
  Serial.print(F(" ftol=")); Serial.println(FTOL, 1);

}

// Тест моторов (только когда не взведён): оба колеса «вперёд» 1 с,
// оба счётчика обязаны вырасти в плюс
void motorTest(int pwm) {
  if (armed) { Serial.println(F("сначала положи робота (не взведён)")); return; }
  noInterrupts(); totA = totB = 0; interrupts();
  motorOut(PWMA, AIN1, AIN2, pwm, 0);
  motorOut(PWMB, BIN1, BIN2, pwm, 0);
  delay(1000);
  motorsOff();
  Serial.print(F("test totA=")); Serial.print(totA);
  Serial.print(F(" totB=")); Serial.println(totB);
  tNext = micros();
}

void applyLine(char* line) {
  char* val = line;
  while (*val && *val != ' ' && *val != '=') val++;
  bool hasVal = *val;
  if (hasVal) *val++ = 0;
  float v = atof(val);
  uint32_t now = millis();

  if (!strcmp(line, "h")) { if (armed) { nav = NAV_TURN; faceGo = false; navStart = millis(); } return; }   // домой
  if (!strcmp(line, "x")) { nav = NAV_IDLE; return; }                // стоп возврата
  if (!line[1] && strchr("wsad", line[0])) {             // буквы WASD — без ответа
    if (line[0] == 'w') tW = now; else if (line[0] == 's') tS = now;
    else if (line[0] == 'a') tA = now; else tD = now;
    return;
  }
  if (!strcmp(line, "p")) { printParams(); return; }
  if (!strcmp(line, "tele")) { stream = hasVal ? v != 0 : !stream; return; }   // не "s": это WASD
  if (!strcmp(line, "save")) { cfgSave(); return; }
  if (!strcmp(line, "home")) { odoReset(); Serial.println(F("дом здесь")); return; }
  if (!strcmp(line, "defaults")) { cfgDefaults(); printParams(); return; }
  if (!strcmp(line, "z")) { AZ = angle; Serial.print(F("az=")); Serial.println(AZ, 2); return; }
  if (!strcmp(line, "t")) { motorTest(hasVal ? (int)v : 60); return; }
  if (!hasVal) { Serial.println(F("?")); return; }

  if      (!strcmp(line, "kp"))   KP = v;
  else if (!strcmp(line, "kd"))   KD = v;
  else if (!strcmp(line, "ksp"))  KSP = v;
  else if (!strcmp(line, "ksi"))  KSI = v;
  else if (!strcmp(line, "ilim")) ILIM = v;
  else if (!strcmp(line, "kt"))   KT = v;
  else if (!strcmp(line, "ymax")) YMAX = v;
  else if (!strcmp(line, "move")) MOVE = v;
  else if (!strcmp(line, "az"))   AZ = v;
  else if (!strcmp(line, "dba"))  DBA = v;
  else if (!strcmp(line, "dbb"))  DBB = v;
  else if (!strcmp(line, "fall")) FALL = v;
  else if (!strcmp(line, "ys"))   YS = (v < 0) ? -1 : 1;
  else if (!strcmp(line, "acc") && v > 0) RAMP_UP = v;
  else if (!strcmp(line, "dec") && v > 0) RAMP_DN = v;
  else if (!strcmp(line, "tba") && v > 0) TBA = v;
  else if (!strcmp(line, "bleed") && v >= 0) BLEED = v;
  else if (!strcmp(line, "tff") && v >= 0) TFF = v;
  else if (!strcmp(line, "kw") && v >= 0) KW = v;
  else if (!strcmp(line, "kwi") && v >= 0) KWI = v;
  else if (!strcmp(line, "fla") && v >= 0) FLA = v;
  else if (!strcmp(line, "pks") && v >= 0) PKS = v;
  else if (!strcmp(line, "knt") && v > 0) KNT = v;
  else if (!strcmp(line, "knd") && v > 0) KND = v;
  else if (!strcmp(line, "ntol") && v > 0) NTOL = v;
  else if (!strcmp(line, "ftol") && v > 0) FTOL = v;
  else if (!strcmp(line, "kh") && v >= 0) { KH = v; headTgt = heading; }
  else { Serial.println(F("?")); return; }
  Serial.println(F("ok"));
}

void camPoll() {
  static char line[24]; static uint8_t len = 0;
  while (cam.available()) {
    char c = cam.read();
    if (c == '\n' || c == '\r') {
      if (len) { line[len] = 0; Serial.print(F("cam> ")); Serial.println(line); len = 0; }
    } else if (len < sizeof(line) - 1) line[len++] = c;
  }
}

void serialPoll() {
  static char line[20]; static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { line[len] = 0; applyLine(line); len = 0; }
    } else if (len < sizeof(line) - 1) line[len++] = c;
  }
}

void telemetry() {                                       // ~10 Гц
  Serial.print(armed ? F("A ") : F("- "));
  Serial.print(F("a=")); Serial.print(angle - AZ, 2);
  Serial.print(F(" r=")); Serial.print(rate, 0);
  Serial.print(F(" v=")); Serial.print(vF, 1);
  Serial.print(F(" vA=")); Serial.print(wA, 1);
  Serial.print(F(" vB=")); Serial.print(wB, 1);
  Serial.print(F(" x=")); Serial.print(posI, 0);
  Serial.print(F(" m=")); Serial.print(moveSet, 0);
  Serial.print(F(" yr=")); Serial.print(yawR, 0);
  Serial.print(F(" h=")); Serial.print(heading - headTgt, 1);
  Serial.print(F(" uL=")); Serial.print(uL, 0);
  Serial.print(F(" uR=")); Serial.print(uR, 0);
  Serial.print(F(" px=")); Serial.print(odoX * 0.1f, 0);
  Serial.print(F(" py=")); Serial.print(odoY * 0.1f, 0);
  Serial.print(F(" ph=")); Serial.print(odoTh, 0);
  Serial.print(F(" nav=")); Serial.print(nav);
  Serial.print(F(" hz=")); Serial.print(hz);
  Serial.print(F(" ie=")); Serial.println(imuErr);
}

void setup() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  TCCR1B = (TCCR1B & 0xF8) | 0x02;                       // D9/D10: ШИМ 3.9 кГц
  motorsOff();

  pinMode(PIN_ECA_A, INPUT_PULLUP); pinMode(PIN_ECB_A, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP); pinMode(A3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ECA_A), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ECB_A), isrB, RISING);

  Serial.begin(115200);
  cfgInit();
  cam.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true);                      // без этого I2C висел от помех моторов

  mpuWrite8(0x6B, 0x00); delay(100);                     // будим
  mpuWrite8(0x1A, 0x03);                                 // DLPF 44 Гц (как Balanduino/YABR)
  mpuWrite8(0x1B, 0x08);                                 // гироскоп ±500 °/с
  mpuWrite8(0x1C, 0x00);                                 // аксель ±2g

  Serial.println(F("tumbller: калибровка гироскопа, не трогать (ждёт неподвижности)"));
  gyroCalib(10000);
  if (!mpuRead()) mpuRead();
  angle = accAngle();
  Serial.print(F("gx0=")); Serial.print(gxOff, 0);
  Serial.print(F(" gz0=")); Serial.println(gzOff, 0);
  printParams();
  tNext = micros();
  hzMark = millis();
}

void loop() {
  serialPoll();
  camPoll();
  if ((long)(micros() - tNext) < 0) return;
  tNext += TICK_US;
  if ((long)(micros() - tNext) > (long)TICK_US) tNext = micros();   // отстали — не догонять пачкой

  controlTick();
  hzCnt++;
  if (millis() - hzMark >= 1000) { hzMark += 1000; hz = hzCnt; hzCnt = 0; imuErr = 0; }
  if (stream && (++tickCnt % 20) == 0) telemetry();
}
