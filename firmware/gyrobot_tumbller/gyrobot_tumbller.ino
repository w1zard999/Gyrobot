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
float AZ   = -2.8f;   // ноль угла: при нём интеграл положения стоя ≈ 0
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
bool  PIVOT = true;   // поворот на месте по скоростям колёс (pvt 0 — обычный поворот по гироскопу)
float KW   = 4.0f;    // П: ШИМ на имп/40мс ошибки разности колёс; 8 — рывки
float KWI  = 0.5f;    // И: ШИМ за 40 мс на имп — П оставляет тугое колесо стоять, интеграл докручивает
#define WI_LIM 80     // потолок интегральной добавки, ШИМ
#define WPD 0.095f    // имп/40мс колеса на °/с рыскания: из логов, 80 °/с ≈ 7.5 имп/40мс
float FLA  = 10;      // прямая подача лишнего трения левого колеса, ШИМ (в воздухе моторы равны)
float PKS  = 3.0f;    // множитель KSP в повороте: 0.5 хуже, 1.5/2.5 лучше, 3 — выбрано, 4 качка
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
float vAF = 0, vBF = 0;                    // скорость каждого колеса (телеметрия)
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
  vF = posI = speedOut = 0; vAF = vBF = 0;
  moveSet = turnSet = 0;
  spdCnt = 0;
  tW = tS = tA = tD = 0;
  stopping = false; lastFwd = 0; pivot = false; wIA = wIB = 0;
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

void updateTargets() {                                   // раз в 40 мс
  int fwd = (int)held(tW) - (int)held(tS);
  int lr  = (int)held(tA) - (int)held(tD);
  if (fwd && !lastFwd && !stopping) posRest = posI;     // начало езды: запомнить стоянку
  if (fwd) stopping = false;
  else if (lastFwd) stopping = true;                    // отпустил W/S
  lastFwd = fwd;
  float mt = fwd * MOVE;
  // рампа RAMP_UP/RAMP_DN имп/40мс за тик (2 ≈ 0.5 м/с², наклон ≈3°)
  float step = (fabs(mt) > fabs(moveSet)) ? RAMP_UP : RAMP_DN;
  moveSet += constrain(mt - moveSet, -step, step);
  float yt = lr * YMAX;
  if (lr == 0) turnSet = 0;                              // отпустил — сразу ноль, тормозит KT
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
    vAF = 0.7f * vAF + 0.3f * cA;
    vBF = 0.7f * vBF + 0.3f * cB;
    updateTargets();
    pivot = PIVOT && turnSet != 0 && moveSet == 0;         // поворот на месте, см. PIVOT
    posI = constrain(posI + vF - moveSet, -ILIM, ILIM);    // положение минус цель
    if (stopping) {                                        // забываем отставание от цели
      float d = posRest - posI;
      posI += constrain(d, -BLEED, BLEED);
      if (fabs(d) <= BLEED && moveSet == 0) stopping = false;
    }
    speedOut = (pivot ? PKS : 1.0f) * KSP * vF + KSI * posI;   // тот же знак, что у balance
  }

  heading += yawR * dt;
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
  Serial.print(F(" pvt=")); Serial.print(PIVOT);
  Serial.print(F(" kw=")); Serial.print(KW, 2);
  Serial.print(F(" kwi=")); Serial.print(KWI, 2);
  Serial.print(F(" fla=")); Serial.print(FLA, 0);
  Serial.print(F(" pks=")); Serial.println(PKS, 2);

}

// Тест моторов (только когда не взведён): оба колеса «вперёд» 1 с,
// оба счётчика обязаны вырасти в плюс
// ЗАМЕР (диагностика, 2026-09-28): ступенька ШИМ на оба колеса, робот лежит на боку.
// 100 отсчётов по 5 мс: 60 под ШИМ, 40 после выключения. На каждый отсчёт —
// импульсы за 5 мс и последний период между фронтами. Для оценки постоянной
// времени мотора и шума скорости перед внутренним регулятором скорости колёс.
#define STEP_N 100
#define STEP_ON 60
void stepTest(int pwm) {
  if (armed) { Serial.println(F("сначала положи робота (не взведён)")); return; }
  static int8_t sa[STEP_N], sb[STEP_N];
  static uint16_t qa[STEP_N], qb[STEP_N];
  noInterrupts(); encA = encB = 0; perA = perB = 0; interrupts();
  uint32_t t = micros();
  for (uint8_t i = 0; i < STEP_N; i++) {
    if (i == 0) { motorOut(PWMA, AIN1, AIN2, pwm, 0); motorOut(PWMB, BIN1, BIN2, pwm, 0); }
    if (i == STEP_ON) motorsOff();
    t += 5000;
    while ((long)(micros() - t) < 0) {}
    noInterrupts();
    sa[i] = constrain(encA, -127, 127); sb[i] = constrain(encB, -127, 127); encA = encB = 0;
    uint32_t now = micros();
    qa[i] = (now - edgeA > 65535) ? 65535 : perA;   // колесо давно стоит — период «бесконечный»
    qb[i] = (now - edgeB > 65535) ? 65535 : perB;
    interrupts();
  }
  motorsOff();
  Serial.print(F("step pwm=")); Serial.println(pwm);
  Serial.println(F("i,cA,cB,perA,perB"));
  for (uint8_t i = 0; i < STEP_N; i++) {
    Serial.print(i); Serial.print(','); Serial.print(sa[i]); Serial.print(',');
    Serial.print(sb[i]); Serial.print(','); Serial.print(qa[i]); Serial.print(',');
    Serial.println(qb[i]);
  }
  Serial.println(F("step end"));
  tNext = micros();
}

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

  if (!line[1] && strchr("wsad", line[0])) {             // буквы WASD — без ответа
    if (line[0] == 'w') tW = now; else if (line[0] == 's') tS = now;
    else if (line[0] == 'a') tA = now; else tD = now;
    return;
  }
  if (!strcmp(line, "p")) { printParams(); return; }
  if (!strcmp(line, "s")) { stream = hasVal ? v != 0 : !stream; return; }
  if (!strcmp(line, "z")) { AZ = angle; Serial.print(F("az=")); Serial.println(AZ, 2); return; }
  if (!strcmp(line, "t")) { motorTest(hasVal ? (int)v : 60); return; }
  if (!strcmp(line, "step")) { stepTest(hasVal ? (int)v : 80); return; }
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
  else if (!strcmp(line, "pvt")) PIVOT = v != 0;
  else if (!strcmp(line, "kw") && v >= 0) KW = v;
  else if (!strcmp(line, "kwi") && v >= 0) KWI = v;
  else if (!strcmp(line, "fla") && v >= 0) FLA = v;
  else if (!strcmp(line, "pks") && v >= 0) PKS = v;
  else if (!strcmp(line, "kh") && v >= 0) { KH = v; headTgt = heading; }
  else { Serial.println(F("?")); return; }
  Serial.println(F("ok"));
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
  if ((long)(micros() - tNext) < 0) return;
  tNext += TICK_US;
  if ((long)(micros() - tNext) > (long)TICK_US) tNext = micros();   // отстали — не догонять пачкой

  controlTick();
  hzCnt++;
  if (millis() - hzMark >= 1000) { hzMark += 1000; hz = hzCnt; hzCnt = 0; imuErr = 0; }
  if (stream && (++tickCnt % 20) == 0) telemetry();
}
