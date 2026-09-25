// ============================================================
// Гиробот — порт алгоритма listing_PID_2I_mov ("сознательное движение").
// Перезаписан с нуля 2026-09-21 по мотивам разбора порта: минимум кода,
// порядок цикла и логика как в оригинале, только железные отличия нашей
// платы (TB6612, оси/офсеты MPU, шкала ±500, I2C-таймаут, автовзвод).
//
// Порядок цикла (как в оригинале):
//   1. моторы по выходу ПРОШЛОЙ итерации
//   2. чтение MPU, комплементарный фильтр
//   3. падение -> стоп, ждём вертикаль
//   4. PID: Kp*угол + Kd*гироскоп + Ki*интеграл
//   5. адаптивный ноль: пока |pid|>0.7, точка равновесия уползает,
//      feedforward снимает накопленное
// ============================================================
#include "config.h"
#include "imu.h"
#include "motors.h"
#include "speed.h"

// ---------- Состояние ----------
float balancing_zerro = 0.4;  // точка равновесия (подбор 2026-09-21, было 1.4)
float GyYsum = 0;             // фильтрованный угол
float GyYsumPID = 0;          // выход PID прошлой итерации
float SumIntegral = 0;
float rate = 0, dt = 0;
bool  armed = false;
uint32_t timer = 0, uprightSince = 0, iterCnt = 0, hzCnt = 0, hzMark = 0;
uint16_t loopHz = 0;

// ---------- Живая настройка по сериалу (без перепрошивки) ----------
float kp = KP_DEF, kd = KD_DEF, ki_s = KI_DEF, pid_dead = DEAD_DEF;
float imax = 1.0f;   // потолок интеграла (подбор: 1.0 -> 0.3 -> 0.9 -> вернули 1.0)

// ---------- WASD (BT/USB) ----------
float drv = -15.0f, trn = 60.0f;     // наклон (минус = вперёд) и поворот
float drvTarget = 0, drvNow = 0;    // сглаженный наклон добавляется к нолю равновесия
float trnTarget = 0, trnNow = 0;    // дифференциал: A +, B −
bool keyW = false, keyS = false, keyA = false, keyD = false;
uint32_t cmdDeadline = 0;

void applyParam(const char* line) {
  char name[8]; uint8_t i = 0;
  while (line[i] && line[i] != ' ' && i < 7) { name[i] = line[i]; i++; }
  name[i] = 0;
  while (line[i] == ' ') i++;
  float v = atof(line + i);
  if (!strcmp(name, "p")) {
    Serial.print(F("kp=")); Serial.print(kp, 3);
    Serial.print(F(" kd=")); Serial.print(kd, 4);
    Serial.print(F(" ki=")); Serial.print(ki_s, 2);
    Serial.print(F(" dead=")); Serial.print(pid_dead, 2);
    Serial.print(F(" imax=")); Serial.print(imax, 2);
    Serial.print(F(" pmin=")); Serial.print(pmin, 0);
    Serial.print(F(" atr=")); Serial.print(atr, 0);
    Serial.print(F(" atr=")); Serial.print(atr, 0);
    Serial.print(F(" brt=")); Serial.print(brt, 0);
    Serial.print(F(" ksp=")); Serial.print(ksp, 3);
    Serial.print(F(" ksi=")); Serial.print(ksi, 4);
    Serial.print(F(" kdy=")); Serial.print(kdyaw, 3);
    Serial.print(F(" kdi=")); Serial.print(kdyi, 4);
    Serial.print(F(" esa=")); Serial.print(esA, 0);
    Serial.print(F(" esb=")); Serial.print(esB, 0);
    Serial.print(F(" zr=")); Serial.println(balancing_zerro, 3);
  } else if (!strcmp(name, "kp") && v > 0) { kp = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "kd") && v >= 0) { kd = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "ki") && v >= 0) { ki_s = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "dead") && v >= 0) { pid_dead = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "imax") && v > 0) { imax = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "pmin") && v > 0 && v <= 100) { pmin = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "atr") && v >= 0 && v <= 30) { atr = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "brt") && v >= 0 && v <= 30) { brt = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "ksp") && v >= 0) { ksp = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "kdy") && v > -5 && v < 5) { kdyaw = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "kdif") && v >= 0 && v <= 0.1) { kdif = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "kdi") && v > -5 && v < 5) { kdyi = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "ksi") && v >= 0) { ksi = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "esa") && fabs(v) == 1) { esA = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "esb") && fabs(v) == 1) { esB = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "drv") && v >= -10 && v <= 10) { drv = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "trn") && v >= -100 && v <= 100) { trn = v; Serial.println(F("ok"));
  } else if (!strcmp(name, "zr") && v >= -3 && v <= 3) { balancing_zerro = v; Serial.println(F("ok"));
  } else if (name[1] == 0 && strchr("wsad", name[0])) {
    if (name[0] == 'w') keyW = true;
    if (name[0] == 's') keyS = true;
    if (name[0] == 'a') keyA = true;
    if (name[0] == 'd') keyD = true;
    cmdDeadline = millis() + 300;
    drvTarget = (keyW ? drv : 0) - (keyS ? drv : 0);
    trnTarget = (keyD ? trn : 0) - (keyA ? trn : 0);
    Serial.println(F("ok"));
  }
}

void paramsPoll() {                      // строки "имя значение", "p" — показать
  static char line[16]; static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { line[len] = 0; applyParam(line); len = 0; }
    } else if (len < 15) line[len++] = c;
  }
}

void setup() {
  motorsInit();
  speedInit();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);               // чтение ~0.4 мс вместо ~1.6
  Wire.setWireTimeout(25000, true);    // без этого I2C зависал от помех моторов

  Serial.println(F("калибровка гироскопа 2с, не трогать"));
  mpuWake();
  gyroCalib(2000);

  if (!mpuRead()) mpuRead();
  GyYsum = accAngle() + balancing_zerro;   // сеем угол фильтра
  Serial.println(F("поставь на колёса"));
  timer = micros();
  hzMark = millis();
  armed = true;
}

void loop() {
  iterCnt++; hzCnt++;
  if (millis() - hzMark >= 1000) { hzMark = millis(); loopHz = hzCnt; hzCnt = 0; }
  paramsPoll();                          // живая настройка: "kd 0.005", "p"
  if (millis() > cmdDeadline && (keyW || keyS || keyA || keyD)) {   // поток букв кончился — стоим
    keyW = keyS = keyA = keyD = false;
    drvTarget = 0; trnTarget = 0;
  }
  speedTick();                           // окно 40 мс: скорость/путь (и в idle)

  // --- после падения: моторы стоят, ждём секунду стойки вертикально ---
  if (!armed) {
    drive(0);
    if (!mpuRead()) return;    timer = micros();                    // не давать dt копить простой
    if (fabs(accAngle()) < 10) {
      if (!uprightSince) uprightSince = millis();
      if (millis() - uprightSince > 1000) {
        GyYsum = constrain(accAngle() + balancing_zerro, -8.0f, 8.0f);  // сеять без пинка
        SumIntegral = 0; GyYsumPID = 0;
        speedFilt = 0; speedInt = 0; speedTerm = 0;   // чистый старт контура скорости
        yawFilt = 0; yawInt = 0; yawTerm = 0;         // и контура курса
        noInterrupts(); encA = 0; encB = 0; interrupts();
        uprightSince = 0;
        armed = true;
        Serial.print(F("armed a=")); Serial.print(accAngle(), 1);
        Serial.print(F(" bias=")); Serial.print(gyroBiasX, 1);
        Serial.print(F(" zr=")); Serial.println(balancing_zerro, 2);
      }
    } else uprightSince = 0;
    return;
  }

  // 1. Двигаемся по выходу ПРОШЛОЙ итерации (порядок оригинала)
  if (GyYsumPID > pid_dead) {
    drive(pwmFromPid(GyYsumPID) * DRIVE_SIGN);
  } else if (GyYsumPID < -pid_dead) {
    drive(-pwmFromPid(-GyYsumPID) * DRIVE_SIGN);
  } else {
    drive(0);
  }

  // 2. Чтение MPU и комплементарный фильтр (тяга к акселю по времени)
  if (!mpuRead()) return;                // сбой I2C — пропустить цикл
  uint32_t t2 = timer; timer = micros();
  dt = (timer - t2) * 0.000001f;
  if (dt > 0.02f) dt = 0.02f;            // защита от застрявшего dt
  drvNow += (drvTarget - drvNow) * (dt / 0.3f);   // TAU 0.3 c: ступенька = качели
  trnNow += (trnTarget - trnNow) * (dt / 0.45f);  // поворот мягче: TAU 0.45
  rate = (GyX - gyroBiasX) / GYR_LSB;    // + = кренится вперёд
  GyYsum += rate * dt + (accAngle() + balancing_zerro + drvNow - GyYsum) * (dt / TAU_ACC);

  // 3. Падение
  if (fabs(GyYsum) > FALL_DEG) {
    armed = false; uprightSince = 0;
    drvNow = 0; trnNow = 0; drvTarget = 0; trnTarget = 0;
    keyW = keyS = keyA = keyD = false;
    Serial.println(F("fall"));
    return;
  }

  // 4. PID. Интеграл по времени, потолок imax (анти-windup). speedTerm —
  //    энкодерный демпфер качения + возврат на место
  SumIntegral = constrain(SumIntegral + GyYsum * dt, -imax, imax);
  GyYsumPID = kp * GyYsum + kd * rate + ki_s * SumIntegral + speedTerm;

  // 5. АДАПТИВНЫЙ НОЛЬ — гейт |pid|>0.7 + защита (|угол|<10, |rate|<60), кламп ±6
  if ((GyYsumPID > 0.7f || GyYsumPID < -0.7f) && fabs(GyYsum) < 10.0f && fabs(rate) < 60) {
    balancing_zerro += GyYsumPID * ZR_RATE * dt;
    balancing_zerro = constrain(balancing_zerro, -6.0f, 6.0f);
    GyYsum += GyYsumPID * FF_RATE * dt;  // их feedforward, снимает накопленное
  }

  // 6. Телеметрия ~7 Гц
  if ((iterCnt & 0x7F) == 0) {
    Serial.print(F("a=")); Serial.print(GyYsum, 2);
    Serial.print(F(" r=")); Serial.print(rate, 1);
    Serial.print(F(" pid=")); Serial.print(GyYsumPID, 2);
    Serial.print(F(" zr=")); Serial.print(balancing_zerro, 3);
    Serial.print(F(" d=")); Serial.print(drvNow, 2);
    Serial.print(F(" t=")); Serial.print(trnNow, 1);
    Serial.print(F(" s=")); Serial.print(speedFilt, 1);
    Serial.print(F(" i=")); Serial.print((long)speedInt);
    Serial.print(F(" kd=")); Serial.print(kd, 4);
    Serial.print(F(" x=")); Serial.print(pwmOut);
    Serial.print(F(" hz=")); Serial.println(loopHz);
  }
}
