// Энкодеры и контур скорости (вернули 2026-09-21 для гашения дёрганья
// после толчка). D2 = мотор A фаза A (INT0), A2 = фаза B; D3 = мотор B
// фаза A (INT1), A3 = фаза B. 11 CPR x 45:1 = 495 имп/оборот колеса 63 мм.
// Энкодер ДО редуктора шумный: скорость фильтруется тяжело, вклады маленькие.
// Знаки esA/esB и вклады ksp/ksi крутятся по сериалу ("esa -1", "ksp 0.1").
#pragma once
#include "config.h"

volatile long encA = 0, encB = 0;
float speedFilt = 0, speedInt = 0, speedTerm = 0;
float esA = 1.0f, esB = -1.0f;   // в сумме "+" = вперёд (проверено толчком 2026-09-21)
float ksp = 0.3f;                // демпфер качения (лучший по толчкам 2026-09-21)
float ksi = 0.005f;              // возврат на место: против медленного уезжания
float kdyaw = 0.1f;              // демпфер вращения (разность колёс)
float kdyi = 0.004f;             // удержание курса: ШИМ на имп «поворота»
float kdif = 0.01f;              // нормализация колёс: ШИМ на имп рассинхрона
float yawFilt = 0, yawInt = 0, yawTerm = 0;
float difInt = 0, difTerm = 0;
bool drivingPrev = false;
uint32_t lastSpeedMs = 0;

void isrA() { encA += (PINC & _BV(PC2)) ? 1 : -1; }
void isrB() { encB += (PINC & _BV(PC3)) ? 1 : -1; }

void speedInit() {
  pinMode(PIN_ECA_A, INPUT_PULLUP); pinMode(PIN_ECB_A, INPUT_PULLUP);
  pinMode(PIN_ECA_B, INPUT_PULLUP); pinMode(PIN_ECB_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ECA_A), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ECB_A), isrB, RISING);
}

// Окно скорости 40 мс: снимаем счётчики атомарно, тяжёлый EMA, копим путь.
// Вызывается ВСЕГДА (и в idle) — телеметрия s= показывает скорость.
void speedTick() {
  uint32_t now = millis();
  if (now - lastSpeedMs < 40) return;
  lastSpeedMs = now;
  noInterrupts();
  long a = encA, b = encB;
  encA = 0; encB = 0;
  interrupts();
  long fwdA = (long)(esA * a), fwdB = (long)(esB * b);
  long imp = fwdA + fwdB;                        // сумма = качение
  long dif = fwdA - fwdB;                        // разность = рассинхрон колёс
  long yaw = fwdA - fwdB;                        // разность = уход курса
  speedFilt = 0.9f * speedFilt + 0.1f * imp;
  yawFilt   = 0.9f * yawFilt   + 0.1f * yaw;
  // во время езды: якорь позиции и память скорости гасим (ksi не тормозит
  // поездку, а в момент отпускания стартуем с нуля — пятиться нечему)
  if (fabs(drvNow) > 0.5f) { speedInt = 0; speedFilt = 0; }
  else {
    if (!drivingPrev) { speedFilt = 0; speedInt = 0; }   // момент отпускания
    speedInt = constrain((speedInt + imp) * 0.995f, -2000.0f, 2000.0f);
  }
  drivingPrev = fabs(drvNow) > 0.5f;
  yawInt   = constrain(yawInt + yaw, -2000.0f, 2000.0f);
  speedTerm = ksp * speedFilt + ksi * speedInt;
  yawTerm   = kdyaw * yawFilt + kdyi * yawInt;
  if (trnTarget != 0) yawInt *= 0.9f;
  // нормализация колёс: рассинхрон копится только при езде ПРЯМО — в повороте
  // колёсам положено крутиться по-разному
  if (trnTarget == 0 && fabs(drvNow) > 0.5f)
    difInt = constrain(difInt + dif, -800.0f, 800.0f);
  difInt *= 0.998f;                              // забыто за ~20 с
  difTerm = kdif * difInt;
}
