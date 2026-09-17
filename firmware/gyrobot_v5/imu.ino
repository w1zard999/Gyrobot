// MPU6050: чтение, смещения из cfg, комплементарный фильтр.
// Угол: 0 = стойка, растёт при наклоне вперёд (axf = «вперёд», axu = «вверх»).
// Тилт-гироскоп = ось axg (знак совпадает с ростом угла), рыскание = axt.
#include "config.h"

float imuAngle = 0, imuGyro = 0;   // °, °/с
uint16_t imuErr = 0;
int16_t rax, ray, raz, rgx, rgy, rgz;   // сырые значения последнего чтения

static float bootSum = 0;
static uint16_t bootN = 0;
bool imuBootDone = false;

static bool wr8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg); Wire.write(val);
  uint8_t r = Wire.endTransmission();
  if (r != 0) { imuErr++; Wire.clearWireTimeoutFlag(); return false; }
  return true;
}

bool imuInit() {
  Wire.begin();
  Wire.setWireTimeout(25000, true);
  delay(5);
  if (!wr8(0x6B, 0x00)) return false;   // выход из сна
  wr8(0x1A, 0x03);                      // DLPF ~44/42 Гц
  wr8(0x1B, 0x08);                      // гироскоп ±500 °/с
  wr8(0x1C, 0x00);                      // акселерометр ±2g
  return true;
}

// Ось со смещением и знаком: sel = ±1/±2/±3
static float accAxis(float sel) {
  float s = (sel < 0) ? -1.0f : 1.0f;
  int8_t a = (int8_t)(sel < 0 ? -sel : sel);
  float v, b;
  switch (a) {
    case 1:  v = rax; b = cfg.abx; break;
    case 2:  v = ray; b = cfg.aby; break;
    default: v = raz; b = cfg.abz;
  }
  return s * (v - b);
}
float gyrAxis(float sel) {
  float s = (sel < 0) ? -1.0f : 1.0f;
  int8_t a = (int8_t)(sel < 0 ? -sel : sel);
  float v, b;
  switch (a) {
    case 1:  v = rgx; b = cfg.gbx; break;
    case 2:  v = rgy; b = cfg.gby; break;
    default: v = rgz; b = cfg.gbz;
  }
  return s * (v - b);
}

bool imuRead() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) { imuErr++; Wire.clearWireTimeoutFlag(); return false; }
  uint8_t n = Wire.requestFrom((uint8_t)0x68, (uint8_t)14);
  if (n != 14 || Wire.getWireTimeoutFlag()) {
    imuErr++; Wire.clearWireTimeoutFlag(); return false;
  }
  uint8_t b[14];
  for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();
  rax = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  ray = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
  raz = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
  rgx = (int16_t)(((uint16_t)b[8] << 8) | b[9]);
  rgy = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
  rgz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);

  float fwd = accAxis(cfg.axf);
  float up  = accAxis(cfg.axu);
  imuGyro   = gyrAxis(cfg.axg) / GYR_LSB;
  float aAcc = atan2(fwd, up) * 57.29578f;

  if (bootN < BOOT_CYCLES) {
    bootSum += aAcc; bootN++;
    if (bootN == BOOT_CYCLES) { imuAngle = bootSum / BOOT_CYCLES; imuBootDone = true; }
    return true;
  }
  imuAngle = cfg.alpha * (imuAngle + imuGyro * CTRL_DT) + (1.0f - cfg.alpha) * aAcc;
  return true;
}

void gyroCalib() {   // 3 с неподвижно, моторы выключить
  drive(0, 0);
  Serial.println(F("calib 3s still..."));
  long sx = 0, sy = 0, sz = 0;
  const uint16_t N = 600;
  uint32_t t0 = micros();
  for (uint16_t i = 0; i < N; i++) {
    while (micros() - t0 < (uint32_t)(i + 1) * 5000UL) {}
    imuRead();
    sx += rgx; sy += rgy; sz += rgz;
  }
  cfg.gbx = (float)sx / N; cfg.gby = (float)sy / N; cfg.gbz = (float)sz / N;
  Serial.print(F("gbx=")); Serial.print(cfg.gbx, 1);
  Serial.print(F(" gby=")); Serial.print(cfg.gby, 1);
  Serial.print(F(" gbz=")); Serial.println(cfg.gbz, 1);
  Serial.println(F("do: w"));
}

void rawPrint() {
  Serial.print(F("a "));
  Serial.print(rax); Serial.print(' '); Serial.print(ray); Serial.print(' '); Serial.print(raz);
  Serial.print(F(" |a|="));
  float m = sqrt((float)rax * rax + (float)ray * ray + (float)raz * raz) / 16384.0f;
  Serial.print(m, 3); Serial.print(F("g g "));
  Serial.print(rgx); Serial.print(' '); Serial.print(rgy); Serial.print(' '); Serial.println(rgz);
}
