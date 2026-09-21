// MPU-6050: чтение 14 байт с таймаутом, угол крена, калибровка bias.
// Оси нашей платы: вперёд=+Y, вверх=+Z, гироскоп крена=X (STATUS.md).
#pragma once
#include "config.h"

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float gyroBiasX = 0;

void mpuWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}

void mpuWake() {
  mpuWrite8(0x6B, 0);    // будим
  delay(100);
  mpuWrite8(0x1B, 0x08); // шкала ±500 °/с под GYR_LSB=65.5. Дефолт ±250 (131 LSB)
                         // даёт rate x2 -> тряску (найдено 2026-09-21 по burst)
}

bool mpuRead() {                         // false = сбой I2C, цикл пропустить
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                    // читаем 14 байт: акксель, темп, гироскоп
  Wire.endTransmission(false);
  uint8_t n = Wire.requestFrom(MPU_ADDR, 14, true);
  if (n < 14 || Wire.getWireTimeoutFlag()) {   // зависание шины от помех моторов
    Wire.clearWireTimeoutFlag();
    return false;
  }
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();            // температура — не нужна
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  return true;
}

void gyroCalib(uint16_t ms) {            // робот должен стоять неподвижно
  long sum = 0; int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (mpuRead()) { sum += GyX; n++; }
    delay(2);
  }
  gyroBiasX = n ? (float)sum / n : 0;
}

float accAngle() {                       // + = наклон вперёд (как 'a' в v3)
  return atan2(AcY - ABY, AcZ - ABZ) * 57.29578f;
}
