// TB6612: направление, STBY, ШИМ 3.9 кГц (Timer1 /8) и развёртка PID -> ШИМ.
#pragma once
#include "config.h"

int pwmOut = 0;                        // последний ШИМ (телеметрия x=)
float pmin = 16.0f;                    // стартовый ШИМ моторов: 16 трясёт корпус,
                                       // крутится по сериалу "pmin 10"
float atr = 16.0f;                     // мёртвая зона мотора A (встает 16 vs 12 у B):
float brt = 15.0f;                     // прибавка сверху команды, только при вращении

void motorsInit() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  TCCR1B = (TCCR1B & 0xF8) | 0x02;     // D9/D10: ШИМ 3.9 кГц (Timer1, делитель 8)
}

static void setMotor(uint8_t p1, uint8_t p2, uint8_t pw, int v, float trim) {
  bool fwd = v >= 0;
  digitalWrite(p1, fwd); digitalWrite(p2, !fwd);
  int p = abs(v);
  if (p > 0) p += (int)trim;           // компенсация мёртвой зоны: только при вращении
  analogWrite(pw, constrain(p, 0, 255));
}

void drive(int pwm) {                  // >0 вперёд, <0 назад, 0 стоп
  pwmOut = pwm;
  float yawEff = (trnTarget != 0) ? 0.0f : yawTerm;   // рулит водитель — курс молчит
  int c = (int)constrain(yawEff + trnNow, -60.0f, 60.0f);
  int vA = pwm + c;                    // по-колёсный знаковый привод: разворот
  int vB = pwm - c;                    // на месте работает, ничто не клампится в 0
  setMotor(AIN1, AIN2, PWMA, vA, atr);
  setMotor(BIN1, BIN2, PWMB, vB, brt);
}

// Развёртка PID -> ШИМ. У оригинала map(pid*40, 28, 200, 100, 255) под его
// моторы и 490 Гц; у нас мёртвая зона 16/12 на 3.9 кГц (STATUS.md), поэтому
// нижняя граница PWM_MIN=16 и потолок входа 320 (pid=8).
int pwmFromPid(float pid) {
  long x = map((long)(pid * 40.0f), 16, 320, (long)pmin, 255);
  return (int)constrain(x, pmin, 255);
}
