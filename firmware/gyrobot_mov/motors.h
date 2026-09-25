// TB6612: направление, STBY, ШИМ 3.9 кГц (Timer1 /8) и развёртка PID -> ШИМ.
#pragma once
#include "config.h"

int pwmOut = 0;                          // последний ШИМ (телеметрия x=)
float pmin = 16.0f;                      // стартовый ШИМ моторов: 16 трясёт корпус,
                                         // крутится по сериалу "pmin 10"
float atr = 0.0f;                        // статические добавки выключены: курсом
float atrb = 0.0f;                       // теперь владеет контур kdy/kdi

void motorsInit() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  TCCR1B = (TCCR1B & 0xF8) | 0x02;     // D9/D10: ШИМ 3.9 кГц (Timer1, делитель 8)
}

void drive(int pwm) {                  // >0 вперёд, <0 назад, 0 стоп
  pwmOut = pwm;
  bool fwd = pwm >= 0;
  int p = constrain(abs(pwm), 0, 255);
  digitalWrite(AIN1, fwd); digitalWrite(AIN2, !fwd);
  digitalWrite(BIN1, fwd); digitalWrite(BIN2, !fwd);
  // дифференциал: yawTerm (курс) + trnNow (водитель). Кап ±60. Без ворот p>0:
  // при p=0 дифференциал = поворот на месте — штатно
  int c = (int)constrain(yawTerm + trnNow, -60.0f, 60.0f);
  analogWrite(PWMA, constrain(p + (int)(fwd ? atr : atrb) + c, 0, 255));
  analogWrite(PWMB, constrain(p - c, 0, 255));
}

// Развёртка PID -> ШИМ. У оригинала map(pid*40, 28, 200, 100, 255) под его
// моторы и 490 Гц; у нас мёртвая зона 16/12 на 3.9 кГц (STATUS.md), поэтому
// нижняя граница PWM_MIN=16 и потолок входа 320 (pid=8).
int pwmFromPid(float pid) {
  long x = map((long)(pid * 40.0f), 16, 320, (long)pmin, 255);
  return (int)constrain(x, pmin, 255);
}
