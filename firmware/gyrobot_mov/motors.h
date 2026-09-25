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
  // дифференциал курса: yawTerm -> левому +, правому −. Только при вращении
  // моторов (p>0), на месте корректировать нечем — иначе гул
  int c = (p > 0) ? (int)constrain(yawTerm, -30.0f, 30.0f) : 0;
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

// Прямое движение WASD (аналог _speed + forward/backward/left/right оригинала):
// знаковый ШИМ на каждое колесо отдельно, баланс тут не участвует
void moveRaw(int a, int b) {
  digitalWrite(AIN1, a >= 0); digitalWrite(AIN2, a < 0);
  digitalWrite(BIN1, b >= 0); digitalWrite(BIN2, b < 0);
  analogWrite(PWMA, constrain(abs(a), 0, 255));
  analogWrite(PWMB, constrain(abs(b), 0, 255));
  pwmOut = a;
}
