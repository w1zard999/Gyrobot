// TB6612: ШИМ 3.9 кГц (Timer1 /8), направление, STBY, мёртвая зона.
// Энкодеры: INT0/INT1 по фронту канала A, направление по каналу B (счёт x1).
#include "config.h"

volatile long totA = 0, totB = 0;

static void isrA() { totA += (digitalRead(PIN_ECA_B) == HIGH) ? 1 : -1; }
static void isrB() { totB += (digitalRead(PIN_ECB_B) == HIGH) ? 1 : -1; }

void motorsInit() {
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  pinMode(PIN_ECA_A, INPUT_PULLUP); pinMode(PIN_ECA_B, INPUT_PULLUP);
  pinMode(PIN_ECB_A, INPUT_PULLUP); pinMode(PIN_ECB_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ECA_A), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ECB_A), isrB, RISING);
  analogWrite(PIN_PWMA, 0); analogWrite(PIN_PWMB, 0);
  TCCR1B = (TCCR1B & 0xF8) | 0x02;   // Timer1 /8 → ~3.9 кГц на D9/D10
}

static void chWrite(uint8_t in1, uint8_t in2, uint8_t pwmPin, int16_t v, int8_t mv) {
  v *= mv;
  int16_t mag = (v < 0) ? -v : v;
  if (mag > 0 && mag < (int16_t)cfg.dead) mag = (int16_t)cfg.dead;
  if (mag > (int16_t)cfg.maxpwm) mag = (int16_t)cfg.maxpwm;
  if (v > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (v < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else            { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }
  analogWrite(pwmPin, mag);
}

void drive(int16_t pa, int16_t pb) {
  chWrite(PIN_AIN1, PIN_AIN2, PIN_PWMA, pa, (int8_t)cfg.mvA);
  chWrite(PIN_BIN1, PIN_BIN2, PIN_PWMB, pb, (int8_t)cfg.mvB);
}

// Прогон мотора командой t: чистый ШИМ без знаков и мёртвой зоны.
void manDrive(char ch, int16_t pwm) {
  int16_t mag = pwm < 0 ? -pwm : pwm;
  if (mag > 255) mag = 255;
  if (ch == 'a') {
    if (pwm > 0)      { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW); }
    else if (pwm < 0) { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); }
    else              { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, LOW); }
    analogWrite(PIN_PWMA, mag);
  } else {
    if (pwm > 0)      { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW); }
    else if (pwm < 0) { digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); }
    else              { digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, LOW); }
    analogWrite(PIN_PWMB, mag);
  }
}
