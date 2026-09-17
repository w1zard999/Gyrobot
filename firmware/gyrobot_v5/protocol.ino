// Протокол Serial (115200): команды + телеметрия 20 Гц.
// p · <имя> <знач> · e 0/1 · s 0/1 · c · z · d <v> <turn> · t a|b <pwm> · n [0]
// raw · burst · w/l/f · io <pin> <0|1> · h
#include "config.h"

static char sbuf[36];
static uint8_t slen = 0;
bool teleOn = true;

void teleLine() {
  if (!teleOn) return;
  Serial.print(F("a=")); Serial.print(imuAngle, 2);
  Serial.print(F(" r=")); Serial.print(imuGyro, 1);
  Serial.print(F(" v=")); Serial.print(vFilt, 1);
  Serial.print(F(" vr=")); Serial.print(vRef, 1);
  Serial.print(F(" pos=")); Serial.print((long)posEnc);
  Serial.print(F(" tilt=")); Serial.print(tiltShift, 2);
  Serial.print(F(" i=")); Serial.print(ivi, 2);
  Serial.print(F(" pA=")); Serial.print(pwmA);
  Serial.print(F(" pB=")); Serial.print(pwmB);
  Serial.print(F(" en=")); Serial.print(en ? 1 : 0);
  Serial.print(F(" fall=")); Serial.print(fallen ? 1 : 0);
  Serial.print(F(" z=")); Serial.print(cfg.zero, 2);
  Serial.print(F(" hz=")); Serial.print(hzVal);
  Serial.print(F(" imuerr=")); Serial.println(imuErr);
}

static void printHelp() {
  Serial.println(F("gyrobot v5: p=params | <name> <val> | e 0/1 | s 0/1 | c=cal gyro"));
  Serial.println(F("  z=zero here | d <v> <turn> | t a|b <pwm> (e0) | n [0]=enc"));
  Serial.println(F("  raw | burst | w=save | l=load | f=defaults | io <pin> 0/1 (e0)"));
}

static void printParams() {
  for (uint8_t i = 0; i < NPARAM; i++) {
    Serial.print(PNAMES[i]); Serial.print('=');
    Serial.print(*PADDR[i], 4);
    if ((i % 6) == 5 || i == NPARAM - 1) Serial.println(); else Serial.print(' ');
  }
}

static float fv(const char* s) { return atof(s); }

static void handleLine(char* line) {
  // убрать \r
  for (char* p = line; *p; p++) if (*p == '\r') *p = 0;
  char c1[12] = "", c2[12] = "", c3[12] = "";
  int nx = sscanf(line, "%11s %11s %11s", c1, c2, c3);
  if (nx < 1) return;

  if (!strcmp(c1, "p")) { printParams(); return; }
  if (!strcmp(c1, "h")) { printHelp(); return; }
  if (!strcmp(c1, "raw")) { rawPrint(); return; }
  if (!strcmp(c1, "burst")) { burstArm(); return; }
  if (!strcmp(c1, "w")) { saveEEP(); Serial.println(F("saved")); return; }
  if (!strcmp(c1, "l")) { loadEEP(); Serial.println(F("loaded")); return; }
  if (!strcmp(c1, "f")) { defaults(); Serial.println(F("defaults")); return; }
  if (!strcmp(c1, "c")) { gyroCalib(); return; }
  if (!strcmp(c1, "z")) { cfg.zero = imuAngle; Serial.print(F("zero=")); Serial.println(cfg.zero, 3); return; }

  if (!strcmp(c1, "e")) {
    en = (nx > 1 && atoi(c2) == 1);
    if (!en) { controlReset(); }
    Serial.print(F("en=")); Serial.println(en ? 1 : 0);
    return;
  }
  if (!strcmp(c1, "s")) {
    teleOn = !(nx > 1 && atoi(c2) == 0);
    Serial.println(F("ok"));
    return;
  }
  if (!strcmp(c1, "d")) {
    cmdV = nx > 1 ? fv(c2) : 0;
    cmdTurn = nx > 2 ? fv(c3) : 0;
    Serial.print(F("d ")); Serial.print(cmdV, 1); Serial.print(' '); Serial.println(cmdTurn, 1);
    return;
  }
  if (!strcmp(c1, "t")) {
    if (en) { Serial.println(F("only e 0")); return; }
    if (nx > 2) manDrive(c2[0], (int16_t)atoi(c3));
    else Serial.println(F("t a|b <pwm>"));
    return;
  }
  if (!strcmp(c1, "n")) {
    if (nx > 1 && atoi(c2) == 0) {
      noInterrupts(); totA = 0; totB = 0; interrupts();
      controlReset(); posEnc = 0; posRef = 0;
      Serial.println(F("enc reset"));
    }
    Serial.print(F("totA=")); Serial.print(totA);
    Serial.print(F(" totB=")); Serial.println(totB);
    return;
  }
  if (!strcmp(c1, "io")) {
    if (en) { Serial.println(F("only e 0")); return; }
    if (nx > 2) {
      int pin = atoi(c2), val = atoi(c3);
      pinMode(pin, OUTPUT); digitalWrite(pin, val ? HIGH : LOW);
      Serial.println(F("ok"));
    }
    return;
  }

  // установка параметра по имени
  if (nx > 1) {
    for (uint8_t i = 0; i < NPARAM; i++) {
      if (!strcmp(c1, PNAMES[i])) {
        *PADDR[i] = fv(c2);
        Serial.print(PNAMES[i]); Serial.print('='); Serial.println(*PADDR[i], 4);
        return;
      }
    }
  }
  Serial.println(F("what?"));
}

void serialPoll() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      sbuf[slen] = 0;
      handleLine(sbuf);
      slen = 0;
    } else if (slen < sizeof(sbuf) - 1) {
      sbuf[slen++] = ch;
    }
  }
}
