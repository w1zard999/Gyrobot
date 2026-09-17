// Два контура + машина состояний.
// Быстрый (200 Гц): PD по углу, D = гироскоп. Выход — ШИМ.
// Медленный (50 Гц): PI по скорости колёс → смещение уставки угла (±tiltmax),
// P по позиции → уставка скорости. Ссылка позиции двигается только командой d.
#include "config.h"

bool en = false, fallen = true, motorsOn = false;
float vFilt = 0, vRef = 0, tiltShift = 0, ivi = 0;
float posEnc = 0, posRef = 0, cmdV = 0, cmdTurn = 0;
int16_t pwmA = 0, pwmB = 0;

static long lastTotA = 0, lastTotB = 0;
static bool wasFallen = true;
static bool fellLatch = false;
static uint32_t uprightMs = 0, lastMs = 0;

// burst-захват
static int16_t burstBuf[BURST_N][3];
static uint16_t burstCnt = 0;
static volatile bool burstOn = false, burstReady = false;

void controlReset() {
  ivi = 0; vFilt = 0; tiltShift = 0;
  lastTotA = totA; lastTotB = totB;
  posRef = posEnc;
  cmdV = 0; cmdTurn = 0;
}

void controlTick(uint16_t cyc) {
  uint32_t nowMs = millis();
  float dtMs = (float)(nowMs - lastMs);
  if (dtMs > 50) dtMs = 50;
  lastMs = nowMs;

  // --- падение ---
  bool nowFallen = fabs(imuAngle - cfg.zero) > cfg.fall;
  if (nowFallen && !wasFallen) {
    controlReset();
    if (cfg.arec < 0.5f) fellLatch = true;   // arec=0: после падения только e 1
  }
  if (!en) fellLatch = false;
  wasFallen = nowFallen;
  fallen = nowFallen;

  // --- разрешение моторов: en + стойка 500 мс + фильтр сошёлся ---
  if (!fallen && en) uprightMs += (uint32_t)dtMs; else uprightMs = 0;
  motorsOn = en && !fallen && uprightMs >= 500 && imuBootDone && !burstOn && !fellLatch;

  // --- медленная часть: 50 Гц ---
  if (cyc % SPEED_DIV == 0) {
    noInterrupts();
    long a = totA, b = totB;
    interrupts();
    float da = (a - lastTotA) * cfg.esA;
    float db = (b - lastTotB) * cfg.esB;
    lastTotA = a; lastTotB = b;
    float vRaw = (da + db) * 0.5f;                     // тиков за период
    vFilt = cfg.sfilt * vFilt + (1.0f - cfg.sfilt) * vRaw;
    float vBody = vFilt - cfg.kth * imuGyro;
    posEnc += vRaw;
    if (motorsOn) posRef += cmdV;                      // путь растёт только от команды
    vRef = -cfg.kpos * (posEnc - posRef) + cmdV;
    float err = vRef - vBody;
    if (err > 20) err = 20; if (err < -20) err = -20;
    ivi += cfg.kii * err;
    if (ivi > cfg.tiltmax) ivi = cfg.tiltmax;
    if (ivi < -cfg.tiltmax) ivi = -cfg.tiltmax;
    tiltShift = -(cfg.kvi * err + ivi);                // едет вперёд → наклон назад
    if (tiltShift > cfg.tiltmax) tiltShift = cfg.tiltmax;
    if (tiltShift < -cfg.tiltmax) tiltShift = -cfg.tiltmax;
  }

  // --- быстрая часть: 200 Гц ---
  float setp = cfg.zero + tiltShift;
  float u = cfg.kp * (imuAngle - setp) - cfg.kd * imuGyro;   // + = вперёд
  float diff = cmdTurn + cfg.ktd * (-gyrAxis(cfg.axt) / GYR_LSB);
  pwmA = (int16_t)constrain(u + diff, -255.0f, 255.0f);
  pwmB = (int16_t)constrain(u - diff, -255.0f, 255.0f);

  // --- burst-захват ---
  if (burstOn) {
    burstBuf[burstCnt][0] = (int16_t)(imuAngle * 100);
    burstBuf[burstCnt][1] = (int16_t)(imuGyro * 10);
    burstBuf[burstCnt][2] = pwmA;
    burstCnt++;
    if (burstCnt >= BURST_N) { burstOn = false; burstReady = true; }
  }
}

void burstArm() {
  drive(0, 0);
  burstCnt = 0; burstOn = true; burstReady = false;
}

void burstTick() {   // выгрузка после заполнения
  if (!burstReady) return;
  burstReady = false;
  for (uint16_t i = 0; i < BURST_N; i++) {
    Serial.print(F("b ")); Serial.print(i);
    Serial.print(' '); Serial.print(burstBuf[i][0] * 0.01f, 2);
    Serial.print(' '); Serial.print(burstBuf[i][1] * 0.1f, 1);
    Serial.print(' '); Serial.println(burstBuf[i][2]);
  }
}
