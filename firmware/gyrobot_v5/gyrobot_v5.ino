// Главный файл скетча gyrobot v5: цикл 200 Гц, машина состояний.
#include "config.h"

Config cfg;
const char* PNAMES[NPARAM] = {
  "kp","kd","zero","alpha","dead","maxpwm","fall","arec","autoen",
  "kvi","kii","tiltmax","kpos","sfilt","kth","ktp","ktd",
  "esA","esB","mvA","mvB",
  "gbx","gby","gbz","abx","aby","abz","axu","axf","axg","axt"
};
float* PADDR[NPARAM];

static uint32_t tCtrl = 0;
static uint16_t cyc = 0;
static uint32_t hzCnt = 0, hzT = 0;
uint32_t hzVal = 0;
static bool lastMotorsOn = false;

void defaults() {
  cfg.kp = 12; cfg.kd = 0.30f; cfg.zero = 0; cfg.alpha = 0.98f;
  cfg.dead = 12; cfg.maxpwm = 200; cfg.fall = 35;
  cfg.arec = 1; cfg.autoen = 0;                       // моторы не оживут до проверки знаков
  cfg.kvi = 0; cfg.kii = 0; cfg.tiltmax = 5; cfg.kpos = 0;
  cfg.sfilt = 0.85f; cfg.kth = 0;
  cfg.ktp = 0; cfg.ktd = 0;
  cfg.esA = 1; cfg.esB = -1; cfg.mvA = -1; cfg.mvB = -1;   // живые значения 2026-09-17
  cfg.gbx = 319; cfg.gby = 46; cfg.gbz = 33;
  cfg.abx = 234; cfg.aby = -14808; cfg.abz = 426;
  cfg.axu = 3; cfg.axf = 2; cfg.axg = 1; cfg.axt = 3;
}

struct EEBlob {
  uint16_t magic; uint8_t ver; uint8_t n;
  float vals[NPARAM];
  uint16_t crc;
};

uint16_t crc16(const uint8_t* d, uint8_t n) {
  uint16_t c = 0xFFFF;
  for (uint8_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

void buildPaddr() {
  float* base[] = {
    &cfg.kp,&cfg.kd,&cfg.zero,&cfg.alpha,&cfg.dead,&cfg.maxpwm,&cfg.fall,&cfg.arec,&cfg.autoen,
    &cfg.kvi,&cfg.kii,&cfg.tiltmax,&cfg.kpos,&cfg.sfilt,&cfg.kth,&cfg.ktp,&cfg.ktd,
    &cfg.esA,&cfg.esB,&cfg.mvA,&cfg.mvB,
    &cfg.gbx,&cfg.gby,&cfg.gbz,&cfg.abx,&cfg.aby,&cfg.abz,&cfg.axu,&cfg.axf,&cfg.axg,&cfg.axt
  };
  for (uint8_t i = 0; i < NPARAM; i++) PADDR[i] = base[i];
}

void saveEEP() {
  EEBlob e; e.magic = EEPROM_MAGIC; e.ver = EEPROM_VER; e.n = NPARAM;
  const float* base[] = {
    &cfg.kp,&cfg.kd,&cfg.zero,&cfg.alpha,&cfg.dead,&cfg.maxpwm,&cfg.fall,&cfg.arec,&cfg.autoen,
    &cfg.kvi,&cfg.kii,&cfg.tiltmax,&cfg.kpos,&cfg.sfilt,&cfg.kth,&cfg.ktp,&cfg.ktd,
    &cfg.esA,&cfg.esB,&cfg.mvA,&cfg.mvB,
    &cfg.gbx,&cfg.gby,&cfg.gbz,&cfg.abx,&cfg.aby,&cfg.abz,&cfg.axu,&cfg.axf,&cfg.axg,&cfg.axt
  };
  for (uint8_t i = 0; i < NPARAM; i++) e.vals[i] = *base[i];
  e.crc = crc16((const uint8_t*)e.vals, sizeof(e.vals));
  EEPROM.put(0, e);
}

void loadEEP() {
  defaults();
  buildPaddr();
  EEBlob e;
  EEPROM.get(0, e);
  if (e.magic != EEPROM_MAGIC || e.ver != EEPROM_VER || e.n != NPARAM) return;
  if (e.crc != crc16((const uint8_t*)e.vals, sizeof(e.vals))) return;
  float* base[] = {
    &cfg.kp,&cfg.kd,&cfg.zero,&cfg.alpha,&cfg.dead,&cfg.maxpwm,&cfg.fall,&cfg.arec,&cfg.autoen,
    &cfg.kvi,&cfg.kii,&cfg.tiltmax,&cfg.kpos,&cfg.sfilt,&cfg.kth,&cfg.ktp,&cfg.ktd,
    &cfg.esA,&cfg.esB,&cfg.mvA,&cfg.mvB,
    &cfg.gbx,&cfg.gby,&cfg.gbz,&cfg.abx,&cfg.aby,&cfg.abz,&cfg.axu,&cfg.axf,&cfg.axg,&cfg.axt
  };
  for (uint8_t i = 0; i < NPARAM; i++) *base[i] = e.vals[i];
}

void setup() {
  Serial.begin(115200);
  loadEEP();
  if (!imuInit()) Serial.println(F("IMU FAIL"));
  motorsInit();
  en = cfg.autoen > 0.5f;
  controlReset();
  Serial.println(F("gyrobot v5"));
}

void loop() {
  uint32_t now = micros();
  if (now - tCtrl >= 5000) {           // 200 Гц
    tCtrl += 5000;
    imuRead();
    cyc++;
    hzCnt++;
    if (now - hzT >= 1000000UL) { hzVal = hzCnt; hzCnt = 0; hzT = now; }
    controlTick(cyc);

    // --- вывод на моторы: включены только в стойке ---
    if (motorsOn) {
      drive(pwmA, pwmB);
      lastMotorsOn = true;
    } else {
      if (lastMotorsOn) { drive(0, 0); lastMotorsOn = false; }
    }

    if (cyc % TELE_DIV == 0) teleLine();
    burstTick();
  }
  serialPoll();
}
