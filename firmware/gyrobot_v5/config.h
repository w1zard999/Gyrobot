#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

// ---------- Пины (STATUS.md, прозвонено 2026-09-07) ----------
#define PIN_PWMA 9
#define PIN_AIN1 7
#define PIN_AIN2 8
#define PIN_PWMB 10
#define PIN_BIN1 5
#define PIN_BIN2 4
#define PIN_STBY 6
#define PIN_ECA_A 2   // INT0, энкодер A
#define PIN_ECA_B A2
#define PIN_ECB_A 3   // INT1, энкодер B
#define PIN_ECB_B A3

// ---------- Константы цикла ----------
#define CTRL_DT     (1.0f / 200.0f)   // цикл 200 Гц
#define SPEED_DIV   4                 // контур скорости 50 Гц
#define TELE_DIV    10                // телеметрия 20 Гц
#define GYR_LSB     65.5f             // ±500 °/с
#define BOOT_CYCLES 100               // 0.5 с сходимость фильтра
#define BURST_N     60
#define EEPROM_MAGIC 0x4753           // 'GS'
#define EEPROM_VER  5

struct Config {
  float kp, kd, zero, alpha, dead, maxpwm, fall;
  float arec, autoen;
  float kvi, kii, tiltmax, kpos, sfilt, kth;
  float ktp, ktd;
  float esA, esB, mvA, mvB;
  float gbx, gby, gbz, abx, aby, abz;
  float axu, axf, axg, axt;
};

#define NPARAM 31
extern Config cfg;
extern const char* PNAMES[NPARAM];
extern float* PADDR[NPARAM];

void defaults();
void saveEEP();
void loadEEP();
uint16_t crc16(const uint8_t* d, uint8_t n);

// --- межфайловые интерфейсы ---
// imu.ino
extern float imuAngle, imuGyro;
extern uint16_t imuErr;
extern bool imuBootDone;
bool imuInit();
bool imuRead();
void gyroCalib();
void rawPrint();
float gyrAxis(float sel);   // ось гироскопа со смещением и знаком, LSB
// motors.ino
extern volatile long totA, totB;
void motorsInit();
void drive(int16_t pa, int16_t pb);      // + = вперёд
void manDrive(char ch, int16_t pwm);     // raw, только при !en
// control.ino
extern bool en, fallen, motorsOn;
extern float vFilt, vRef, tiltShift, ivi, posEnc, cmdV, cmdTurn;
extern int16_t pwmA, pwmB;
extern uint32_t hzVal;
void controlReset();
void controlTick(uint16_t cyc);
void burstArm();
void burstTick();
// protocol.ino
void serialPoll();
void teleLine();
