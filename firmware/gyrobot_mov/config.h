#pragma once
#include <Arduino.h>
#include <Wire.h>

// ---------- Пины (STATUS.md, прозвонено 2026-09-07) ----------
#define PWMA    9     // TB6612 PWMA, A = левое колесо
#define AIN1    7
#define AIN2    8
#define PWMB    10    // B = правое
#define BIN1    5
#define BIN2    4
#define STBY    6
#define PIN_ECA_A 2   // энкодер A фаза A (INT0)
#define PIN_ECA_B A2  // энкодер A фаза B
#define PIN_ECB_A 3   // энкодер B фаза A (INT1)
#define PIN_ECB_B A3  // энкодер B фаза B
#define MPU_ADDR  0x68

// ---------- Регулятор (значения оригинала, пересчитанные «за секунду») ----------
// У оригинала цикл ~500 Гц и константы «за итерацию»: ki=0.01 -> KI_S=5.0/с,
// подстройка нуля 0.008 -> ZR_RATE=4.0/с, feedforward 0.016 -> FF_RATE=8.0/с,
// alfa=0.001 -> TAU_ACC=2.0 с. KP/KD — наши. KD честная = половина старой 0.02:
// до фикса шкалы гироскопа rate был x2, и 0.02 действовал как 0.04.
#define DRIVE_SIGN (-1)   // если робот ловит наклон, уезжая В СТОРОНУ падения, — поменять
#define KP_DEF 0.5f
#define KD_DEF 0.007f     // подбор 2026-09-21: 0.02 -> ... -> 0.001 (мало демпфировала) -> 0.007
#define KI_DEF 5.0f
#define DEAD_DEF 0.4f     // |выход| меньше — моторы выключены
#define FALL_DEG 30.0f
#define TAU_ACC 2.0f      // тяга фильтра к акселю, с
#define ZR_RATE 4.0f      // скорость адаптивного нуля, 1/с
#define FF_RATE 8.0f      // feedforward нуля, 1/с
#define PWM_MIN 16        // мёртвая зона на 3.9 кГц: 16/12 (STATUS.md)

// Живые значения (крутятся по сериалу без перепрошивки: "kd 0.005", "p" — показать)
// Живые значения (крутятся по сериалу без перепрошивки: "kd 0.005", "p" — показать)
extern float kp, kd, ki_s, pid_dead, imax;   // живые (сериал)
extern float pmin;                           // стартовый ШИМ моторов (сериал)
extern float atr;                            // выравнивание момента мотора A (сериал)
extern float atrb;                           // то же при движении назад (сериал)
extern float drv, trn;                       // WASD: наклон и поворот (сериал)
extern float trnTarget;                      // угасание yawInt в speed.h
// Энкодерный контур (тоже живые: ksp/ksi — вклад, esa/esb — знаки каналов)
extern float esA, esB, ksp, ksi;
extern float kdyaw, kdyi;                    // курс: демпфер вращения + удержание
void paramsPoll();
// speed.h
extern volatile long encA, encB;
extern float speedFilt, speedInt, speedTerm;
extern float yawFilt, yawInt, yawTerm;
void speedInit();
void speedTick();

// ---------- MPU-6050 ----------
#define ABX 234.0f        // смещения акселя, LSB (STATUS.md)
#define ABY (-14808.0f)
#define ABZ 426.0f
#define GYR_LSB 65.5f     // шкала ±500 °/с (пишется в 0x1B при старте!)

// --- межфайловые интерфейсы ---
// imu.h
extern int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
extern float gyroBiasX;
bool mpuRead();           // false = сбой I2C, цикл пропустить
void mpuWake();           // будим MPU + шкала ±500
void gyroCalib(uint16_t ms);
float accAngle();         // + = наклон вперёд
// motors.h
extern int pwmOut;
void motorsInit();
void drive(int pwm);      // >0 вперёд, <0 назад, 0 стоп
int pwmFromPid(float pid);
// gyrobot_mov.ino
extern float balancing_zerro;
extern bool armed;
