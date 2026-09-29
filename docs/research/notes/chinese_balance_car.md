# Chinese 平衡小车 (two-wheel balance car) control: 平衡小车之家 / minibalance and its Arduino ports

Scope: DC gear motor + encoder + TB6612 + MPU6050 cars. Primary sources are actual source files pulled from GitHub (the 平衡小车之家 STM32 `control.c`, an Arduino port of the 平衡小车之家 code, and the Yahboom BST-ABC Arduino code), plus CSDN/知乎 tuning write-ups. Where I say "verified in source" I read the file myself (downloaded raw).

Primary code sources used throughout:
- 平衡小车之家 STM32 original, `MiniBalance/CONTROL/control.c` (header: `作者：平衡小车之家`), defaults in `USER/main.c`, filters in `MiniBalance/filter/filter.c` — [zycczy/minibalance control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c), [main.c](https://github.com/zycczy/minibalance/blob/master/USER/main.c), [filter.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/filter/filter.c)
- Arduino port of 平衡小车之家 code (functions still labelled `作者：平衡小车之家`), Nano-class, MsTimer2, TB6612, INT0 + PinChangeInt encoders, Bluetooth serial — [jfboy/BalanceCar balance_car_2.0.ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino) ("基于Arduino平台实现的两轮平衡小车")
- Yahboom "BST-ABC ver 1.2" Arduino balance car (Kalman, MsTimer2 5 ms, WASD-style serial `w/s/q/e/a/d` commands), mirrored at [GaloisInc/lean4-balance-car ArduinoBalanceCar.ino](https://github.com/GaloisInc/lean4-balance-car/blob/main/ArduinoBalanceCar/ArduinoBalanceCar.ino)
- Other Arduino repos found (not inspected in detail): [kukushu/BalanceCar](https://github.com/kukushu/BalanceCar) ("arduino实现平衡小车"), [wantingqiufeng/PID-balancing-car](https://github.com/wantingqiufeng/PID-balancing-car), [CharlieLeee/SelfBlanceBot](https://github.com/CharlieLeee/SelfBlanceBot); Nano+TB6612+MPU6050 (English): [CASE-Association/CBR-1](https://github.com/CASE-Association/CBR-1); kit docs: [Keyestudio KS0193](https://docs.keyestudio.com/projects/KS0193/en/latest/index.html), [OSOYOO balance car lesson 3 (Chinese)](https://osoyoo.com/zh/2018/03/28/arduino-balance-car-lesson-3/)

---

## Q1. Exact formulas: 直立环 PD, 速度环 PI, 转向环, combination into PWM, sign conventions

### Takeaway
Three independent loops whose PWM outputs are simply **summed**: `Moto1 = Balance + Velocity − Turn`, `Moto2 = Balance + Velocity + Turn`. Balance is PD on (angle − 中值) and raw gyro; velocity is PI on a low-passed **sum** of both encoders (the integral is wheel position); turn is P on a ramped turn target plus D on gyro Z. All three are computed in the same 5/10 ms interrupt, then clamped and written to the TB6612.

### Cited Findings
- **Original STM32 control ISR (verified in source):** the ISR is triggered by the MPU6050 INT pin every 5 ms; odd ticks only read the IMU, even ticks (10 ms) do control: "`10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据`", then:
  ```c
  Encoder_Left=Read_Encoder(2);  Encoder_Right=Read_Encoder(4);
  Get_Angle(Way_Angle);
  Balance_Pwm =balance(Angle_Balance,Gyro_Balance);          //===平衡PID控制
  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);          //===速度环PID控制 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
  Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn);   //===转向环PID控制
  Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                    //===计算左轮电机最终PWM
  Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                    //===计算右轮电机最终PWM
  Xianfu_Pwm();                                               //===PWM限幅 (±8200 of 8400)
  if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right)) Flag_Stop=1;
  if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right)) Flag_Stop=0;
  if(Turn_Off(Angle_Balance,Voltage)==0) Set_Pwm(Moto1,Moto2);
  ```
  — [minibalance control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- **直立环 PD (verified):**
  ```c
  int balance(float Angle,float Gyro)
  {  float Bias; int balance;
     Bias=Angle-Zhongzhi;                       //===求出平衡的角度中值 和机械相关
     balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===PD控制
     return balance; }
  ```
  Angle is in degrees; `Gyro` is the **raw** MPU6050 gyro X count (not divided; the code sets `Gyro_Balance=Gyro_X` before `Gyro_X=Gyro_X/16.4f`, i.e. ±2000 °/s range, 16.4 LSB/(°/s)), which is why Kd is ~1 while Kp is ~300. — [control.c Get_Angle](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- **Default gains of the STM32 original:** `float Balance_Kp=300,Balance_Kd=1,Velocity_Kp=80,Velocity_Ki=0.4;` and `float Zhongzhi=1; //机械中值`, `u8 Way_Angle=2; //1：四元数(DMP) 2：卡尔曼 3：互补滤波` — [minibalance main.c](https://github.com/zycczy/minibalance/blob/master/USER/main.c). Note Velocity_Ki = Velocity_Kp/200 exactly.
- **速度环 PI (verified, STM32 original):**
  ```c
  Encoder_Least =(encoder_left+encoder_right)-0;   //===测量速度（左右编码器之和）-目标速度（此处为零）
  Encoder *= 0.8f;                                 //===一阶低通滤波器
  Encoder += Encoder_Least*0.2f;                   //===一阶低通滤波器
  Encoder_Integral +=Encoder;                      //===积分出位移 积分时间：10ms
  Encoder_Integral=Encoder_Integral-Movement;      //===接收遥控器数据，控制前进后退
  if(Encoder_Integral>10000)  Encoder_Integral=10000;   //===积分限幅
  if(Encoder_Integral<-10000) Encoder_Integral=-10000;
  Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;
  if(Flag_Hover==1) Zhongzhi=-Encoder/10-Encoder_Integral/300;   //这些斜坡悬停使用的算法
  if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0; //===电机关闭后清除积分
  ```
  — [control.c velocity()](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- **转向环 (verified, STM32 original):** `Kp=52, Kd=0` normally, `Kd=0.5` only while driving forward/back ("转向的时候取消陀螺仪的纠正 有点模糊PID的思想"); `Turn=-Turn_Target*Kp-gyro*Kd;` with `Turn_Target` ramped by `Turn_Convert` (0.6–3, scaled as `50/|encoder sum at start of turn|`) and clamped to `Turn_Amplitude=100/Flag_sudu`; `Turn_Target=0` when no turn key — [control.c turn()](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- **Motor output (verified):** `if(moto1>0) AIN2=0,AIN1=1; else AIN2=1,AIN1=0; PWMA=myabs(moto1)*1.17;` (B side has opposite pin order because the motor is mirrored) and `int Amplitude=8200; //===PWM满幅是8400 限制在8200` — [control.c Set_Pwm/Xianfu_Pwm](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- **Arduino port (jfboy, verified):** same functions, Arduino scale:
  ```c
  float Balance_Kp= 15,  Balance_Kd=0.5;           // 直立PD
  float Velocity_Kp= -2.4,  Velocity_Ki= -0.012;   // 速控PI，参考 kp = 2, ki = kp / 200;
  float Turn_Kp = 2, Turn_Kd = 0.001;              // 转向PD
  #define ZHONGZHI -8                              //小车的机械中值
  ...
  Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;  //直立速度转向环的叠加
  Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;
  int Amplitude = 250;  //===PWM满幅是255 限制在250
  if(Angle > 40 || Angle < -50) Motor1=0,Motor2=0;
  ```
  Here the velocity gains are negative **and** subtracted, i.e. the same net sign as the STM32 version; the author left commented-out staged versions: `Motor = Balance_Pwm` (纯直立调节), then `Balance_Pwm - Velocity_Pwm` (直立速控调节), then all three (三环融合). Turn: `Turn = - Turn_Target * Turn_Kp + gyro * Turn_Kd;` with `Turn_Target += Turn_Convert(=1)` per call, clamped to `Turn_Amplitude = 40`. — [jfboy balance_car_2.0.ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino)
- **Yahboom BST-ABC Arduino (verified):** `car.angleoutput = kp * (angle + angle0) + kd * Gyro_x;` (`kp 38, kd 0.58`, Gyro_x in °/s = `(gx - 128.1)/131`), speed PI `output = ki_speed*(0.0 - car.positions) + kp_speed*(0.0 - speeds_filter)` (`kp_speed 3.1, ki_speed 0.05`), turn `turnout_put = -car.turnout * kp_turn - Gyro_z * kd_turn` (`kp_turn 28, kd_turn 0.29`), combination `car.pwm1 = -car.angleoutput - speedoutput - turnoutput; car.pwm2 = -car.angleoutput - speedoutput + turnoutput;` clamped ±255 — [Yahboom code mirror](https://github.com/GaloisInc/lean4-balance-car/blob/main/ArduinoBalanceCar/ArduinoBalanceCar.ino)

### Inferences
- Sign conventions differ between codebases (STM32: `+Balance +Velocity`; jfboy: `+Balance −Velocity` with negative gains; Yahboom: `−angle −speed` with speed written as `(0 − x)`). The invariant that all share: **when the wheels roll in direction X, the velocity term adds PWM in direction X (same direction as the balance term that caused the roll)**. Signs must be fixed experimentally (see Q4), not copied.
- Gyro units matter for Kd: STM32 uses raw LSB at ±2000 °/s (Kd≈1 ≈ 16.4 PWM-counts per °/s at an 8400 scale); Yahboom uses °/s (Kd 0.58 at a 255 scale).

### Gaps
- Official 平衡小车之家 PDF manual not retrieved; the "13-line × 30:1, 4x" encoder spec quoted in the task was not confirmed from a primary source (the STM32 code does use timer encoder mode `TIM_EncoderMode_TI12` = 4x — [encoder.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance_HARDWARE/ENCODER/encoder.c)).

---

## Q2. Why the speed loop is "positive feedback" and why a precise 机械中值 is not needed

### Takeaway
The speed loop is really the **outer loop of a cascade**: its PI output is a *target tilt angle* for the balance loop. When you substitute it into the balance PD and expand, the velocity term appears with the "wrong-looking" sign—so in the flat code it looks like positive feedback (wheel rolls forward → push wheels further forward, which tilts the body back and then decelerates). Its integral term (wheel position) is an integrator that settles at whatever tilt makes net drive zero, so it automatically absorbs 中值 error and CoG shifts.

### Cited Findings
- The STM32 source comment itself: "速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点" (to slow down when fast, run even faster first) — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- 知乎 explanation: ordinary motor speed PID is negative feedback, "但是在以直立为主的平衡车系统中，负反馈的速度控制将会导致小车加速倒下". Series (串级) form: "速度环的输出作为直立环的输入… 把速度控制系统输出看做是一个角度，原本要保持角度为零就变成保持一个小的角度a1"; merging the two equations transforms into "一个单独的负反馈的直立环 + 一个单独的正反馈的速度环，也就是大家常说的平衡车的速度环是正反馈的由来" — [知乎 平衡小车从原理到实践](https://zhuanlan.zhihu.com/p/206522126)
- Why a speed loop is needed at all: with only the balance loop, a small disturbance gives a steady acceleration with "小车平移速度没有限制", until PWM saturates and there is no restoring force, so the car falls — [知乎 p/206522126](https://zhuanlan.zhihu.com/p/206522126). Similarly: "单单靠直立环控制小车，小车能短暂直立，但会出现往前走或往后走，然后倒下，那么速度环就是用来抑制此现象" and "单纯的直立环能使小车站稳5s就说明调的很好了" — [CSDN 平衡小车PID，就该这么调](https://blog.csdn.net/best_xiaolong/article/details/105153978)
- "首先明确速度环控制的原理是正反馈，就是小车在直立环的时候，小车往一个方向加速，速度环的作用就是用一个同方向更快的速度使小车平衡" — [CSDN 平衡车之速度环分析及调试](https://blog.csdn.net/zhaoyuaiweide/article/details/54572569)
- Kp multiplies velocity, Ki multiplies displacement: "kp*速度… ki*位移: 速度的积分就是位移"; the encoder sum is not divided by 2 or by dt because only a relative quantity is needed — [CSDN 54572569](https://blog.csdn.net/zhaoyuaiweide/article/details/54572569)
- Low-pass rationale: "平衡为主，其他为辅，为了让直立环处于主导地位，对速度环进行适当的低通滤波" — [知乎 p/206522126](https://zhuanlan.zhihu.com/p/206522126); "这样是为了减缓速度差值，对直立的干扰" — [CSDN 54572569](https://blog.csdn.net/zhaoyuaiweide/article/details/54572569)
- **机械中值 definition and measurement:** hold the car with motors off; the angle at which it balances briefly by itself — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978). Procedure: rotate the car about the motor axis forward and backward until it falls to the other side; take the middle of the two tipping angles, e.g. 2° and −3° → −1° (`float Med_Angle=-1.0;`) — [CSDN 串级PID调参 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386). Code uses it as `Bias=Angle-Zhongzhi` (STM32 `Zhongzhi=1`, jfboy Arduino `ZHONGZHI -8`) — [main.c](https://github.com/zycczy/minibalance/blob/master/USER/main.c), [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino). Yahboom ships `#define angle0 0.00 // Mechanical balance angle` — [Yahboom mirror](https://github.com/GaloisInc/lean4-balance-car/blob/main/ArduinoBalanceCar/ArduinoBalanceCar.ino)
- **Adaptive zero ("斜坡悬停") in the original:** `if(Flag_Hover==1) Zhongzhi=-Encoder/10-Encoder_Integral/300;` — i.e. the speed-loop state is fed directly into the balance setpoint for hovering on slopes — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)

### Inferences
- Derivation (my algebra, consistent with the 知乎 description): inner `u = Kp·(θ − θ_ref) + Kd·ω`, outer `θ_ref = −(Kpv·v + Kiv·x)/Kp`-type PI on wheel speed/position ⇒ `u = Kp·θ + Kd·ω + Kpv·v + Kiv·x`. The v/x terms carry the **same sign as the θ term** (forward tilt → forward drive; forward motion → more forward drive), hence "positive feedback". Physically, extra forward drive tilts the body backwards, the balance loop then drives backwards, and the car decelerates: net negative feedback on velocity through the pendulum.
- The integral `Kiv·x` is exactly equivalent to a slowly adapting 中值: at steady state the car stands at the tilt where `Kp·(θ−中值) + Kiv·x = 0`, so a 中值 error of Δ just shifts the parked wheel position by `Kp·Δ/Kiv` counts (e.g. STM32: 300·1°/0.4 = 750 counts). The car will drift a little after power-on and then hold. So 中值 need only be roughly right; a large error costs integral headroom (±10000 limit) and causes a start-up creep. `Flag_Hover` makes this explicit.
- For a very low CoM robot (~18 mm above axle), gravity torque per degree of 中值 error is small, so 中值 error matters even less; however the pendulum is fast (point-mass estimate √(g/l) ≈ √(9.81/0.018) ≈ 23 rad/s, real value lower because of distributed inertia), which favours the fastest balance loop rate and a heavily filtered, slow speed loop.

### Gaps
- The 知乎 cascade equations are images; the exact expanded formula was not captured as text.

---

## Q3. How forward/backward driving and turning commands are injected

### Takeaway
Driving is done by **subtracting a constant `Movement` from the position integral every speed-loop tick** (a moving position setpoint = ramp), not by changing the speed error. The integral clamp (±10000) then caps top speed. Turning adds a ramped differential term `±Turn_Target·Kp` that is split ± between wheels, and the gyro-Z damping is switched on only while driving straight.

### Cited Findings
- STM32 original: `Target_Velocity=110;` (55 in obstacle mode); `if(1==Flag_Qian) Movement=-Target_Velocity/Flag_sudu; else if(1==Flag_Hou) Movement=Target_Velocity/Flag_sudu; else Movement=0;` then `Encoder_Integral=Encoder_Integral-Movement;` — comment "修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了" — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- Arduino port (40 ms speed loop): `Movement = -300` forward, `+300` back; when released, the integral is bled towards zero quickly to stop: `if (Encoder_Integral > 300) Encoder_Integral -= 200; if (Encoder_Integral < -300) Encoder_Integral += 200;` ("这里是停止的时候反转，让小车尽快停下来"); comment on the clamp: "`//===积分限,控制最高速度`" — [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino)
- Yahboom: `car.positions += front; car.positions += back;` with `'w': front = 700` and `'s': back = -700` (per 50 ms speed tick), `constrain(car.positions, -3550, 3550)` — [Yahboom mirror](https://github.com/GaloisInc/lean4-balance-car/blob/main/ArduinoBalanceCar/ArduinoBalanceCar.ino)
- Why integral injection rather than speed target: two choices exist, `Encoder_Least=(L+R)-0` (speed target) or `Encoder_Integral -= Movement`; "我们选择后者是因为直接给速度偏差，对直立造成的干扰太大了，实际实验的时候也很不稳，而积分出来的位移给他位移偏差变化缓慢，对直立影响不大"; "movement决定速度的大小。积分的限幅决定速度的最大上限"; "控制速度的本质就是改变倾角" — [CSDN 54572569](https://blog.csdn.net/zhaoyuaiweide/article/details/54572569)
- Another trick mentioned for fast starts/race cornering: give an offset to the angle loop — [CSDN 54572569](https://blog.csdn.net/zhaoyuaiweide/article/details/54572569)
- Turning (STM32): ramp `Turn_Target -= / += Turn_Convert` while key held, `Turn_Convert=50/|encoder sum at turn start|` clamped 0.6–3 ("根据旋转前的速度调整速度的起始速度，增加小车的适应性"), clamp `±Turn_Amplitude(100)`, reset to 0 on release; `Kd=0.5` only when `Flag_Qian||Flag_Hou`, else 0 — [control.c turn()](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- Yahboom turning: separate "turn while moving" (turnmax ±3) and "spin in place" (turnmax ±10) modes, `rotationratio = 5 / turnspeed` clamped 0.5–5, `turnout_put = -car.turnout * kp_turn - Gyro_z * kd_turn` — [Yahboom mirror](https://github.com/GaloisInc/lean4-balance-car/blob/main/ArduinoBalanceCar/ArduinoBalanceCar.ino)
- Remote logic in STM32 is mutually exclusive: forward/back keys are read only when not turning and vice versa (`Get_MC6`) — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- A PWM trim constant compensates motor mismatch while driving: `#define DIFFERENCE 2 … if(Flag_Qian==1) Motor2-=DIFFERENCE;` — [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino)

### Inferences
- Steady cruise speed with `Movement` injection: integral stops growing when `Encoder(filtered sum per tick) = Movement`, so **Movement = desired encoder-sum per speed tick**. STM32: 110 counts/10 ms (sum of both wheels, 4x). jfboy: 300 counts/40 ms (sum, 2x decoding).
- For WASD over HC-05 this maps cleanly: W/S set Movement = ±M, A/D ramp Turn_Target, key release sets Movement=0 (optionally bleed integral like jfboy) and Turn_Target=0. Keys must auto-release on a timeout if BT drops (none of the reference codes do this; Yahboom relies on explicit stop char).

---

## Q4. Tuning procedure, typical ranges, and scaling to different loop rate / encoder resolution

### Takeaway
Canonical order: 机械中值 → 直立 Kp polarity & magnitude (until large low-frequency oscillation) → Kd polarity & magnitude (until high-frequency buzz) → multiply both by 0.6 → speed Kp polarity (with balance loop disabled: spinning one wheel must make both wheels accelerate the same way) → speed Kp magnitude with Ki = Kp/200 → turn loop polarity & magnitude.

### Cited Findings
- Steps list: "确立机械中值 / 直立环（内环）——Kp极性、Kp大小、Kd极性、Kd大小 / 速度环（外环）——Kp&Ki极性、Kp&Ki大小 / 转向环——系数极性、系数大小" — [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386)
- **Balance Kp range:** from PWM max and tolerable tilt: PWM 7200, ≤10° swing → Kp in 0–700 — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978); "如果kp=720，角度偏差10度，电机就到达满转了… kp的范围（0 - 720）" — [知乎 p/206522126](https://zhuanlan.zhihu.com/p/206522126)
- **Kp polarity:** correct = wheels drive toward the side the car falls; wrong = car is driven down faster — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978), [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386)
- **Kp magnitude:** increase until "大幅度的低频抖动" — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978); one builder: range 150–350, low-freq oscillation at 320 (STM32 scale) — [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386)
- **Kd polarity:** set Kp=0; lift the car and rotate it about the axle: correct if the wheels turn the same way as the body rotation ("车轮同向转动，有跟随") — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978), [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386)
- **Kd magnitude:** start at 0.1 (raw gyro counts are 4-digit), increase until high-frequency violent shaking — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978); range 0.1–0.9, high-freq at 0.8, switch off motors quickly to protect driver — [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386)
- **×0.6 rule:** "直立环调试完毕后，对所有确立的参数乘以0.6作为最终参数… 根据工程经验平衡小车的理想参数为最大参数乘以0.6"; after ×0.6 the shaking disappears but standing gets worse until the speed loop is added; final `Vertical_Kp=200, Vertical_Kd=0.5` — [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386); same rule in [知乎 p/206522126](https://zhuanlan.zhihu.com/p/206522126) and [华为云 blog](https://bbs.huaweicloud.com/blogs/333346)
- **Speed Ki = Kp/200:** "这里可以确定为200，别问为什么" — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978); STM32 defaults 80/0.4 — [main.c](https://github.com/zycczy/minibalance/blob/master/USER/main.c); jfboy comment "参考 kp = 2, ki = kp / 200" — [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino)
- **Speed Kp range estimate:** full-speed encoder sum per 10 ms ≈160 (STM32, quadrature); assume 50% speed error → full PWM: 160/2=80, 7200/80=90 → Kp_max≈90 — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978). Another: full-speed sum per 5 ms ≈100 → 7200/50 = 144 — [知乎 p/206522126](https://zhuanlan.zhihu.com/p/206522126)
- **Speed polarity test:** disable the balance loop (and turn loop); lift the car, spin one wheel by hand: negative feedback (other wheel counter-rotates to cancel) = wrong; both wheels accelerate the same way up to max speed = correct positive feedback — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978), [CSDN 54572569](https://blog.csdn.net/zhaoyuaiweide/article/details/54572569) (uses kp=±50), [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386)
- **Speed magnitude symptoms:** too small → car stands 1–2 s then accelerates one way / slowly drifts back and forth; too large → when pushed it cannot return, oscillates with large body tilt, or high-frequency shake — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978), [CSDN 54572569](https://blog.csdn.net/zhaoyuaiweide/article/details/54572569); target: "小车保持平衡的同时，速度接近于0，且回位效果好"; that builder's final `Velocity_Kp=0.30, Velocity_Ki=0.0015` (range 0.1–0.9 in his units) — [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386). Method: 试凑法+二分法 from small to large — [知乎 p/206522126](https://zhuanlan.zhihu.com/p/206522126)
- **Turn loop polarity:** lift the car and rotate it about Z: correct (negative feedback) if the wheels resist, i.e. try to turn opposite to the body rotation; increase Kp until straight-line driving is good without shaking; example `Turn_Kp=-0.6` — [CSDN 113786386](https://blog.csdn.net/weixin_44270218/article/details/113786386)
- The jfboy Arduino file has the staged combinations commented in the code (balance only → balance−velocity → all three), which is the practical way to do the polarity steps — [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino)
- **Reference scales:** STM32: PWM ±8400 (clamp 8200), Balance 300/1 (raw gyro), Velocity 80/0.4 per 10 ms 4x-decoded sum, Turn 52 — [main.c](https://github.com/zycczy/minibalance/blob/master/USER/main.c), [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c). Arduino: PWM ±255 (clamp 250), Balance 15/0.5, Velocity 2–2.4/0.01–0.012 per 40 ms 2x-decoded sum, Movement 300, Turn 2/0.001 — [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino). Yahboom: Balance 38/0.58 (°/s), speed 3.1/0.05 per 50 ms single-channel `CHANGE` counts, front 700 — [Yahboom mirror](https://github.com/GaloisInc/lean4-balance-car/blob/main/ArduinoBalanceCar/ArduinoBalanceCar.ino)

### Inferences (conversion to our hardware — my arithmetic, not from sources)
- Our encoder: 495 counts per wheel rev (11 CPR × 45, one edge of A); wheel circumference π·63 mm = 197.9 mm → **≈2501 counts/m per wheel**. Two-wheel sum at 1 m/s: 25 counts per 5 ms, 50 per 10 ms, 200 per 40 ms.
- Scaling rules to keep the same physical behaviour when PWM range P, counts per metre C (per wheel) and speed-tick period T change:
  - Balance Kp scales with P only (angle in degrees): Kp_new = Kp_ref·P_new/P_ref (300 at 8400 → ≈9 at 255; the Arduino ports use 15–38, i.e. motor/wheel/voltage differences dominate — treat as order-of-magnitude only). Kd scales with P and with the gyro unit (raw LSB vs °/s: ×16.4 or ×131 depending on range).
  - Speed Kp multiplies "counts per tick" → Kp_new = Kp_ref·(P_new/P_ref)·(C_ref·T_ref)/(C_new·T_new).
  - Speed Ki multiplies accumulated position counts (independent of T) → Ki_new = Ki_ref·(P_new/P_ref)·(C_ref/C_new). But because the integral is **updated once per tick**, the Ki/Kp = 1/200 ratio at a different tick rate changes the integral's effective time constant; with a 40 ms tick vs 10 ms, the same ratio gives an integral 4× slower in real time (jfboy keeps 1/200 at 40 ms anyway).
  - Movement (counts per tick) = desired speed × C_sum × T; integral clamp (counts) caps speed/lean: scale clamp by C_ref/C_new.
  - Low-pass 0.8/0.2 per tick has a time constant ≈ T/0.2 = 5 ticks (50 ms at 10 ms, 200 ms at 40 ms); 0.7/0.3 ≈ 3.3 ticks. Keep the time constant, not the coefficient, if T changes.
- Our single-edge counting has **no direction information** unless B is read in the ISR. Yahboom infers direction from PWM sign (`if ((car.pwm1 < 0) && (car.pwm2 < 0)) { rpluse = -rpluse; ...}`), which is wrong near zero crossings — exactly where a balancing robot lives. jfboy reads B inside a `CHANGE` ISR on A ("具有二倍频功能") — reading B in the INT0/INT1 ISR is strongly preferable for us.
- With 495 counts/rev and a 10 ms tick, low speeds give 0–2 counts per wheel per tick; quantisation noise on the speed P term will be large, which argues for a 20–40 ms speed tick (as the Arduino ports do) or heavier low-pass.

### Gaps
- No source gives JGA25-370 45:1 no-load speed at 7.4 V; full-speed encoder sum for the Kp_max estimate must be measured.
- The official 平衡小车之家 manual's own tuning text was not retrieved (secondary blogs reproduce it consistently).

---

## Q5. Loop timing, attitude filters, speed low-pass

### Takeaway
5 ms IMU sampling; control at 10 ms (STM32, synchronised to the MPU6050 INT pin) or 5 ms balance with 20 ms turn / 40–50 ms speed sub-loops (Arduino, MsTimer2). Angle from DMP, Kalman or first-order complementary filter (K1 = 0.02–0.05). Speed low-pass 0.8/0.2 (STM32) or 0.7/0.3 (Arduino).

### Cited Findings
- STM32: "5ms定时中断由MPU6050的INT引脚触发 严格保证采样和数据处理的时间同步"; "5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果"; control every 10 ms "为了保证M法测速的时间基准，首先读取编码器数据" — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- STM32 angle options: `Way_Angle` 1 DMP (`Angle_Balance=-Roll; Gyro_Balance=-gyro[0]; Gyro_Turn=gyro[2]`), 2 Kalman (default), 3 complementary; accel angle `atan2(Accel_Y,Accel_Z)*180/PI`; "三种算法经过我们的调校，都非常理想" — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c), [main.c](https://github.com/zycczy/minibalance/blob/master/USER/main.c)
- Filter constants: `float K1 =0.02f;` `Q_angle=0.001f; Q_gyro=0.003f; R_angle=0.5f; dt=0.005f;` and complementary `angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * 0.005f);` — [filter.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/filter/filter.c)
- jfboy Arduino: `MsTimer2::set(5, control); //使用Timer2设置5ms定时器中断`; `sei();//全局中断开启` at ISR start; balance every 5 ms, `if (++Velocity_Count >= 8)` speed every 40 ms, `if (++Turn_Count >= 4)` turn every 20 ms; Kalman with `K1 = 0.05; Q_angle = 0.001, Q_gyro = 0.005; dt = 0.005` — [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino)
- Yahboom: `MsTimer2::set(5, inter)`, turn every 10 ms (`turncount > 1`), speed every 50 ms (`speedcc >= 10`), Kalman `qAngle 0.001, qGyro 0.005, rAngle 0.5`, speed low-pass `speeds_filterold *= 0.7; speeds_filter = speeds_filterold + speeds * 0.3`; comment "in the timed interrupt, the code executed cannot exceed 5ms, otherwise it will destroy the overall interrupt" — [Yahboom mirror](https://github.com/GaloisInc/lean4-balance-car/blob/main/ArduinoBalanceCar/ArduinoBalanceCar.ino)
- Speed low-pass 0.8/0.2 STM32 — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c); 0.7/0.3 variants — [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino), [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978)

### Inferences
- MsTimer2 uses Timer2, which on the ATmega328P also drives PWM on D3/D11; TB6612 PWM pins must then be on Timer0/Timer1 pins (D5/D6/D9/D10). Running I2C inside a timer ISR requires `sei()` as the ports do; an alternative is a flag set by the timer (or MPU INT) and the control executed in `loop()`.
- On a Nano, reading only 6 raw registers (getMotion6) plus a complementary filter fits easily in 5 ms; DMP is optional.

---

## Q6. Fall detection, stop, integral reset, pick-up / put-down detection

### Takeaway
Motors are cut at |tilt| > ~40° (or low battery), the speed integral is zeroed whenever motors are off, and the STM32 original adds a 3-stage pick-up detector (still → big Z-accel near upright → wheels spin up by positive feedback) and a put-down detector (upright, wheels still, then wheels hand-turned slightly).

### Cited Findings
- `Turn_Off`: `if(angle<(-40+Zhongzhi)||angle>(40+Zhongzhi)||1==Flag_Stop||voltage<1110)` → all TB6612 IN pins low (coast), returns 1 — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- Integral reset: `if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0; //===电机关闭后清除积分` — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c); variant `if(pitch<-40||pitch>40) Encoder_Integral=0;` — [CSDN 105153978](https://blog.csdn.net/best_xiaolong/article/details/105153978)
- `Pick_Up` (10 ms ticks): stage 1 `myabs(encoder_left)+myabs(encoder_right)<30` for >10 ticks (近似静止); stage 2 within 2000 ms `Acceleration>26000 && |Angle−Zhongzhi|<20` ("小车是在0度附近被拿起"); stage 3 within 1000 ms `myabs(encoder_left+encoder_right)>160` ("小车的轮胎因为正反馈达到最大的转速") → returns 1, sets `Flag_Stop=1` — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- `Put_Down`: only when `Flag_Stop` set; stage 1 `|Angle−Zhongzhi|<10 && encoder_left==0 && encoder_right==0`; stage 2 within 500 ms both encoders in (−60, −3) ("小车的轮胎在未上电的时候被人为转动") → `Flag_Stop=0` — [control.c](https://github.com/zycczy/minibalance/blob/master/MiniBalance/CONTROL/control.c)
- Yahboom pick-up: when no command active and the accel/gyro angle `angle6` exceeds ±10° and accumulated pulses `stopl+stopr > 1500 || < -3500`, zero PWM and set `flag1=1`, which zeros `car.positions` (speed integral); plus hard cut at `angle > 30 || angle < -30` — [Yahboom mirror](https://github.com/GaloisInc/lean4-balance-car/blob/main/ArduinoBalanceCar/ArduinoBalanceCar.ino)
- jfboy Arduino: only `if(Angle > 40 || Angle < -50) Motor1=0,Motor2=0;`; the integral reset line is commented out — [jfboy ino](https://github.com/jfboy/BalanceCar/blob/master/balance_car_2.0.ino)

### Inferences
- Pick-up thresholds are in 4x counts per 10 ms at an 8400 PWM scale (160 ≈ max speed); for our 495-count single-edge encoders these must be rescaled (≈ ×495/1560 if their wheel-rev count is 1560, unverified), or better expressed as a fraction of the measured max speed.
- Integral reset on fall/stop is essential: a wound-up position integral will make the robot lurch on restart. Also reset the speed low-pass state and turn target.

### Gaps
- No source found that documents re-arming/soft-start (ramping gains) after put-down; the originals just resume.
