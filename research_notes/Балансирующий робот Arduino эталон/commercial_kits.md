# Commercial Arduino self-balancing kits: firmware anatomy (Elegoo Tumbller, Keyestudio KS0193, Yahboom BST-ABC, Osoyoo)

Method note: the source code of all four kits was downloaded and read directly (git clones and the official zip), not taken from marketing pages. Every code quote below is verbatim from those files. Short names: **T** = Elegoo Tumbller, **K** = Keyestudio KS0193, **Y** = Yahboom BST-ABC, **O** = Osoyoo.

Source files used:
- T (official zip `Tumbller_Code_20191012.zip`): https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial. The files are `Tumbller/BalanceCar.h` (the whole control loop), `KalmanFilter.cpp/.h`, `Tumbller.ino` (state machine, motion setpoints), `Command.h` (Bluetooth parsing) and `Pins.h`.
  - A reformatted mirror with identical constants (checked by diff): https://github.com/unforgiven512/elegoo-tumbller-arduino
  - A modular PlatformIO refactor: https://github.com/atrzeciak/Tumbller
- K: https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit, folder `01.Source Code/`. The final sketches are `11_Bluetooth_control_1/11_Bluetooth_control_1.ino` and `12_Bluetooth_adjust_angle___PID/...ino`. The docs are https://docs.keyestudio.com/projects/KS0193/en/latest/KS0193.html and the official download is https://fs.keyestudio.com/KS0193.
- Y: https://github.com/YahboomTechnology/Arduino-Balance-Car, folder `3.Arduino balance car code_2019_2_14/bst_abc/`. The files are `bst_abc.ino`, `BalanceCar.cpp/.h` and `KalmanFilter.cpp/.h`. The repo also contains `balance car Bluetooth communication protocol.pdf/.xlsx`.
  - https://github.com/liadbiz/balance_car is a modified copy of Y with Q-learning code added. Do not use it as a reference.
- O: https://osoyoo.com/wp-content/uploads/2018/03/osoyoo_abc.zip, which contains `osoyoo_abc.ino`. The lesson page is https://osoyoo.com/2018/03/28/arduino-balance-car-lesson-3/. This is a rebadge of the Yahboom BST-ABC code: same structure, same `balancecar.speedpiout / turnspin / pwma` calls, same `$...#` protocol.

## 1. Controller architecture (angle, speed and turn loops; output summation; signs)

### Takeaway
All four kits use the same Chinese "BST" balance-car template:
- **Angle loop**: PD on the Kalman angle (in degrees) and the gyro rate (in deg/s), every 5 ms.
- **Speed loop**: PI on the *sum* of the two wheel encoder counts. It runs every 8 ticks (40 ms), or 10 ticks (50 ms) in O. The count is low-pass filtered (0.7/0.3) and the integral is clamped at ±3000 (±3550 in K).
- **Turn loop**: a setpoint plus gyro-Z damping.
- **Output**: `PWM = ±angle_out − speed_out ∓ turn_out`, clamped to ±255.

The integral of filtered speed is effectively a wheel-position term, so the speed loop is really a PD on wheel position.

### Cited Findings
- **T angle loop** (in the 5 ms ISR): `double balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);` The D term uses the raw gyro rate `Gyro_x = (gx - 128.1) / 131;` (a hard-coded offset, in deg/s). It does not use the Kalman bias-corrected rate. — [Elegoo BalanceCar.h / KalmanFilter.cpp](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- **T speed loop**, verbatim:
  ```
  speed_control_period_count++;
  if (speed_control_period_count >= 8) {
    speed_control_period_count = 0;
    double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0; encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
    rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
  }
  pwm_left  = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;
  pwm_left = constrain(pwm_left, -255, 255); pwm_right = constrain(pwm_right, -255, 255);
  ```
  So the effective output is `PWM = kp·θ + kd·ω + kp_speed·v_f + ki_speed·Σ(v_f − v_set) ∓ turn`. The speed terms enter with the *same* sign as the tilt term: positive feedback on wheel speed, which is the standard non-minimum-phase trick. — [Elegoo BalanceCar.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- **T encoder direction.** T reads only channel A (CHANGE interrupt: pin 2 via INT0, pin 4 via PinChangeInt). The direction comes from the sign of the last PWM output, not from quadrature: `encoder_left_pulse_num_speed += (pwm_left < 0) ? -encoder_count_left_a : encoder_count_left_a;`. — [Elegoo BalanceCar.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- **T turn loop**: `kp_turn = 2.5` is declared but **never used**. The turn output is only `setting_turn_speed + kd_turn*Gyro_z`, computed inside the 40 ms speed block. `Gyro_z = -gz / 131;` has no offset, and because `gz` is an int16 this is integer division. — [Elegoo BalanceCar.h / KalmanFilter.cpp](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- **K angle loop**: `PD_pwm = kp * (angle + angle0) + kd * angle_speed;`. Here `angle_speed = gyro_m - q_bias` is the Kalman bias-corrected rate, and `Gyro_x = -gx / 131;` (the sign is flipped relative to T). — [Keyestudio 11_Bluetooth_control_1.ino](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)
- **K speed loop**:
  ```
  float speeds = (pulseleft + pulseright) * 1.0;
  pulseright = pulseleft = 0;
  speeds_filterold *= 0.7;
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter; positions += front; positions += back;
  positions = constrain(positions, -3550,3550);
  PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter);
  ```
  K uses the sum of both wheels, not the average. — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)
- **K output**: `pwm2=-PD_pwm - PI_pwm + Turn_pwm;` (left) and `pwm1=-PD_pwm - PI_pwm - Turn_pwm;` (right). The turn term is `Turn_pwm = -turnout * kp_turn - Gyro_z * kd_turn;`. — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)
- **Y and O angle loop**: `balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;` (raw gyro rate with the 128.1 offset, as in T).
- **Y and O speed loop**: `speedpiout()` is the same as K (sum of both wheels, 0.7/0.3 filter, `positions += f; positions += b;`), with the clamp `constrain(positions, -3000,3000)`.
- **Y and O output**: `pwm1 = -angleoutput - speedoutput - rotationoutput; pwm2 = -angleoutput - speedoutput + rotationoutput;`.
- — [Yahboom BalanceCar.cpp / bst_abc.ino](https://github.com/YahboomTechnology/Arduino-Balance-Car); [Osoyoo osoyoo_abc.ino](https://osoyoo.com/wp-content/uploads/2018/03/osoyoo_abc.zip)
- **Turn setpoint ramp (K, Y, O).** While a turn flag is set, `turnout` is ramped by `rotationratio = 5 / turnspeed` (clamped to 0.5..5, where `turnspeed` = |pulse sum|) every turn tick, up to `turnmax=3` for turns and `10` for spins (spins are Y/O only). When the flag is released it snaps back to 0. — [Yahboom BalanceCar.cpp](https://github.com/YahboomTechnology/Arduino-Balance-Car); [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)

### Inferences
- The unit system is PWM counts (±255) per degree and per deg/s. Speed is in raw encoder counts per 40 ms window, and the "integral" is accumulated counts, i.e. wheel position.
- The overall minus sign on the whole sum in K/Y (versus plus in T) only reflects different motor wiring and IMU orientation. The *relative* signs are identical across all kits: the speed feedback reinforces the tilt command, and the turn term is differential.
- T's `ki_speed·Σ(v_f − v_set)` is a position-error loop. Its steady state requires the filtered speed to equal `setting_car_speed`.

### Gaps
- Y's `balance car Bluetooth communication protocol.pdf` was not read in full; the protocol description below is derived from the parser code.
- O's `BalanceCar.cpp` sits in `libraries.zip`, which was not downloaded. The integral clamp and output formula for O are assumed identical to Y because `osoyoo_abc.ino` calls the same API.

## 2. Loop timing

### Takeaway
Everything runs inside a MsTimer2 (Timer2) ISR at 5 ms. The ISR calls `sei()` first so encoder interrupts can nest, then does the I2C read, Kalman update, PD and motor writes.

| Loop | T | K | Y | O |
|---|---|---|---|---|
| Speed loop | every 8 ticks = 40 ms | every 8 ticks = 40 ms | every 8 ticks = 40 ms | every 10 ticks = 50 ms |
| Turn loop | every 40 ms (inside speed block) | every 5 ticks = 25 ms | every 5 ticks = 25 ms | every 5 ticks = 25 ms |

### Cited Findings
- T: `MsTimer2::set(5, balanceCar); MsTimer2::start();`, `float dt = 0.005;`, and the speed block runs when `speed_control_period_count >= 8`. — [Elegoo BalanceCar.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- K: `MsTimer2::set(5, DSzhongduan);`. The code has `if(cc>=8) //5*8=40，40ms entering PI count of speed once` and `if(turncc>4) //20ms entering PI count of steering once`. `turncc>4` actually fires every 5 ticks, which is 25 ms, so the comment is wrong. — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)
- Y: `MsTimer2::set(5, inter);` with `speedcc >= 8` (40 ms) and `turncount > 4`. — [Yahboom bst_abc.ino](https://github.com/YahboomTechnology/Arduino-Balance-Car)
- O: `if (speedcc >= 10)//50ms进入速度环控制` ("50 ms: enter speed loop") and `turncount > 4`. — [Osoyoo osoyoo_abc.ino](https://osoyoo.com/wp-content/uploads/2018/03/osoyoo_abc.zip)
- K and Y re-call `attachPinChangeInterrupt(...)` on every pass of `loop()`, which is sloppy but works. — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit); [Yahboom](https://github.com/YahboomTechnology/Arduino-Balance-Car)

### Inferences
- The ISR does a blocking `mpu.getMotion6()` over I2C (about 0.5 ms or less at 100/400 kHz), float Kalman math and `analogWrite`. On a 16 MHz AVR this fits inside the 5 ms budget.
- Because Timer2 is used, PWM on pins 3 and 11 is lost (the K code comment says so), which is why the motor PWM goes on pins 5/6 (T) or 9/10 (K, Y).

### Gaps
- None of the kits sets the MPU-6050 DLPF or the I2C clock in the control code. `mpu.initialize()` from the i2cdevlib library leaves ±250 dps and ±2 g as defaults (consistent with the /131 scaling), but I did not verify the DLPF register state from source.

## 3. Angle estimation

### Takeaway
All kits use the same scalar 2-state Kalman filter (angle plus gyro bias, the "Kalman_Filter" routine common in Chinese balance-car code). Parameters:
- `Q_angle = 0.001`, `Q_gyro = 0.005` (K uses 0.003), `R_angle = 0.5`, `C_0 = 1`, `dt = 0.005`.
- The accelerometer angle is `atan2(ay, az)` in degrees.
- A complementary filter (`Yiorderfilter`, `K1 = 0.05`) is also computed, but only for the pitch/roll axis that Y/O use for pick-up detection.

No kit uses the DMP.

### Cited Findings
- T: `float Angle = atan2(ay , az) * 57.3; Gyro_x = (gx - 128.1) / 131; Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0); Gyro_z = -gz / 131;`. The parameters are `float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;`. — [Elegoo KalmanFilter.cpp / BalanceCar.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- The Kalman body is identical in T, K and Y:
  ```
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = -P[1][1]; Pdot[2] = -P[1][1]; Pdot[3] = Q_gyro;
  P += Pdot*dt ...
  E = R_angle + C_0*PCt_0;
  K_0 = PCt_0/E; K_1 = PCt_1/E;
  ...
  angle += K_0*angle_err; q_bias += K_1*angle_err;
  angle_dot = gyro_m - q_bias;
  ```
  — [Yahboom KalmanFilter.cpp](https://github.com/YahboomTechnology/Arduino-Balance-Car); [Elegoo](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- K uses `Q_gyro = 0.003` and a negated accelerometer angle: `float Angle = -atan2(ay , az) * (180/ PI); Gyro_x = -gx / 131;`. — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)
- Y: `if (gz > 32768) gz -= 65536; Gyro_z = -gz / 131; accelz = az / 16.4;`. The complementary filter is `angle6 = K1*angle_m + (1-K1)*(angle6 + gyro_m*dt)` applied to `atan2(ax, az)` with `Gyro_y`. — [Yahboom KalmanFilter.cpp](https://github.com/YahboomTechnology/Arduino-Balance-Car)

### Inferences
- The gyro offset is a hard-coded constant (128.1 LSB on X) rather than being calibrated at boot. The Kalman bias state absorbs the remainder for the angle estimate. However, T, Y and O feed the *raw* `Gyro_x`, not `angle_dot`, into the D term, so the D term carries any offset error.

## 4. Driving forward/back and turning

### Takeaway
None of the kits tilts the balance-angle setpoint to drive. All of them drive through the speed loop's integral (position) term:
- **T** subtracts `setting_car_speed` from the integral every 40 ms (value 80 in Bluetooth mode, 40 default, 20 in follow mode). This makes the setpoint a target filtered speed in counts per 40 ms.
- **K, Y and O** add `front = 250` or `back = -250` to `positions` every speed tick.
- **Turning** is a differential PWM term (a setpoint plus `kd_turn·Gyro_z` damping), added to one wheel and subtracted from the other.

### Cited Findings
- T setpoints in `setMotionState()` (Tumbller.ino):
  - `case FORWARD: ... case BLUETOOTH: setting_car_speed = 80;` and `BACKWARD ... = -80`. The default mode uses ±40 and FOLLOW uses ±20.
  - `TURNLEFT ... BLUETOOTH: setting_turn_speed = 80;` and `TURNRIGHT ... = -80`. In other modes turning uses ±50, and non-Bluetooth modes also set `setting_car_speed = 0`.
  - `STANDBY: setting_car_speed = 0; setting_turn_speed = 0;`.
  - — [Elegoo Tumbller.ino](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- In T's BLUETOOTH mode, turning does not zero `setting_car_speed`, so forward plus turn combine. The turn PWM is simply `setting_turn_speed + kd_turn*Gyro_z` (80 PWM counts differential). — [Elegoo Tumbller.ino / BalanceCar.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- K: `case 'F': front=250; break; case 'B': back=-250; break; case 'L': left=1; ... case 'S': front=0,back=0,left=0,right=0;` and in the PI step `positions += front; positions += back;`. — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)
- Y: `case enRUN: ResetCarState();front = 250; break; case enBACK: ResetCarState();back = -250; break; case enLEFT: turnl = 1; ... case enTLEFT: spinl = 1;`. — [Yahboom bst_abc.ino](https://github.com/YahboomTechnology/Arduino-Balance-Car)

### Inferences
- **T in steady state.** The integral stops changing when `speed_filter == setting_car_speed`. For T, 80 counts per 40 ms of the averaged wheel is 2000 counts/s. At 660 counts per wheel revolution and about 67 mm diameter, that is about 3 rev/s, or roughly 0.6 m/s.
  - The P term `-kp_speed*speed_filter` then leaves a constant integral offset. Physically this is a small lean that the angle loop holds. The lean comes out of the loop structure; it is not commanded.
- **K, Y and O in steady state.** Adding a constant 250 per tick to `positions` makes the loop settle at `speeds_filter = −250` counts per window (sum of two wheels), in whatever sign convention applies.
  - Both approaches are the same thing: a *speed reference inside the integrator*.
  - With the ±3000/±3550 clamp, a large command can saturate the integrator. That is the only built-in limit on driving speed.
- The switch between standing still and driving is abrupt (a step in `setting_car_speed`). There is no ramp. The low-pass on the measured speed (0.3 new) and the integrator smooth the response.

## 5. Mechanical balance point, zero angle, integral limits, fall handling

### Takeaway
The zero angle is a fixed constant, 0 in every kit (`angle_zero = 0` in T; `angle0 = 0` or `1` in K; `angle0 = 0.00` in Y/O). The speed-loop integral absorbs any center-of-mass offset: an offset produces a steady wheel-position error that the integral trims out.
- K's sketch 12 lets the phone app overwrite `angle0` at runtime.
- The integrator is clamped at ±3000 (T, Y) or ±3550 (K) and is reset on fall or restart.

### Cited Findings
- T: `double angle_zero = 0; //x axle angle calibration` and `double angular_velocity_zero = 0;`. — [Elegoo BalanceCar.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- T fall logic: `balance_angle_min = -22; balance_angle_max = 22;` (±27 is commented out). Beyond that, `motion_mode = STOP; carStop();`. In STOP, `car_speed_integeral = 0; setting_car_speed = 0;`. — [Elegoo BalanceCar.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- T restart: the START state waits 2 s, then checks the angle is within ±22°, then sets `car_speed_integeral = 0; setting_car_speed = 0; motion_mode = STANDBY;`. `loop()` auto-issues `key_value = '5'` (START) 2 s after boot. — [Elegoo Tumbller.ino](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- K: sketch 11 has `float angle0 = 1;`. Sketch 12 changes it to `int angle0 = 0;` plus runtime tuning: any byte that is not a command letter is assigned to it (`TT = val; angle0=TT;`). The cut-off is `if(angle>80 || angle<-80) pwm1=pwm2=0;` (the comment says 45°). — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)
- Y: the cut-off is `if (angle > 30 || angle < -30) {pwm1 = 0; pwm2 = 0;}`.
  - Y also has pick-up detection. If `angle6` (the other axis) exceeds ±10° while there is no motion command and `stopl + stopr > 1500 || < -3500` (the wheels spun freely), then the PWM is set to 0 and `flag1=1`, which makes `speedpiout` set `positions=0`.
  - — [Yahboom BalanceCar.cpp](https://github.com/YahboomTechnology/Arduino-Balance-Car)

### Inferences
- None of the kits calibrates the balance angle at boot. They rely on the PI speed loop, and the ±3000 clamp bounds how large a center-of-mass offset the integral can absorb. For T the steady integral contribution can reach up to `0.26·3000 = 780` (more than full PWM), so the absorption range is large.

## 6. Default coefficients and target hardware

### Takeaway
T is the closest match to our robot: Nano, TB6612, 2S 7.4 V 18650, 1:30 motors, single-channel CHANGE counting (22 counts per motor revolution), wheels of about 67 mm. K, Y and O use 3S 18650 packs (about 11.1–12 V) with 12 V motors.

| Kit | kp (angle) | kd (angle) | kp_speed | ki_speed | kp_turn | kd_turn |
|---|---|---|---|---|---|---|
| T | 55 | 0.75 | 10 | 0.26 | 2.5 (unused) | 0.5 |
| K | 34 | 0.62 | 3.6 | 0.080 | 24 | 0.08 |
| Y | 38 | 0.58 | 3.8 | 0.11 | 28 | 0.29 |
| O | 40 | 0.6 | 5.20 | 0.25 | 23 | 0.3 |

Y's restore-defaults array is `{38, 0.0, 0.58, 4.0, 0.12, 0.0}`.

### Cited Findings
- T: `double kp_balance = 55, kd_balance = 0.75; double kp_speed = 10, ki_speed = 0.26; double kp_turn = 2.5, kd_turn = 0.5;`. — [Elegoo BalanceCar.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- T hardware:
  - Motors are 1:30 with a Hall encoder giving 22 pulses per motor revolution, so 660 per wheel revolution.
  - The battery is 2× ICR18650 2200 mAh (7.4 V).
  - The wheel is about 2.63 in (about 67 mm).
  - — [Tom Wilson, Medium pt.2/3](https://medium.com/@tomw3115/programming-the-elegoo-tumbller-self-balancing-robot-part-2-e9ab09c6bea2) (a third-party blog; the Medium page was 403 when fetched, so this is from search snippets)
- T pins (`Pins.h`): `AIN1 7, PWMA_LEFT 5, BIN1 12, PWMB_RIGHT 6, STBY_PIN 8, ENCODER_LEFT_A_PIN 2, ENCODER_RIGHT_A_PIN 4`. Only one direction pin per motor is driven, so AIN2/BIN2 are presumably hard-wired as inverters on the board. — [Elegoo Pins.h](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- K: `double kp = 34, ki = 0, kd = 0.62; double kp_speed = 3.6, ki_speed = 0.080; double kp_turn = 24, ki_turn = 0, kd_turn = 0.08;`. — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit)
- K hardware:
  - The motor is "DC12V, Reduction ratio: 1:30, No-load speed: 247rpm, Rated torque 1.4 kg·cm, Stall 5.5 kg·cm".
  - The wheel is "outer diameter 68mm".
  - The battery is a "18650 3-cell" holder.
  - — [KS0193.md](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit/blob/master/KS0193.md)
- Y: `double kp = 38, ki = 0.0, kd = 0.58; double kp_speed =3.8, ki_speed = 0.11; double kp_turn = 28, ki_turn = 0, kd_turn = 0.29; const double PID_Original[6] = {38, 0.0, 0.58,4.0, 0.12, 0.0};`. The liadbiz copy has kd = 0.56 and kp_speed 3.9 / ki 0.12. — [Yahboom bst_abc.ino](https://github.com/YahboomTechnology/Arduino-Balance-Car)
- O: `double kp = 40, ki = 0.0, kd = 0.6; double kp_speed =5.20, ki_speed = 0.25; double kp_turn = 23, ki_turn = 0, kd_turn = 0.3;`. The lesson describes kp_speed 5.2 / ki_speed 0.25 as a "rough demonstration". — [osoyoo_abc.ino](https://osoyoo.com/wp-content/uploads/2018/03/osoyoo_abc.zip); [Osoyoo lesson 3](https://osoyoo.com/2018/03/28/arduino-balance-car-lesson-3/)
- O hardware: "GM37 with Hall Speed Measuring Encoder" motors and a "Battery box for three 18650 batteries". — [Osoyoo tutorial](http://osoyoo.com/ja/2018/07/18/osoyoo-balancing-car/)

### Inferences (adapting to our hardware)
- **Encoder scale.** T counts 660 per wheel revolution over roughly π·67 mm, about 3.1 counts/mm. We count 495 per wheel revolution (11 CPR × 45, single edge) over π·63 mm, about 2.5 counts/mm, i.e. about 0.8× T.
  - If we count CHANGE on channel A like T, we get 990 counts per revolution, about 5.0 counts/mm, i.e. about 1.6× T.
  - Scale `kp_speed` and `ki_speed` (and `setting_car_speed`, and the ±3000 clamp) inversely to the counts-per-mm ratio to keep the same physical gains.
- **Motor gearing.** Our 45:1 against T's 30:1 means about 1.5× more wheel torque per PWM count and about 0.67× top speed at the same voltage.
  - So the PWM-per-degree gains (kp around 55, kd around 0.75) should transfer roughly, with somewhat less needed.
  - Speed headroom is smaller, so `setting_car_speed = 80` counts per 40 ms (about 0.6 m/s on T) may be near saturation for us.
- **Low center of mass.** Our center of mass (about 18 mm) makes a faster pole (about 19 rad/s), so a 5 ms loop (about 0.1 rad per sample at the pole) remains adequate. The D term, and the rate signal it uses, matter more than on T.
  - T uses the raw gyro with a hard-coded offset. A bias-corrected rate (K's `angle_speed`) is the safer choice.
- **Encoder direction.** T takes the encoder direction from the PWM sign. That is inaccurate near zero crossings, and our JGA25 has quadrature (A/B), so reading real direction is an improvement over all four kits.

### Gaps
- The exact motor model and gear ratio for the Yahboom BST-ABC (possibly a 37 mm 12 V motor, 1:30) was not confirmed from a primary source.
- The motor supply voltage for O and Y is not confirmed beyond "3×18650".

## 7. Bluetooth command protocol

### Takeaway
T and K use single ASCII characters at 9600 baud over UART0. Y and O use a framed ASCII packet `$a,b,c,d,e,f,...#` (which also allows setting PID from the app) at 9600.

### Cited Findings
- T `getBluetoothData()` accepts `'f','b','l','i','s','0'..'9','*','#'`, where `f` = forward, `b` = back, `l` = left, `i` = right and `s` = stop. `f/b/l/i` switch `function_mode = BLUETOOTH` and `s` sets IDLE. Other characters: `'1'` follow, `'2'` obstacle, `'0'` follow2, `'3'` LED effects, `'4'` stop/lie down, `'5'` start/stand up, `'6'..'9'` brightness. The link is `Serial.begin(9600)`. — [Elegoo Command.h / Tumbller.ino](https://github.com/elegooofficial/ELEGOO-TumbllerV1.1-Self-Balancing-Car-Tutorial)
- K sketch 11 uses `'F','B','L','R','S'` plus `'D'`, which returns the angle.
- K sketch 12 adds runtime tuning:
  - `'P'/'O'` kp ±0.5
  - `'I'/'U'` kd ±0.02
  - `'Y'/'T'` kp_speed ±0.05
  - `'G'/'H'` ki_speed ±0.01
  - `'J'/'K'` kp_turn ±0.4
  - `'N'/'M'` kd_turn ±0.01
  - `'Q'` returns the angle
  - any other byte becomes `angle0`
  - The kit uses an HC-06 at 9600 with PIN 1234. — [Keyestudio](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit); [KS0193.md](https://github.com/keyestudio/KS0193-Self-balancing-Car-Kit/blob/master/KS0193.md)
- Y and O: `serialEvent()` collects the bytes from `'$'` to `'#'`.
  - `inputString[1]`: `'1'` run, `'2'` back, `'3'` left, `'4'` right, `'0'` stop.
  - `inputString[3]`: `'1'` or `'2'` means spin left or right (only when the length is 21).
  - `[5]`: `'1'` query PID (reply `$0,0,0,0,0,0,AP..,AD..,VP..,VI..#`), `'2'` restore PID.
  - `[7]`: auto-report on or off.
  - `[9]=='1'` sets the angle PID from `AP..`/`AD..` fields; `[11]=='1'` sets the speed PID from `VP..`/`VI..` fields.
  - The telemetry format is `$LV..,RV..,AC..,GY..,CSB..,VT..#`.
  - — [Yahboom bst_abc.ino](https://github.com/YahboomTechnology/Arduino-Balance-Car)

### Inferences
- T's single-character protocol (`f/b/l/i/s`) maps most directly onto an HC-05 at 115200 with any "Bluetooth serial controller" style app. Only the baud rate needs to change.

### Gaps
- SunFounder and DFRobot Arduino balance kits were not researched (the tool-call budget was spent on reading actual source). By the pattern seen here they may reuse the same template, but that is unverified.
- The newer Elegoo Tumbller app firmware revisions (after 2019-10-12) were not checked for different constants.
