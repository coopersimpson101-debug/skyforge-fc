# Attitude estimation and control

Conventions: world NED, body FRD (x forward, y right, z down), quaternion
`q = {w, x, y, z}` rotates body -> world. Positive roll = right side down,
positive pitch = nose up, positive yaw = nose right. Accelerometers measure
*specific force*, which points up at rest, so at rest the body-frame reading
is `-R^T e3` (about `(0, 0, -1) g` when level).

## Gyro pipeline (gyro rate, 8 kHz)

```
raw -> - bias (still-detecting calibration, Welford mean/variance, 1 s)
    -> biquad notch (center, Q = f0*fc / (f0^2 - fc^2), off by default)
    -> biquad low-pass (RBJ cookbook, Butterworth Q, default 200 Hz)
```

Biquads are transposed direct form II. The tests check the magnitude
response analytically (`biquad_gain_at`) and by pushing sines through the
filter.

## Madgwick AHRS (`src/core/madgwick.c`)

Objective: make the predicted "down" vector in the body frame match the
measured one, `v = -a/|a|`:

```
f(q) = R(q)^T e3 - v = [ 2(xz - wy) - vx,  2(yz + wx) - vy,  1 - 2(x^2+y^2) - vz ]

J = [ -2y   2z  -2w   2x ]      (columns d/dw, d/dx, d/dy, d/dz)
    [  2x   2w   2z   2y ]
    [   0  -4x  -4y    0 ]

qdot = 1/2 q (x) [0, w]  -  beta * (J^T f) / |J^T f|
```

`beta` (rad/s) bounds how fast the accelerometer can pull the estimate;
the correction rate is at most about `2*beta`. A high `beta_boot` is used for
the first 2 s. **Gyro-only fallback**: the accelerometer is ignored when
`| |a| - 1 g | > 0.25 g`, when it reads zero, or (front-end `ahrs.c`) when
the body rate is above 57 deg/s. At those times the accelerometer measures
thrust and drag, not gravity.

## Error-state EKF (`src/core/ekf_att.c`)

Nominal state `q`, gyro bias `b`. Error state `dx = [dtheta, db]` (6), with
`q_true = q (x) exp(dtheta)` (body-frame, multiplicative).

Predict with gyro `w_m`:

```
w   = w_m - b
q  <- q (x) exp(w dt)
Phi = [ I - [w]x dt   -I dt ]       Q = diag(sg^2 dt I, sb^2 dt I)
      [ 0              I    ]
P  <- Phi P Phi^T + Q
```

Update with the accelerometer down vector `z = -a/|a|`:

```
h(q) = R^T e3 = g                    (predicted down in body)
h(q (x) exp(dtheta)) ~ g + [g]x dtheta   =>   H = [ [g]x  0 ]
S  = H P H^T + R,   d2 = y^T S^-1 y   (chi-square gate, 3 dof, default 16)
K  = P H^T S^-1,    dx = K y,   y = z - g
P <- (I - KH) P (I - KH)^T + K R K^T            (Joseph form)
q <- q (x) exp(dtheta),  b <- b + db             (reset dx = 0)
```

`R` is inflated when `|a|` deviates from 1 g. Yaw is unobservable without a
magnetometer. Attitude variances are capped by a congruence scaling
`P <- D P D`, which keeps `P` positive semi-definite. Clamping only the
diagonal does not, and the first version of this filter diverged because of
it (a unit test caught it). From a static pose only the part of the bias
perpendicular to gravity can be observed, and the test checks exactly that.

### Which one?

| | Madgwick | EKF |
|---|---|---|
| cost per update | ~60 flops | ~2,500 flops (6x6 matrix products) |
| gyro bias | none, relies on boot calibration | estimated online |
| tuning | one gain `beta` | noise densities `ekf_gyro_noise`, `ekf_accel_noise`, `ekf_bias_walk` |
| test result (synthetic, 3.7 deg/s uncalibrated bias) | 70 deg drift | 0.13 deg |

Both run the same front-end gating. Select with `set ahrs_type 0|1`.

**A limit of both filters (seen in the simulator):** in sustained flight,
an accelerometer on a multirotor measures thrust plus drag. A sideways
velocity after a manoeuvre biases the "gravity" direction by a degree or
so until the velocity decays. The sim models this, which is why the gust
recovery plot shows a slow residual of about 1 deg. Rotor-drag-aware
estimation is on the roadmap.

## Cascaded control

```
angle mode:  stick -> (roll_sp, pitch_sp) --+
                                            v
               q_des = q(roll_sp, pitch_sp, current yaw)
               q_e   = q^-1 (x) q_des            (body frame, shortest path)
               rate_sp(roll, pitch) = angle_p * rotvec(q_e)     [limited]
               rate_sp(yaw) = rate curve(yaw stick)
acro mode:   rate_sp = rate curve(stick)   (centre sensitivity, max rate, expo)

rate PID (per axis, deg/s):
   e  = sp - gyro
   P  = kp * 3.2e-5 * e * tpa
   I += ki * 2.44e-4 * e * dt   (conditional integration: frozen while the
                                 mixer is saturated unless it unwinds; |I| <= i_limit)
   D  = -kd * 5.29e-7 * PT1(d gyro / dt) * tpa        (on measurement: no setpoint kick)
   F  =  kf * 1.4e-7  * PT1(d sp / dt)                (feed-forward)
   tpa = 1 - tpa_rate * (thr - bp) / (1 - bp) above the breakpoint
```

The scale constants make the gains look like Betaflight's (P ~ 50, I ~ 50,
D ~ 45, F ~ 100). The response is **not** the same as Betaflight with the
same numbers.

## Mixer (quad-X, Betaflight motor order)

```
 4 FL (CW)   2 FR (CCW)       motor = thr + 0.5*(R*roll + P*pitch + Y*yaw)
       \     /                R = +1 left (3,4),  P = +1 front (2,4),
        [ ^ ]                 Y = +1 CCW props (2,3)  (props-in default;
       /     \                    yaw_reversed flips Y for props-out)
 3 RL (CCW)  1 RR (CW)
```

Desaturation: if the roll/pitch/yaw spread exceeds the motor range, the
whole rpy vector is scaled down, which keeps its direction. With airmode
the throttle is then shifted so every motor stays in [0, 1]. You keep full
attitude authority at zero and at full throttle, and the I-term is held at
zero until the throttle first passes 25 % after arming. Without airmode the
outputs are clipped.

The simulator computes torques from motor positions (`r x F`) and prop spin
direction. It does not reuse the mixer table, so the closed-loop tests also
check the mixer signs.

## Arming and failsafe

Arming requires a rising edge on the arm switch while nothing blocks it:
`GYRO_CAL, RX_LOSS, FAILSAFE, THROTTLE (> 5 %), ANGLE (> 25 deg), CLI,
BENCH, IMU`. A switch that is already on at boot, or held through a block,
has to be cycled. `status` in the CLI prints the active blocks.

Failsafe: no valid RC frame for `fs_timeout_ms` (300 ms) while armed. With
`fs_procedure = 0` (DROP) the craft disarms at once. With `fs_procedure = 1`
(LAND) the FC forces angle mode, centres the sticks and holds `fs_throttle`
for `fs_land_time_s`, then disarms. The landing is open loop: there is no
altitude sensor. Set `fs_throttle` a little below hover. If the link comes
back with the switch off, the pilot disarm wins. After a failsafe disarm,
re-arming needs the link back and the switch cycled.

## Loop scheduling

The IMU runs at an 8 kHz ODR and its data-ready interrupt clocks the
loop. `pid_loop_hz` (500..8000) sets `pid_denom = gyro_rate / pid_loop_hz`,
which gives 8, 4, 2 or 1 kHz. Background tasks (CRSF parsing, CLI,
telemetry, LED) run cooperatively between gyro samples. `loop_stats_t`
records min/avg/max period and overruns (period > 1.5x expected), and the
CLI `status` command shows them.
