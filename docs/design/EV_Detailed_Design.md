# EV — Detailed Design (Pico 2 + BNO085, servo-steered pose tracking)

*The buildable spec for the redesign: track geometry, pin map, wiring, power, firmware state machine, and control-loop pseudocode. This is the design the firmware in `src/` implements.*

Companion to `EV_Active_Tracking_Proposal.md` (architecture rationale), `EV_Design_Review.md`, `EV_Rules_Scoring_Model.md`, `EV_How_It_Works.md`.

---

## 0. Track geometry & the path we're following (from the official rules)

Coordinates: **x** = forward along the centerline (Start → Target), **y** = left of centerline (positive).

- **Start** at (0, 0). **Target Point** at (**D**, 0), straight ahead on the centerline; D = announced target distance, 7.00–10.00 m.
- **Bonus line** at x = D/2, perpendicular, to the **left**. Outer can inside edge at **y = 100 cm** (placed by the ES). Inner can placed by you, its edge 0–100 cm inboard of the outer can. Your vehicle must pass **between** the cans.
- **Can Bonus = −0.5 × (110 − gap_cm)** — lower total score wins, so a *narrower* gap earns more (bounded by your car's width).

**Planned path = a constant-curvature circular arc** through (0,0) → apex (D/2, **h**) → (D, 0), where **h** (the sideways bulge / sagitta) is set so the car's center threads the can gate (~90–95 cm for a tight, high-bonus gate).

```
Radius:      R   = h/2 + D^2 / (8h)
Arc angle:   PHI = 2 * asin( D / (2R) )
Arc length:  L   = R * PHI          // distance the car actually travels; this is the stop target
Curvature:   k   = 1 / R            // constant (left turn)
Arc center:  C   = (D/2,  h - R)    // note h - R < 0, i.e. to the right of centerline
```

The car follows curvature `k` and stops when the free-roller encoder reads arc length `L`. Because it's a single circular arc, the *nominal* steering is a constant angle; the feedback loop's whole job is to hold the car on that arc despite misalignment, slip, and floor imperfections — the thing the fixed-angle design can't do.

*(Implemented in `src/path.cpp` — `pathPlan(D, h)` returns R, L, k, C; unit-tested in `host_check/`.)*

---

## 1. System block diagram

```
              +----------------------- 8x AA NiMH pack (~9.6-12 V) ----------------------+
              |                          |                              |
              v                          v                              v
        [BTS7960 driver]           [5 V regulator] --> servo,LCD    [Barrel plug -> Pico VSYS]
              |  ^                        |
        550 motor  |PWM/EN                | 5 V
         (rear)    |                      v
                   |                 [Steering servo] --(direct couple)--> front pivot
                   |
   =========================  Raspberry Pi Pico 2 (RP2350, 3.3 V)  =========================
     I2C0 (SDA/SCL) ---- BNO085 (0x4A/0x4B)  +  16x2 LCD (0x27)   [shared bus]
     PIO SM0 <---------- Front free-roller encoder A/B  (level-shifted if 5 V)
     PWM ---------------> BTS7960 RPWM/LPWM,  R_EN/L_EN
     PWM ---------------> Steering servo signal
     GPIO <-------------- Rotary dial CLK/DT/btn, Start btn, Set btn
   ========================================================================================
```

---

## 2. Pin map (Raspberry Pi Pico 2 / RP2350)

All GPIO are 3.3 V and **not** 5 V-tolerant. PWM: any GPIO. PIO: any GPIO. *(Matches `src/config.h`.)*

| Function | Signal | Pico GPIO | Notes |
|---|---|---|---|
| **I2C0** | SDA | GP4 | shared: BNO085 + LCD |
| | SCL | GP5 | 3.3 V bus; 4.7 kΩ pull-ups |
| **BNO085** | INT | GP6 | optional (data-ready) |
| | RST | GP7 | optional (clean reset) |
| **Front encoder** (distance) | A | GP10 | PIO quadrature; **level-shift if 5 V** |
| | B | GP11 | PIO quadrature |
| *(opt) drive-motor encoder* | A / B | GP12 / GP13 | for the speed loop only |
| **Motor driver BTS7960** | RPWM | GP16 | PWM (forward) |
| | LPWM | GP17 | PWM (reverse / brake) |
| | R_EN + L_EN | GP18 | tie both enables to one GPIO |
| | R_IS / L_IS | GP26 / GP27 (ADC) | optional current sense |
| **Steering servo** | SIG | GP15 | 50 Hz, 1000–2000 µs; powered from 5 V |
| **Rotary dial** | CLK / DT | GP20 / GP21 | value entry |
| | button | GP22 | cycles increment |
| **Buttons** | START | GP14 | pressed by the pencil trigger |
| | SET | GP19 | confirm dist/time |

---

## 3. Power & wiring notes

- **Motor** runs straight off the pack through the BTS7960. Keep motor + logic grounds **common** but route motor current on its own thick leads (12 AWG) back to the pack, not through the logic ground.
- **Servo** wants a stable **5 V** and can pull >1 A on a stall spike — feed it from a regulator (a 5 V, ≥2 A buck), **not** a Pico pin. Common-ground with the Pico.
- **Pico** powers from the pack via VSYS — VSYS accepts ~1.8–5.5 V; feed the 5 V rail into VSYS, not the raw pack.
- **BNO085** at 3.3 V from the Pico's 3V3(OUT); on the 3.3 V I2C bus directly.
- **Encoder**: if 5 V, power at 3.3 V (if supported) or put a level shifter on A and B before the Pico.
- **Decoupling**: 100 µF+ across the motor driver supply and 470 µF near the servo rail; 0.1 µF at each IC.

*(Full wiring detail + a diagram: `docs/WIRING_GUIDE.md` and `docs/wiring_diagram.html`.)*

---

## 4. Firmware architecture

Modules (implemented in `src/`):

- `imu` — BNO085 over I2C; **Game Rotation Vector**; `imuYawDeg()` (zeroed at arm in `pose`).
- `odo` — read the front-encoder PIO counter; `distance_m()`.
- `pose` — integrate `(x, y, θ)` from `ds` and heading; gated so it owns I2C only during a run.
- `path` — from `D` and `h`: `R, L, k, C`; `pathCrosstrack(pose)` and `pathArcRemaining(distance)`.
- `steer` — servo I/O + Stanley-style control → servo µs; `steerSetAngle(delta)`.
- `speed` — target-time → cruise PWM (calibrated table); trapezoidal profile.
- `drive` — BTS7960 helpers: `driveForward(pwm)`, `driveBrake()`, `driveCoast()`.
- `ui` — LCD + dial + buttons; entry of `D` and `T`; post-run readout.
- `sm` — the state machine below.
- `lcd` — PCF8574 I2C LCD driver. Plus harvested `imu`/`pid`/`motor_controller`/`utils`/`ram_*` and the PIO program.

Two-core split: **core 1 = pose estimator** at the control tick; **core 0 = state machine**.
Control tick: **500 Hz** (2 ms). BNO085 polled at ~100 Hz, latest heading held between reads.

---

## 5. State machine

```
BOOT  -> init IMU, PIO encoder, servo(center), LCD; motor coast -> SETUP
SETUP -> dial + SET to enter D (m), then T (s); optional gate bulge h;
         compute R,L,k,C and cruise plan; center steering -> ARMED
ARMED -> zero pose=(0,0,0), encoder=0, IMU heading ref = current yaw;
         steering pre-aimed to arc tangent; wait for START (pencil) -> LAUNCH
LAUNCH-> forward(pwm_launch) ~150 ms; start timer -> CRUISE
CRUISE-> each tick: update pose; delta=steering_control(pose); pwm=speed_control(...);
         until arc_remaining <= creep_zone -> APPROACH
APPROACH (the #1 accuracy win) -> creep at low pwm, keep steering on arc,
         stall-guard bumps pwm if not progressing, until distance >= L -> BRAKE
BRAKE -> active reverse pulse, then coast -> DONE
DONE  -> settle; read final distance & time; LCD readout; -> SETUP (2nd run)
```

*(Implemented in `src/sm.cpp`; APPROACH stall-guard and CRUISE IMU-loss abort are in place.)*

---

## 6. Control-loop pseudocode

### 6.1 Pose estimation
```c
void update_pose() {
    long   c   = encoder_count();
    double ds  = (c - c_prev) * M_PER_COUNT;   // M_PER_COUNT = circumference / CPR
    c_prev     = c;
    theta      = imu_heading();                // BNO085 yaw, zeroed at ARM
    x += ds * cos(theta);  y += ds * sin(theta);
    distance += ds;                            // arc length traveled (free roller)
}
```

### 6.2 Lateral control — feedforward arc + cross-track feedback
```c
double steering_control() {
    double delta_ff = atan(WHEELBASE_L * k);          // k = 1/R (left +)
    double dxc = x - Cx, dyc = y - Cy;
    double e_ct  = sqrt(dxc*dxc + dyc*dyc) - R;        // >0 = outside the arc
    double tangent = atan2(dxc, -dyc);                // sign set on bench
    double e_head  = wrap(tangent - theta);
    double delta_fb = Kh * e_head + atan2(Kc * e_ct, v_est + EPS);
    return clamp(delta_ff + delta_fb, -DELTA_MAX, +DELTA_MAX);
}
void set_angle(double delta) {
    int us = SERVO_CENTER_US + (int)(delta * SERVO_US_PER_RAD);
    servo_write_us(clamp(us, SERVO_MIN_US, SERVO_MAX_US));
}
```
*(Signs of `e_ct`, `e_head`, `SERVO_US_PER_RAD` are finalized on the bench — push the car off the arc; the servo must steer back toward it.)*

### 6.3 Longitudinal control — hit the target time
```c
double speed_control() {
    double v_ref = trapezoid_v(distance, L, T);   // desired speed now
    double pwm   = pwm_for_speed(v_ref);          // calibrated table
    return clamp(pwm, 0, PWM_MAX);                // optional inner loop off the free roller
}
```
*Time is only 1 pt/sec — table + trapezoid is plenty. Don't let timing tuning disturb the stop.*

### 6.4 The stop
Triggered by **`distance >= L_target`** on the **free-roller** encoder, at low creep speed, with a stall guard — lands on the same arc length every run regardless of battery/friction. Biggest score gain (distance is 2 pt/cm).

---

## 7. Calibration procedure (bench, in order)

1. **M_PER_COUNT** — roll the car a marked **5.000 m** straight, read counts; `M_PER_COUNT = 5.000 / counts`. Repeat 5×; spread = best-case distance precision.
2. **WHEELBASE_L** — front-axle to rear-axle contact, in meters.
3. **Servo map** — `SERVO_CENTER_US` (wheels dead straight) and `SERVO_US_PER_RAD` (command ±known angle, measure actual).
4. **PWM↔speed table** — at PWM 30/40/…/100 %, measure steady speed; store for `pwm_for_speed()`.
5. **Steering gains** `Kh, Kc` — start small, increase until it holds the arc without weaving.
6. **Creep** `pwm_creep`, `creep_zone`, stall-guard, brake pulse — tune for a dead stop, no overshoot.
7. **Gate/bulge h** — from your can strategy; verify the car threads it before narrowing the gap.

*(Full bench walkthrough with expected serial output: `docs/BRINGUP_AND_TUNING.md`.)*

---

## 8. Test plan
- **Static**: IMU heading stable + zeroing; encoder counts clean; servo hits angles; motor fwd/brake.
- **Straight-line hold** (h→∞ / k→0): drive straight 5 m; measure heading drift + stopping spread.
- **Arc hold**: follow a fixed arc; measure lateral error at midpoint (should land in the gate).
- **Full run**: dial D and T, run, read Fin Dist/Time; log 10 runs, look at the *spread*.

---

## 9. What gets reused vs added

**Reused:** 550 motor, 2WD gearbox, rear drive, single BTS7960, belt-driven free-roller encoder, LCD + dial + buttons UI, chassis, `getArcLength` math.
**Added:** Raspberry Pi Pico 2, Adafruit BNO085, one metal-gear micro servo (on the existing pivot), a 5 V regulator, and (if the encoder is 5 V) a level shifter.
**Removed:** the fixed adjustment arm + digital caliper as the *steering setter* (the servo + IMU loop replaces them), and the open-loop calibrate-then-blind-pulse logic. *(See `docs/hardware/CAD_SUMMARY.md`.)*

---

*Design locked: servo-steered front-pivot retrofit · single 550 rear drive (reused) · Pico 2 (RP2350) · BNO085 Game Rotation Vector · free-rolling front encoder for distance (PIO) · constant-curvature arc path through the can gate · encoder-triggered closed-loop stop.*
