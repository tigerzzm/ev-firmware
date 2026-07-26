# EV — Active Pose-Tracking Redesign: Architecture Proposal

*Goal: replace the fixed-angle mechanical steering with **closed-loop pose tracking** — the car continuously estimates its position and heading and actively steers to follow a planned path to the target point (and through the can gate). This directly attacks stopping accuracy, timing, and run-to-run consistency, because the same estimator that steers also triggers a precise, self-correcting stop.*

Companion to `EV_How_It_Works.md`, `EV_Design_Review.md`, `EV_Rules_Scoring_Model.md`.

**Decision (this version):** after weighing both architectures against the constraint that the current build is **very compact and single-motor**, the chosen path is the **servo-steered front pivot** (Architecture B), implemented as a *retrofit of the existing steering pivot* rather than a ground-up rebuild. The differential-drive comparison is kept below for the rationale.

**Rules check:** multiple motors, a servo, an IMU, and a more capable microprocessor are all legal — propulsion just has to be electric from ≤ 8 AA (no lithium/lead-acid), and microprocessors/electronics are explicitly allowed.

---

## 1. The idea in one paragraph

Every control cycle (~200–1000 Hz), the car reads its distance encoder and the IMU, updates an estimate of its pose **(x, y, θ)** — where it is on the floor and which way it points — and compares that to a **planned path**. A steering controller commands the servo angle that nudges it back onto the path; a speed controller keeps it on schedule for the target time; and when the along-path distance reaches the target, it brakes on the encoder count. Nothing depends on a steering angle being *preset* perfectly, because errors are seen and corrected continuously instead of accumulating.

---

## 2. The localization stack

This is the heart of the redesign.

**Heading (θ) — from the BNO085 (fusion done on-chip).** The BNO085 runs its own sensor-fusion firmware and outputs a ready-made orientation quaternion — no gyro integration or bias handling in your code. Use the **Game Rotation Vector** report (accelerometer + gyro, **no magnetometer**): it gives a stable heading relative to power-on and is immune to the magnetic disturbance from motors/metal that would corrupt a magnetometer-based heading. Over a 10–20 s run its yaw is rock-steady. Your firmware just reads the quaternion and extracts yaw:
```
yaw = atan2( 2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz) )
```
The chip also streams a calibrated gyro yaw-rate you can use as feedforward for smoother steering.

**Distance (s) — from the free-rolling encoder.** Your current build already belt-drives the encoder off the **undriven front axle** — that *is* a free-rolling distance wheel, and it's the right way to do it: an undriven wheel doesn't slip under acceleration/braking the way a driven wheel does, so its count tracks true ground distance. Distance = counts × (calibrated wheel circumference / counts-per-rev). This is your precise along-path progress and your **stop trigger**.

**Pose update (dead reckoning).** Each tick, with along-path step `ds` and current heading `θ`:
```
x += ds * cos(θ)
y += ds * sin(θ)
```
θ comes from the BNO085; `ds` from the free roller.

**Why this beats the current design:** the old code trusts a *preset* mechanical angle to trace the right circle with zero feedback. Here, if the car drifts off heading, the estimator *sees* it and the servo corrects it every few milliseconds.

### Sensor division of labor
Each sensor does the one thing it is best at:

| Sensor | Job | Why it's the right source |
|---|---|---|
| **BNO085** | Heading θ | On-chip fusion, mag-free, drift-negligible over a 20 s run |
| **Free-rolling front encoder** | Distance + stop trigger | Undriven → no wheel slip → true ground distance |
| **(optional) drive-motor encoder** | Motor speed loop | Regulates cruise speed for the target time |

---

## 3. Chosen architecture — Servo-steered front pivot (retrofit)

**The key realization:** the current car *already has* active-steering hardware — the front axle pivots. It's just held at a fixed angle by the manual adjustment arm (set with the digital caliper). Motorize that same pivot and it becomes commandable.

**Mechanical change (minimal):** replace the fixed adjustment arm with a small **metal-gear servo** direct-coupled to the existing front pivot. Everything else stays: the 550 motor, the 2WD gearbox, rear drive, the BTS7960, and the belt-driven front encoder. No second motor, no widening — the compact single-motor layout is preserved.

**Kinematics / odometry (bicycle model; heading from the IMU):**
```
ds  = front (free-roller) encoder step
θ   = BNO085 heading (primary)
x += ds*cos(θ);  y += ds*sin(θ)
```

**Steering control:** a path follower (pure pursuit / Stanley) outputs a desired path curvature κ, converted to a commanded steering angle and sent to the servo:
```
δ = atan(κ * L)          // L = wheelbase
servo_us = center_us + k_servo * δ
```
The BNO085 sees the resulting heading and the loop trims δ — so servo backlash and centering imperfection are **absorbed by the feedback**, not baked into the result.

**Why this fits your build**
- Preserves the compact, single-motor drivetrain and reuses the existing pivot and front encoder.
- A tiny servo is far lighter and smaller than a second 550 + gearbox + driver.
- Closed-loop heading masks the servo's main weakness (backlash/trim).

**Watch-outs (and their fixes)**
- *Backlash / centering* → direct-couple the servo horn to the pivot (no sloppy linkage), use a metal-gear servo, let the IMU loop correct residual error. **(See `docs/hardware/CAD_SUMMARY.md` for the steering-slack analysis and the direct-coupling recommendation.)**
- *Heading depends entirely on the IMU* → fine here; the BNO085 is a trustworthy single source over a 20 s run.
- *Minimum turning radius* → irrelevant; event paths are straight or gentle arcs.

---

## 4. Why not differential drive (the comparison)

Differential drive (two independently driven wheels, steer by speed difference) is the textbook "no steering linkage" answer and has the best odometry (two encoders give heading directly). **But it needs two drive motors** — you cannot get true differential steering from a single motor without adding a differential gearbox plus per-wheel brakes/clutches, which is heavier and more complex than the thing we're trying to simplify. It also widens the drivetrain and abandons the free-rolling-encoder advantage the current car already has. Given the **compactness + single-motor** constraint, differential drive fights the build; the servo-steered pivot wins.

| Criterion | Differential drive | **Servo-steered pivot (chosen)** |
|---|---|---|
| Stopping accuracy | Excellent | Excellent (same encoder stop) |
| Timing accuracy | Excellent | Excellent |
| Consistency | Best (dual encoder + IMU) | Very good (IMU heading; backlash absorbed by loop) |
| Motors required | **Two** | **One (existing)** |
| Compactness | Wider, heavier | **Preserves current compact layout** |
| Reuses current build | Little | **Most of it** (motor, gearbox, driver, front encoder, pivot) |

---

## 5. Servo sizing (front-pivot steering)

Only the weight on the **front** wheels taxes the steering servo, and the car is **rear-heavy** (motor, gearbox, 8-AA pack in back).

- At a target vehicle mass of **~2–2.5 lb (0.9–1.13 kg)**, the front axle carries roughly 35–50% → ~**0.4–0.57 kg** (≈4–5.6 N).
- Stationary tire-scrub torque at that front load, small wheels on a smooth floor: about **0.3–0.8 kg·cm**. Rolling, it's a fraction of that.
- An **MG90S-class metal-gear micro servo (~2.0–2.5 kg·cm)** gives ~**3–5× margin even dry-steering while stopped**, and much more while moving. Comfortably enough.

Keep the margin healthy by: biasing mass rearward (also helps drive traction), a low-friction pivot bearing, small caster, and **steering only while rolling** (pre-set roughly straight before the start). If a bench test ever says you're tight, add a **servo→pivot linkage ratio** (short arm on servo, longer on pivot) to multiply torque — you have servo speed to spare.

**Definitive check (5 min):** hook a fish/luggage scale to the steering arm at a known distance from the pivot, pull until the wheels turn, torque = force × arm, on the real surface at final weight. Under ~half the servo's rated stall = safe.

*(Note: heavier mass doesn't hurt steering — it's fine — but it does make braking/stopping harder due to momentum. There's no mass limit in the rules; pick weight for traction vs. stopping, not for steering.)*

---

## 6. Hardware (locked to your parts)

- **MCU: Raspberry Pi Pico 2 (RP2350)** — dual Cortex-M33 @ 150 MHz with **hardware FPU** (pose/trig math is free) and **PIO** blocks that decode the quadrature encoder *in hardware* (zero CPU cost, no missed counts even at speed). Runs the Arduino `arduino-pico` core or the C/MicroPython SDK. **(This firmware uses the C/C++ pico-sdk.)**
- **IMU: Adafruit BNO085 (BNO08x)** — on-chip fusion; read the **Game Rotation Vector** over I2C (up to 400 kHz) or SPI using the Adafruit BNO08x library. Also gives a calibrated gyro rate for steering feedforward.
- **Steering servo:** a **metal-gear micro servo** (MG90S-class, ~2.0–2.5 kg·cm), direct-coupled to the existing front pivot.
- **Drive motor + driver:** **reuse** the 550 motor, 2WD gearbox, and the single **BTS7960** you already have.
- **Distance encoder:** **reuse** the existing belt-driven front-axle (free-rolling) encoder, wired into the Pico's PIO. *(Optional: an encoder on the drive motor for a tighter speed loop.)*
- Keep the **LCD + rotary dial + Set/Start buttons** front-end — that UI is fine and worth reusing.

**Electrical cautions (3.3 V logic):**
- RP2350 GPIO is **not 5 V-tolerant.** If the encoder outputs 5 V, level-shift its A/B lines down (or run it at 3.3 V if supported). The BNO085 breakout is happy at 3.3 V.
- Confirm the **BTS7960 logic inputs** switch reliably on 3.3 V PWM (they generally do; verify on the bench).
- Power the **servo from a 5 V rail** (regulated from the pack), not a Pico pin; common-ground everything (Pico, BNO085, BTS7960, servo).

---

## 7. Control stack (what the firmware will do)

1. **Estimator** — read BNO085 heading + free-roller distance → pose (x, y, θ), updated every tick.
2. **Planner** — from the entered target distance + the can-gate geometry, generate a path: a **straight line** if the cans sit on the centerline, or a smooth **curve** if they're offset. (Active control makes the path a *software* choice now — a big flexibility gain over the baked-in arc.)
3. **Lateral follower** — pure-pursuit or heading-PID → curvature command → **servo angle** `δ = atan(κ·L)`.
4. **Longitudinal profile** — pick a cruise speed from the target time (trapezoidal accel/cruise/decel) so you arrive on schedule.
5. **Closed-loop stop** — inside the last stretch, creep at low speed until the free-roller encoder hits the target count, then brake. The self-correcting stop we identified as your #1 scoring win.

---

## 8. Calibrations this design needs

- **Effective wheel circumference** (front roller, loaded) — drive a marked 5.00 m, read counts; this is the single most important distance constant.
- **Wheelbase L** — measure front-axle to rear-axle contact distance (used in the steering-angle math).
- **Servo center + angle scale** (`center_us`, `k_servo`) — find the microseconds for straight-ahead and the µs-per-degree, so commanded δ maps to real wheel angle.
- **Gyro bias** — handled by the BNO085's own calibration; just let it sit still ~1 s at power-on before a run.
- **Follower gains** — tune pure-pursuit lookahead / PID on the bench.

---

## 9. Open items

1. **Can placement** — pull the official track diagram to confirm where the cans can sit, which decides whether the planned path is a straight line or a curve. *(Recommended next research step.)*
2. **Scope of next deliverable** — a **detailed design**: full pin map, wiring diagram, state machine, and control-loop pseudocode for the Pico (still on paper; no code committed to the build until you say so). **(Done — see `EV_Detailed_Design.md` and the implemented firmware in `src/`.)**

*Design locked: servo-steered front-pivot retrofit · single 550 rear drive (reused) · Raspberry Pi Pico 2 (RP2350) · Adafruit BNO085 (Game Rotation Vector) · free-rolling front encoder for distance, decoded in PIO · MG90S-class metal-gear steering servo.*
