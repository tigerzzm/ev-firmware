# EV firmware — harvest notes

What this project is, where each piece came from, what changed, and what's left
to write. Source of the harvest: **robotour-pico** (RP2040 differential-drive
Robot Tour firmware). Target of the harvest: **EV** — single-motor, servo-steered,
straight-arc drag-and-stop vehicle on **Raspberry Pi Pico 2 (RP2350)**.

Design references (in the project): `EV_Detailed_Design.md`,
`EV_Active_Tracking_Proposal.md`, `EV_Design_Review.md`, `EV_Rules_Scoring_Model.md`.

> **Verified in a host harness, not on hardware** (no pico-sdk/ARM toolchain here):
> all modules compile-check clean under `-Wall -Wextra` and the math is unit-tested.
> Build on your Pico setup with `cmake && make` (the CMake keeps the Pico VS Code
> extension block). See "Readiness & what was verified" and "Build" below.

---

## Decisions locked with you

- **MCU:** Pico 2 (RP2350). Ports cleanly from robotour's RP2040; gains a hardware FPU.
- **Speed/time control:** closed-loop, but **off the free-roller** — no separate
  drive-motor encoder. The one front encoder does both the precise stop *and* the
  ground-speed signal. (Optional drive-motor encoder pins left commented in `config.h`.)
- **IMU report:** Game Rotation Vector (mag-free), per your design.

---

## Readiness & what was verified (deep pass, July 2026)

This sandbox has no ARM toolchain, so every module was **compile-checked** and the
math **unit-tested** in a host harness (mock pico-sdk / BNO08x headers matching the
API surface actually used).

- **Compiles clean:** all 14 source modules pass `g++ -std=gnu++17 -fsyntax-only
  -Wall -Wextra`. Catches syntax, type, undeclared-symbol, and signature errors —
  not linker output or hardware behavior.
- **Math verified (unit tests pass):** arc geometry (start/end/apex all lie on the
  planned circle to 1e-3; cross-track = 0 at apex; L > chord), the straight-line
  degenerate case (k=0, L=D, cross-track=y), the speed trapezoid's cruise-time
  integral (~15.9 s for a 15 s target — placeholder profile, refined by the PWM
  table), and the angle utilities.
- **Bug fixed — cross-core I2C contention:** the estimator (core1, IMU on I2C0) and
  the LCD (core0, same bus) could hit the shared bus simultaneously. The estimator
  is now **gated** (`poseSetActive`): it runs only between LAUNCH and BRAKE, and the
  LCD is written only while it's inactive (car stopped). No bus mutex required.
- **Smaller fixes:** portable format specifier in `imu` logging; init-order warning
  in `pid`; the speed trapezoid is floored so cruise never coasts to a stall before
  the closed-loop creep takes over.
- **Robustness added:** APPROACH now has a **stall guard** (steps creep PWM up if the
  car isn't progressing) and CRUISE **aborts to a safe stop on IMU loss** instead of
  coasting blind — the last two `sm.cpp` TODOs.
- **Re-runnable harness:** the compile-check + math tests are committed under
  `host_check/` — run `bash host_check/run.sh` anytime (needs only `g++`).

**What this does NOT prove:** it is not a real RP2350 build/flash and does not
exercise hardware timing, I2C/PIO behavior, or the calibration values. First
on-hardware step is `cmake && make` (expect minor toolchain-specific tweaks), then
the bench bring-up below.

## Provenance & status — module by module

| EV module (`src/`) | From robotour | Status |
|---|---|---|
| `BNO08x/` (whole dir) | `src/BNO08x/` | **verbatim** — vendored Adafruit BNO08x + sh2 lib |
| `quadrature_encoder.pio` | same | **verbatim** — PIO quadrature decoder |
| `ram_logger.*`, `ram_printf.*` | same | **verbatim** — optional RAM logging (not wired in; include `ram_printf.h` in a file to auto-capture its printf) |
| `position.h` | same | **verbatim** — 2D pose struct + helpers |
| `utils.*` | same | **adapted** — only change: `utils.h` includes `position.h` instead of `chassis.h` |
| `pid.*` | `components/PID.*` | **verbatim** (renamed) — scalar PID (used by steering) |
| `motor_controller.*` | `components/Motor.*` | **verbatim** (renamed) — PID+feedforward velocity controller for the drive speed loop |
| `imu.*` | `imu.*` | **adapted** — see below |
| `odo.*` | `odom.cpp` (read pattern) | **new/derived** — single free-roller distance via PIO |
| `pose.*` | `odom.cpp` (structure) | **new/derived** — bicycle-model dead reckoning, mutex-guarded, IMU-zeroed-at-arm |
| `config.h` | — | **new** — EV pin map + calibration constants |
| `drive.*` | `l298n.*` (PWM pattern) | **new** — BTS7960 single-motor driver |
| `path.*` | — | **new** — constant-curvature arc geometry (from `EV_Detailed_Design §0`) |
| `steer.*` | — | **new** — servo I/O is real; control-law gains/signs are bench TODO |
| `speed.*` | — | **new** — trapezoid + PWM→speed table (table is empty until calibrated) |
| `sm.*` | `main.cpp` (2-core idea) | **new** — state-machine skeleton (`EV_Detailed_Design §5`), real module calls, some TODO branches |
| `ui.*` | `code_ref/main_code` | **ported** — dial+Set entry flow with increment cycling, Arduino→Pico |
| `lcd.*` | — | **new** — minimal PCF8574 I2C 16x2 driver (replaces Arduino `LiquidCrystal_I2C`) |
| `main.cpp` | `main.cpp` (structure) | **new** — core0 = state machine, core1 = estimator |

**Deliberately NOT harvested** (irrelevant to a straight-arc vehicle): grid path
generation, in-place turns (`turnTo`/`timedTurnToHeading`), A→B→A reverse maneuvers,
ultrasonic wall localization, TCS34725 color finish sensor, and the L298N dual-motor
driver. Also skipped robotour's *time-budget velocity* (it couples distance and
time; your design intentionally decouples them).

---

## IMU changes (the highest-value reuse)

`imu.cpp` keeps robotour's battle-tested init-with-retries, reset-recovery, and
health-check scaffolding. What changed for the EV:

1. **Report:** `GYRO_INTEGRATED_RV (0x2A)` → `GAME_ROTATION_VECTOR (0x08)`; reads
   `event.un.gameRotationVector`.
2. **ARVR report dropped** — robotour enabled it only to make the gyro report
   stream; Game Rotation Vector streams on its own.
3. **I2C pins:** `i2c0` on **GP4/GP5** (was GP20/GP21); reset **GP7**.
4. **API surface:** `imuInit()`, `imuYawDeg()` (raw degrees, held on miss),
   `imuHealthy()`. Heading is zeroed at ARM in `pose.cpp` (same offset trick as
   robotour's `resetValues.theta`), not inside the driver.

---

## What is REAL vs what you still write/tune

**Real and ready** (structure + logic complete; still needs calibration numbers):
`imu`, `odo`, `pose`, `drive`, `path`, servo I/O in `steer`, the trapezoid in `speed`,
the `lcd` driver, the dial entry in `ui`, the two-core wiring in `main`, and the run
sequence in `sm`.

**You still write / tune:**
- `ui.cpp` — **ported and working**; on the bench confirm the dial direction
  (`DIR_SIGN`) and that the buttons read active-low as wired.
- `speed.cpp` — fill `pwmForSpeed()` from the measured **PWM→speed table**; enable the
  optional inner loop once `MotorController` gains are set.
- `steer.cpp` — set `Kh`, `Kc`, and **verify the signs** of `e_ct`/`e_head`/`SERVO_US_PER_RAD`
  on the bench (push the car off the arc; the servo must steer it back).
- `sm.cpp` — stall guard and IMU-loss abort are now **implemented**; only the
  launch/brake PWM *values* remain to tune (they live in `config.h`).

**Calibration constants in `config.h` that MUST be set before a real run:**
`M_PER_COUNT` (drive a marked 5.000 m), `WHEELBASE_L_M`, `SERVO_CENTER_US`,
`SERVO_US_PER_RAD`. `M_PER_COUNT` ships as `0.0` on purpose so an uncalibrated
build reads zero distance and the mistake is obvious.

---

## Cross-reference: your existing Arduino EV kit (`code_ref/`)

The scaffold was harvested from **robotour-pico**, not from your current EV kit —
that folder was reviewed *after* the harvest. What it confirmed / contributed:

- **`getArcLength()`** in `code_ref/main_code` is the same arc math as `path.cpp`
  (`pathPlan`). The kit hardcodes the bulge as `arcHeight = 100 − (width+margin)/2 =
  92.5 cm`; `path.cpp` takes `h` as a parameter and matches it at `h = 0.925 m`.
  Captured as `GATE_BULGE_H_M` in `config.h`.
- **`getEncoderValue()`** gives a ballpark `M_PER_COUNT ≈ 0.000191 m/count`
  (7.3025 cm wheel, 1200 PPR) — noted in `config.h` as the starting estimate.
- **LCD + dial entry flow** (`set==0/1/2`, increment cycling) — **ported** into
  `ui.cpp`, with a new `lcd.*` PCF8574 driver replacing Arduino `LiquidCrystal_I2C`
  (`digitalRead`→`gpio_get`; the crude CLK/DT poll upgraded to a KY-040 falling-edge
  decode). `RobojaxBTS7960` is Arduino-only; `drive.cpp` is the Pico equivalent.
- **NOT reused:** the kit's `cruise → calibrate → open-loop snail` stop — that's the
  open-loop approach your Design Review flagged; the scaffold replaces it with the
  closed-loop encoder-triggered stop in `sm.cpp` APPROACH.

## Build (Pico 2)

```
# with the Pico SDK available (the CMake fetches it from git if PICO_SDK_PATH is unset)
mkdir build && cd build
cmake ..                 # PICO_BOARD is pinned to pico2 in CMakeLists.txt
make -j                  # -> ev_firmware.uf2
```
Flash `ev_firmware.uf2` to the Pico 2 (BOOTSEL). USB stdio is on; UART off.

> Electrical reminder (from your design): RP2350 GPIO is **not** 5 V-tolerant —
> level-shift a 5 V front encoder, and power the servo from a 5 V rail, common-ground.

## Bench bring-up checklist (do in this order)

Each step nails down specific constants/signs before the next depends on them.
Keep the wheels off the ground through step 5.

1. **Compile & flash.** `cmake .. && make`; flash `ev_firmware.uf2`. Open USB serial —
   expect the boot banner and "IMU ready".
2. **I2C bring-up.** LCD shows text and IMU init succeeds. Blank LCD → check 0x27
   address + 4.7k pull-ups. IMU init fail → check wiring / that a 5 V encoder isn't
   on a 3.3 V pin.
3. **IMU heading sign.** Rotate the car CCW (left) by hand; serial heading should
   INCREASE. If it decreases, flip `HEADING_SIGN` in `pose.cpp`.
4. **Encoder distance (`M_PER_COUNT`).** Roll forward exactly 5.00 m by hand, read the
   count; set `M_PER_COUNT = 5.0 / counts` in `config.h`. Confirm forward motion
   counts UP (else swap encoder A/B).
5. **Servo map.** Find `SERVO_CENTER_US` (wheels dead straight) and `SERVO_US_PER_RAD`
   (command a known angle, measure actual). Confirm dial direction (`DIR_SIGN` in
   `ui.cpp`).
6. **Motor direction + speed table.** On blocks, `driveForward` spins the wheel
   FORWARD (else swap RPWM/LPWM). Build `pwmForSpeed()` in `speed.cpp`: at PWM
   30/40/…/100 %, log steady cm/s.
7. **Steering sign (critical).** Push the car off a set arc while it "runs" — the
   servo must steer BACK toward the arc. If it runs away, flip the feedback sign /
   `SERVO_US_PER_RAD` (see `steer.cpp` TODO).
8. **Straight-line hold.** Set h→0. Run 5 m; measure heading drift + stopping spread.
   Tune `CREEP_PWM`, `CREEP_ZONE_M`, `BRAKE_PULSE_MS` for a dead stop.
9. **Arc hold + gate.** Run the real arc; confirm it threads the can gate; measure
   stop error over ~10 runs (consistency is the score).

*(The host compile-check harness used for the deep pass — mock headers + math tests —
can be regenerated on request; it isn't part of the flashed firmware.)*
