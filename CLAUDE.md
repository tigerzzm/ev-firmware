# CLAUDE.md — project brief for Claude Code

Read this first. It orients you on the EV firmware; deeper detail is linked below.

## What this is

Firmware for a **Science Olympiad Electric Vehicle (Division C, 2027 season)** on a
**Raspberry Pi Pico 2 (RP2350)**. The car drives a single gentle **arc** from Start to
a Target Point and must **stop at an exact distance** at an exact **time**, threading a
two-can gate for a bonus. Scoring (lowest wins, better of 2 runs): **distance = 2 pt/cm
(dominant), time = 1 pt/s**. So the #1 goal is a precise, repeatable **stop**.

## How it works (one paragraph)

A **free-rolling front encoder** measures true ground distance (and triggers the
closed-loop stop). A **BNO085 IMU** gives mag-free heading (Game Rotation Vector). A
**servo** steers a center-pivot front axle to hold a constant-curvature arc. A single
**550 motor** through a **BTS7960** drives the rear. **Core 1** runs the pose estimator
(x, y, θ) at 500 Hz; **core 0** runs the state machine: enter D & T on the dial → arm →
#2-pencil trigger → cruise the arc on a time schedule → creep → stop on encoder count.

## Repo layout

- `src/` — the firmware. Module map:
  - `main.cpp` (core0=state machine, core1=estimator), `sm.*` (state machine),
    `config.h` (**pin map + calibration constants**).
  - `imu.*` (BNO085 Game Rotation Vector), `odo.*` (free-roller distance via PIO),
    `pose.*` ((x,y,θ) dead-reckoning, I2C-gated), `path.*` (arc geometry),
    `steer.*` (servo + Stanley control), `speed.*` (trapezoid + PWM→speed table),
    `drive.*` (BTS7960), `ui.*` + `lcd.*` (LCD + dial), plus harvested
    `pid/motor_controller/utils/position/ram_*` and `quadrature_encoder.pio`.
  - `BNO08x/` — vendored Adafruit BNO08x + sh2 library (do not edit).
- `docs/` — hardware build docs: `START_HERE.md`, `BOM.md`, `WIRING_GUIDE.md` +
  `wiring_diagram.html`, `VEHICLE_BUILD.md`, `BUILD_AND_FLASH.md`, `BRINGUP_AND_TUNING.md`.
- `docs/design/` — the design record (rules, scoring, design review, the
  active-tracking proposal, the detailed design, the session log). **Read these for
  the "why."**
- `docs/hardware/` — `CAD_SUMMARY.md` (mechanical parts, dimensions, the steering
  mechanism + slack analysis) and rendered illustrations. The real STLs are in
  `~/Documents/EV_2026/CAD/` (binary; open the parent EV_2026 folder / use a multi-root
  workspace to see them alongside this repo).
- `host_check/` — offline compile + math test harness. Run `bash host_check/run.sh`.
- `HARVEST_NOTES.md` — provenance of every module, what was changed, readiness status.

## Hardware decisions (locked)

Pico 2 (RP2350) · BNO085 via Game Rotation Vector (I2C0 GP4/GP5, addr 0x4A) ·
BTS7960 + single 550 rear motor · **free-rolling front encoder** for distance (single
encoder does both stop-trigger and speed — no drive-motor encoder) · MG90S-class servo
on the front pivot · reuse the LCD + dial UI. **Removed:** the digital caliper + its
mounts + the adjustment-arm hook (the servo + IMU loop replaces the hand-set angle).
RP2350 is **3.3 V, not 5 V-tolerant** — level-shift a 5 V encoder; run the LCD at 3.3 V.

## Before it runs on hardware — calibration (ships at zero ON PURPOSE)

Set these in `src/config.h` / `src/speed.cpp` after bench measurement (see
`docs/BRINGUP_AND_TUNING.md`). An uncalibrated build reads 0 distance and never stops —
that's the intended tripwire.
- `M_PER_COUNT` (roll a marked 5.00 m), `WHEELBASE_L_M`, `SERVO_CENTER_US`,
  `SERVO_US_PER_RAD`, the `pwmForSpeed()` table, steering gains `Kh`/`Kc`.
- Confirm four **signs** on the bench: `HEADING_SIGN` (pose.cpp), encoder direction
  (odo.cpp), dial `DIR_SIGN` (ui.cpp), steering feedback (steer.cpp).

## Status & how to verify

All modules compile-clean under `-Wall -Wextra` in the host harness; math is
unit-tested; cross-core I2C contention fixed; APPROACH stall-guard + IMU-loss abort
implemented. **Never run on hardware yet.** After any change, run
`bash host_check/run.sh` (needs only `g++`) — it must stay green. The real build is
`cmake && make` with the Pico SDK.

## Conventions / gotchas

- **Don't edit `CMakeLists.txt` or `pico_sdk_import.cmake` casually** — the Pico VS Code
  extension regenerates them with local SDK paths; they may show as locally modified.
- Keep calibration constants as named constants in `config.h`, not magic numbers.
- The estimator is **gated** (`poseSetActive`) so core1 (IMU) and core0 (LCD) never hit
  the shared I2C0 bus at once — don't add LCD writes during a run.
- Open steering question: which feature is the true pivot axis (decides direct-coaxial
  vs arm-driven servo mount) — see `docs/hardware/CAD_SUMMARY.md`.

## Git / workflow

Remote: `github.com/tigerzzm/ev-firmware`. This clone is the working source of truth.
Build/flash/commit/push happen here in VS Code (full toolchain + network).
