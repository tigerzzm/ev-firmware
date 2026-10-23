# CLAUDE.md — project brief for Claude Code

Read this first. It orients you on the EV firmware; deeper detail is linked below.

> ## ⚠️ 2027 RULES CHANGED THE TASK (2026-10-18) — read this before anything else
>
> The 2027 Electric Vehicle rules replace the can-gate arc with a **bottle task**: drive
> forward, **capture a water bottle** on the centerline (2.0 / 3.0 / 4.0 m by level), **push
> it past a Bottle Line** 1–3 m beyond the Target, then **reverse** so the **rear**
> Measurement Point stops on the Target. Target Distance is 5–9 m; **no can bonus**; time
> is only 0.5 pt/s and a **pre-motion start delay is legal** (any stop ≥ 3 s after first
> motion ends the run). New scoring terms: bottle distance 1 pt/cm, −20 bottle-past-line
> bonus, pusher-width bonus (W − 35 cm)·1.5.
>
> **The firmware in `src/` was rewritten for the 2027 task on 2026-10-18** (not yet run
> on hardware). The current source of truth for the 2027 design is:
> - `docs/rules/EV_Rules_2027.pdf` — **the official 2027 rules** (text copy:
>   `docs/rules/EV_Rules_2027.txt`, easier to grep). The rules win over every other doc.
>   **Exception:** §3.j (Design Knowledge) and §3.l (program authorship) were removed in
>   the latest version — ignore them; this PDF predates the change.
> - `docs/design/EV_Rules_Scoring_Model.md` — 2027 rules & scoring
> - `docs/design/EV_Detailed_Design.md` — 2027 run plan, state machine, control laws,
>   calibration, test plan, pusher/MP mechanics
> - `docs/design/EV_Session_Log.md` — newest entry (2026-10-18) lists decisions and
>   which parts of the other docs are superseded
>
> Anything below that mentions the arc, the can gate, `path.*` arc geometry, the front MP
> or "time = 1 pt/s" is 2026-era and superseded.

## What this is

Firmware for a **Science Olympiad Electric Vehicle (Division C, 2027 season)** on a
Raspberry Pi Pico (the bench board is a **Pico 1 / RP2040**; see git log). *2026-era
description, kept for context:* the car drove a single gentle **arc** from Start to a
Target Point, stopping at an exact distance and time while threading a two-can gate.
**For 2027** it drives a straight line, pushes a bottle past a line, and reverses to put
its rear MP on the Target — see the banner above.

## How it works (one paragraph)

A **free-rolling front encoder** measures true ground distance in both directions; the
final stop is on its **net** count. A **BNO085 IMU** gives mag-free heading (Game
Rotation Vector). A **servo** steers a center-pivot front axle to hold the centerline
(forward: Stanley or pure pursuit, `STEER_FWD_LAW`; reverse: aim-at-target, pure
pursuit with the goal behind, or Stanley adapted for reverse, `STEER_REV_LAW` — both
in config.h §4). A single **550 motor** through a
**BTS7960** drives the rear, under a PI speed loop. **Core 1** runs the pose estimator
(x, y, θ) at 500 Hz; **core 0** runs the state machine: enter D, B, T, level, offsets →
plan → arm → #2-pencil trigger → start delay → cruise → capture → push → turnaround →
reverse → stop on net encoder count → hold. See `docs/OPERATING_GUIDE.md`.

## Repo layout

- `src/` — the firmware. Module map:
  - `main.cpp` (core0=state machine, core1=estimator), `sm.*` (state machine),
    `config.h` (**pin map + calibration constants**).
  - `imu.*` (BNO085 Game Rotation Vector), `odo.*` (free-roller distance via PIO,
    forward/reverse counts split), `pose.*` ((x,y,θ) of the roller + rear-MP helper),
    `plan.*` (D,B,T,level → stops, speeds, start delay — pure), `steer_law.*`
    (forward/reverse steering laws — pure), `steer.*` (servo I/O), `speed.*`
    (profiles + PI speed loop — pure), `drive.*` (BTS7960 incl. dynamic-brake hold),
    `ui.*` + `lcd.*` (LCD + dial), `cal.*` (bench menu), plus harvested
    `utils/position/ram_*` and `quadrature_encoder.pio`.
  - `BNO08x/` — vendored Adafruit BNO08x + sh2 library (do not edit).
- `docs/` — hardware build docs: `START_HERE.md`, `BOM.md`, `WIRING_GUIDE.md` +
  `wiring_diagram.html`, `VEHICLE_BUILD.md`, `BUILD_AND_FLASH.md`, `BRINGUP_AND_TUNING.md`.
- `docs/design/` — the design record (rules, scoring, design review, the
  active-tracking proposal, the detailed design, the session log). **Read these for
  the "why."** Includes **`EV_Steering_Linkage_Design.md`** — measured CAD geometry,
  the drag-link vs coaxial decision, the ratio model, and firmware constraints that
  `config.h` does not yet reflect. **Read it before touching `steer*`.**
- `docs/hardware/` — `CAD_SUMMARY.md` (mechanical parts, dimensions, the steering
  mechanism + slack analysis) and rendered illustrations. ⚠️ Three of its steering
  claims are superseded — see §11 of `EV_Steering_Linkage_Design.md`. The real STLs are in
  `~/Documents/EV_2026/CAD/` (binary; open the parent EV_2026 folder / use a multi-root
  workspace to see them alongside this repo).
- `host_check/` — offline compile + math test harness. Run `bash host_check/run.sh`.
- `HARVEST_NOTES.md` — provenance of every module, what was changed, readiness status.

## Hardware decisions (locked)

Pico 1 (RP2040, `PICO_BOARD pico`) · BNO085 via Game Rotation Vector (I2C0 GP20/GP21, addr 0x4A) ·
BTS7960 + single 550 rear motor · **free-rolling front encoder** for distance (single
encoder does both stop-trigger and speed — no drive-motor encoder) · MG90S-class servo
on the front pivot · reuse the LCD + dial UI. **Removed:** the digital caliper + its
mounts + the adjustment-arm hook (the servo + IMU loop replaces the hand-set angle).
The RP2040 is **3.3 V, not 5 V-tolerant** — pull the open-collector encoder up to 3.3 V;
run the LCD at 3.3 V. Full pin map: `src/config.h` §6.

## Before it runs on hardware — calibration (ships at zero ON PURPOSE)

`config.h` §1 ships `M_PER_COUNT`, `L_P_M`, `ENC_FROM_MP_M` and `SERVO_US_PER_RAD` at
0, and `sm.cpp` **refuses to arm** until they are set (LCD: `NOT CALIBRATED <name>`),
so an uncalibrated car never moves on a run. `M_PER_COUNT_REV` also ships at 0, but
0 means "same as forward", and `WHEELBASE_L_M` is required only for pure pursuit. Procedure:
`docs/OPERATING_GUIDE.md` Part 1. Both encoder scales matter; the forward one most
(it multiplies the long forward leg).
- Then: `SERVO_CENTER_US`, the speed feedforward line (`SPEED_FF_*`), `KICK_PWM`,
  steering/speed gains, `T_BIAS_S`.
- `DELTA_MAX_RAD` = 0.12 rad (±7°, the linkage limit) — verify on the car.
- Confirm four **signs** (`config.h` §3): `HEADING_SIGN`, `ENCODER_SIGN`,
  `DIAL_DIR_SIGN`, `STEER_DIR_SIGN` (bench menu 7 checks steering both ways).

## Status & how to verify

2027 rewrite compiles clean under `-Wall -Wextra` in the host harness. `host_check`
tests the plan over the full rules grid, the time model against the commanded
profiles, the PI speed loop against a mismatched motor model, and **both steering laws
closed-loop in a bicycle model** (a sign error makes them diverge and fail), and
re-runs the tests for all six steering-law combinations with measured-style geometry.
The real tree builds clean with Pico SDK 2.3.0 for the RP2040. **Not yet run on
hardware.** After any change, run
`bash host_check/run.sh` (needs only `g++`) — it must stay green. The real build is
`cmake && make` with the Pico SDK.

## Conventions / gotchas

- **Don't edit `CMakeLists.txt` or `pico_sdk_import.cmake` casually** — the Pico VS Code
  extension regenerates them with local SDK paths; they may show as locally modified.
- Keep calibration constants as named constants in `config.h`, not magic numbers.
- **The IMU and LCD are on SEPARATE I2C buses** (IMU = I2C0 GP20/GP21, LCD = I2C1
  GP14/GP15), so they cannot collide. Even so, **nothing on the motion path prints or
  writes the LCD**: a USB printf can block ~500 ms and an LCD write ~30 ms, either of
  which delays a stop. The run logs to RAM and prints after the car has stopped; the
  LCD is only written during the (stationary) start delay.
- The run must never stand still ≥ 3 s after first motion (rules §8.p ends the run);
  `config.h` static_asserts the turnaround and stall budgets against that.
- **Every tunable lives in `src/config.h`**, organised §1–§8. No magic numbers
  elsewhere — if you add a constant, hoist it there.
- ~~Open steering question: which feature is the true pivot axis~~ **Resolved
  2026-08-16:** the vertical Ø12.97 bore at (15.63, 36.01) in `EV-front-axle-v2.stl`,
  dead centre in the beam's X span and on the wheel axle line (trail = 0.03 mm). The
  mount is **arm-driven via a drag link**, not coaxial — the working steering angle is
  only 1–4°, so a 1:1 coaxial mount would pass the servo's full 1–1.5° of gear lash
  straight to the beam. r_lever = **89.09 mm**. See `docs/design/EV_Steering_Linkage_Design.md`.

## Git / workflow

Remote: `github.com/tigerzzm/ev-firmware`. This clone is the working source of truth.
Build/flash/commit/push happen here in VS Code (full toolchain + network).
