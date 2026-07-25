# EV 2026 — START HERE (tomorrow's plan)

You're setting up hardware and flashing the firmware tomorrow. Do it in this order;
each doc is self-contained and cross-links the others.

## The order of operations

1. **`BOM.md`** — lay out all parts, confirm nothing's missing, skim the pin map.
2. **`VEHICLE_BUILD.md`** — the mechanical build: servo-steer retrofit on the front
   pivot, mounting the electronics, the free-roller encoder, weight/measurement point.
   Do the mechanical assembly first so you have a platform to wire onto.
3. **`WIRING_GUIDE.md`** + **`wiring_diagram.html`** — wire the electronics. Follow the
   power tree exactly (this is where a mistake fries a part). Finish with the
   **pre-power checklist** before you ever connect the battery.
4. **`BUILD_AND_FLASH.md`** — install the toolchain, build, flash `ev_firmware.uf2`,
   open the serial monitor.
5. **`BRINGUP_AND_TUNING.md`** — the 9-step bench bring-up (wheels off the ground
   first), each step with the expected serial output, the constant/sign it nails
   down, and troubleshooting. This is where the car goes from "powered" to "runs."

## The one-paragraph mental model

The car drives a single gentle **arc** from Start to the Target Point and must
**stop at an exact distance** at an exact **time**. A **free-rolling front encoder**
measures true ground distance (and triggers the precise stop — the #1 score driver).
A **BNO085 IMU** gives heading. A **servo** steers the front pivot to hold the arc.
A single **550 motor** through a **BTS7960** drives the rear. A **Pico 2** runs it all:
core 1 estimates pose (x, y, heading) at 500 Hz, core 0 runs the state machine
(enter D & T on the dial → arm → pencil trigger → cruise the arc → creep → stop).

## What's already done (software)

The firmware in `src/` is written, compiles clean (host-checked under `-Wall
-Wextra`), and its math is unit-tested. See `HARVEST_NOTES.md` for the full status,
the provenance of every module, and the "Readiness & what was verified" section.

## What only YOU can do tomorrow (the firmware can't guess these)

Set these in `src/config.h` after measuring on the bench — they ship as
zero/placeholder ON PURPOSE so an uncalibrated build fails obviously:

- `M_PER_COUNT` — ground distance per encoder count (drive a marked 5.00 m).
- `WHEELBASE_L_M` — front-axle to rear-axle, in metres.
- `SERVO_CENTER_US`, `SERVO_US_PER_RAD` — servo straight-ahead + angle scale.
- `pwmForSpeed()` in `speed.cpp` — the PWM→speed table (for hitting target time).

And confirm four **signs** on the bench (each is a one-line flip): IMU heading
(`HEADING_SIGN` in `pose.cpp`), encoder direction (`odo.cpp`), dial direction
(`DIR_SIGN` in `ui.cpp`), and steering feedback (`steer.cpp`). `BRINGUP_AND_TUNING.md`
walks each one.

## Safety, every time

- 3.3 V logic is **not** 5 V-tolerant. Never put 5 V on a Pico GPIO. Level-shift a
  5 V encoder; power the LCD at 3.3 V (or behind a level shifter).
- Common-ground everything. Route motor current on its own thick leads.
- Bench with wheels off the ground until the bring-up says otherwise.
- The car must not move until the pencil trigger — that's built in (ARMED waits for
  START), but double-check before every impound.
