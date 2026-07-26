# EV — Hardware Build & Wiring Guide (condensed)

Condensed durable reference. The full step-by-step lives in `docs/` (START_HERE, BOM,
WIRING_GUIDE + wiring_diagram.html, VEHICLE_BUILD, BUILD_AND_FLASH, BRINGUP_AND_TUNING).

## Architecture recap

Single-motor, servo-steered vehicle on a **Raspberry Pi Pico 2 (RP2350)**. Core 1
estimates pose (x, y, heading) at 500 Hz; core 0 runs the state machine. A
**free-rolling front encoder** gives true ground distance and triggers a closed-loop
stop (the #1 score driver); a **BNO085** gives mag-free heading (Game Rotation
Vector); a **servo** steers the front pivot to hold a constant-curvature arc through
the can gate; a single **550 motor** via a **BTS7960** drives the rear.

## Pin map (Pico 2, physical pins in parens)

| Function | GP | Pin | | Function | GP | Pin |
|---|---|---|---|---|---|---|
| I2C SDA (BNO+LCD) | GP4 | 6 | | Servo signal | GP15 | 20 |
| I2C SCL (BNO+LCD) | GP5 | 7 | | BTS7960 RPWM | GP16 | 21 |
| BNO INT (opt) | GP6 | 9 | | BTS7960 LPWM | GP17 | 22 |
| BNO RST (opt) | GP7 | 10 | | BTS7960 R_EN+L_EN | GP18 | 24 |
| Front enc A | GP10 | 14 | | Dial CLK | GP20 | 26 |
| Front enc B | GP11 | 15 | | Dial DT | GP21 | 27 |
| START (pencil) | GP14 | 19 | | Dial SW | GP22 | 29 |
| SET | GP19 | 25 | | 3V3 out / VSYS / GND | — | 36 / 39 / 3,8,13… |

## Power tree

- **Battery (8×AA NiMH, ~9.6 V)** → BTS7960 `B+/B−` (motor, 12 AWG) **and** → buck.
- **Buck → 5 V** → servo V+, BTS7960 `VCC` (logic), Pico `VSYS` (pin 39, ≤5.5 V).
- **Pico 3V3 (pin 36)** → BNO085, LCD (see note), KY-040 dial, level-shifter LV.
- **All grounds common.** Route motor current on its own thick leads.

## The 3.3 V / 5 V rule (don't fry the Pico)

RP2350 GPIO is **not 5 V-tolerant.** Two gotchas: a **5 V encoder** must have its A/B
level-shifted to 3.3 V; a **PCF8574 LCD** must run at **3.3 V** (turn up the contrast
pot) or sit behind a bidirectional I2C level shifter on the 5 V side. BNO085 is 3–5 V
tolerant with onboard pull-ups (addr 0x4A); LCD addr 0x27.

## Verified facts (sources)

- BNO085: addr 0x4A, VIN 3–5 V, onboard 10 K I2C pull-ups — Adafruit pinouts.
- BTS7960/IBT-2: VCC 5 V logic, inputs accept 3.3 V, motor 6–27 V / 43 A — module docs.
- Pico 2 pin-compatible with Pico 1; physical pins as above — pico2.pinout.xyz.

## Bring-up order (each nails a constant/sign)

1. Compile & flash (`cmake && make`; BOOTSEL → drag uf2). Serial shows "IMU ready".
2. I2C health (LCD text + IMU init together).
3. IMU heading sign → `HEADING_SIGN` (pose.cpp).
4. Encoder scale → `M_PER_COUNT` (roll 5.00 m); confirm counts up forward.
5. Servo map → `SERVO_CENTER_US`, `SERVO_US_PER_RAD`, `WHEELBASE_L_M`.
6. Motor direction + PWM→speed table (`pwmForSpeed` in speed.cpp).
7. Steering sign (push off arc → must steer back) — steer.cpp gains/signs.
8. Straight-line hold + stop tuning (`CREEP_PWM`, `CREEP_ZONE_M`, `BRAKE_PULSE_MS`).
9. Arc hold + can gate (`GATE_BULGE_H_M`, steering gains `Kh`/`Kc`); measure spread.

## Firmware status

Written, harvested from robotour-pico + the existing kit's UI, all modules
compile-clean under `-Wall -Wextra` (host harness with mock headers), math
unit-tested, cross-core I2C contention fixed, APPROACH stall guard + IMU-loss abort
implemented. Calibration constants ship at zero/placeholder on purpose. Re-run checks
with `bash host_check/run.sh`. Full detail in `HARVEST_NOTES.md`.
