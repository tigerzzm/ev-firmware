# EV — Bench Bring-up & Tuning

The path from "it powers on" to "it runs the arc and stops on target." Do the steps
**in order** — each nails down a constant or a sign that later steps depend on.
**Wheels off the ground through step 6.** Keep the serial monitor open the whole time.

For each step: what to do → what you should see → what it sets → if it's wrong.

---

## Step 0 — Power on (wheels up)

**Do:** run the pre-power checklist in `WIRING_GUIDE.md §9`, connect the battery, plug
USB for serial.
**See:** the boot banner and `IMU ready`. The LCD backlights and shows the distance
entry screen. The motor is silent; the servo twitches to center and holds.
**If wrong:**
- No serial at all → wrong port, or USB cable is charge-only.
- Banner but IMU init fails (`IMU I2C init failed…`) → I2C wiring, pull-ups, or the
  BNO085 isn't at 0x4A. Check SDA=GP4/SCL=GP5 and 3.3 V on VIN.
- LCD blank → contrast pot (turn it), address (0x27 vs 0x3F), or it's under-powered at
  3.3 V (see `WIRING_GUIDE §4`).
- Motor creeps or servo slams to a rail → **stop, kill power**; likely a driver enable
  or servo signal miswire.

---

## Step 1 — I2C bus health (LCD + IMU coexist)

**Do:** just watch. Both devices share I2C0.
**See:** LCD stable text **and** `IMU ready` in the same boot. The estimator is gated
off until a run starts, so the LCD has the bus to itself during setup — that's by
design (`poseSetActive`).
**Sets:** nothing — confirms the shared bus works.
**If wrong:** if the LCD garbles when the IMU is active later, you have a bus-contention
or pull-up issue; tell me and we'll re-check the gating.

---

## Step 2 — Dial + buttons (UI)

**Do:** turn the dial in the distance screen; cycle the increment with the dial button;
press SET to move to the time screen; press the START button.
**See:** the distance value changes by the current increment; the increment cycles
(1 / 0.5 / 0.1 / 0.01 m); SET advances to time entry; START is detected (serial logs it).
**Sets:** confirms `DIR_SIGN` (dial direction) and button wiring in `ui.cpp`.
**If wrong:**
- Dial counts the wrong way → flip `DIR_SIGN` in `ui.cpp` (`+1`↔`−1`).
- Dial double-counts or skips → it's a debounce/contact issue; a small cap across
  CLK/GND helps, or tell me and I'll widen the debounce.
- A button reads inverted/stuck → it must be wired **to GND** (active-low); the
  firmware supplies the pull-up.

---

## Step 3 — IMU heading sign

**Do:** with the car armed (or add a temporary serial print of `poseGet().theta`),
rotate the whole car **counter-clockwise (to the left)** by hand.
**See:** heading should **increase**.
**Sets:** `HEADING_SIGN` in `pose.cpp`.
**If wrong:** if it decreases when you turn left, flip `HEADING_SIGN` (`+1.0`↔`−1.0`).
Everything steering-related depends on this being right, so don't skip it.

---

## Step 4 — Encoder distance (`M_PER_COUNT`) ← your distance accuracy

**Do:** mark a straight **5.000 m** on the floor. Roll the car (by hand is fine) from
the start mark until the MP passes the end mark, reading the raw encoder count. Easiest
method: temporarily set `M_PER_COUNT = 1.0f`, rebuild, and read `odoDistanceM()` — that
number *is* the raw count. Repeat 5×.
**Compute:** `M_PER_COUNT = 5.000 / average_counts`. Put it in `config.h`, rebuild.
(Your kit's ballpark is ~0.000191 m/count; the free roller may differ — trust your
measurement.)
**See:** after setting it, roll a known distance and confirm `odoDistanceM()` matches.
**Sets:** `M_PER_COUNT`. Also confirm forward motion makes the count **increase**.
**If wrong:**
- Count **decreases** when rolling forward → swap encoder A/B, or negate in `odo.cpp`.
- The 5 repeats scatter a lot → the belt/roller is slipping or bouncing (mechanical);
  fix that first — the spread is literally your best-case distance precision.

> Until `M_PER_COUNT` is nonzero, distance reads 0 and the car will **never reach its
> stop condition** (it would cruise forever). This is the deliberate tripwire.

---

## Step 5 — Servo map (`SERVO_CENTER_US`, `SERVO_US_PER_RAD`)

**Do:**
1. Find center: nudge `SERVO_CENTER_US` (start 1500) until the wheels are **dead
   straight** (use the caliper). Write it into `config.h`.
2. Find scale: command a known steering angle and measure the actual wheel angle.
   E.g. command +0.3 rad via `steerSetAngle(0.3)` and measure the wheel deflection;
   `SERVO_US_PER_RAD ≈ (measured_us_delta) / (measured_rad)`. A couple of points is
   plenty.
**Sets:** `SERVO_CENTER_US`, `SERVO_US_PER_RAD`, and `WHEELBASE_L_M` (measure
front-axle to rear-axle in metres — it feeds the steering feedforward `atan(L·k)`).
**If wrong:** servo hits a mechanical limit before the commanded angle → add a linkage
ratio or reduce `DELTA_MAX_RAD`.

---

## Step 6 — Motor direction + PWM→speed table

**Do (still on blocks):**
1. Confirm `driveForward(30)` spins the wheels **forward**. If reversed, swap the
   motor `M+`/`M−` leads (or RPWM/LPWM).
2. Build the speed table: run the motor at PWM 30, 40, 50, 60, 70, 80, 100 % and
   measure steady **ground speed** (cm/s) at each — easiest on the floor over a fixed
   distance with a stopwatch, or from `odoDistanceM()` deltas. Put the curve into
   `pwmForSpeed()` in `speed.cpp` (replace the placeholder linear guess with
   interpolation over your measured points).
**Sets:** motor direction; the `pwmForSpeed()` table that lets the car hit the target
**time**.
**Note:** time is only **1 pt/s** — getting within ~0.3–0.5 s is plenty. Don't
over-tune this; the stop (step 8) is where the medals are.

---

## Step 7 — Steering sign (the critical bench test)

**Do (still on blocks or hand-held):** set a gentle arc (or straight, `h`→0), start a
"run," and physically push the car **off** the path to one side.
**See:** the servo must steer the wheels **back toward** the path. That's the whole
ballgame for a closed-loop steerer.
**Sets:** the feedback signs in `steer.cpp` (`e_ct`, `e_head`, and the sign of
`SERVO_US_PER_RAD`).
**If wrong:** if it steers **away** from the path (runs off harder), flip the sign of
the feedback term (negate `Kc`/`Kh`, or the `SERVO_US_PER_RAD` sign). A closed-loop
steerer with the wrong sign is unstable — verify this before any floor run.

---

## Step 8 — Straight-line hold + the stop (wheels down now)

**Do:** set `h`→0 (straight path) in the UI or `GATE_BULGE_H_M`. Enter a distance and
time, run it on the floor for ~5 m.
**See:** the car tracks straight (heading holds), cruises, then **creeps and stops**
right at the target distance.
**Tune:** the stop is the score. Adjust for a dead stop with no overshoot:
- `CREEP_PWM` (config) — low but **guaranteed to move**; if a creep tick stalls, raise it.
- `CREEP_ZONE_M` — how far out it switches from cruise to creep.
- `BRAKE_PULSE_MS` — the active brake pulse length.
**Measure:** run it ~10 times and look at the **spread**, not one hero run. Consistency
is what "better of 2 runs" rewards. Getting from ±5 cm to ±1 cm is ~8 points.
**If wrong:**
- Overshoots the target → longer `CREEP_ZONE_M`, lower `CREEP_PWM`, or a stronger brake.
- Stalls before the target → the **stall guard** now bumps the creep PWM up
  automatically (up to `CREEP_PWM_MAX`); if it still falls short, raise `CREEP_PWM`
  or `CREEP_PWM_MAX` in `config.h`.

---

## Step 9 — Arc hold + can gate

**Do:** set the real arc (`GATE_BULGE_H_M`, default 0.925 m). Run the full course.
**See:** the car bulges left, threads the can gate, and stops on target.
**Tune:** the steering gains `Kh`, `Kc` in `steer.cpp` — start small, raise until it
holds the arc without weaving. Then, only if the stop is already reliable, narrow the
can gap for more bonus.
**Measure:** lateral error at the arc midpoint (should sit inside your gate) and the
stopping spread over ~10 runs.

---

## Quick reference — every constant/sign and where it lives

| What | Where | Set by |
|---|---|---|
| `M_PER_COUNT` | `config.h` | Step 4 (roll 5.00 m) |
| `WHEELBASE_L_M` | `config.h` | Step 5 (measure) |
| `SERVO_CENTER_US`, `SERVO_US_PER_RAD` | `config.h` | Step 5 |
| `pwmForSpeed()` table | `speed.cpp` | Step 6 |
| `GATE_BULGE_H_M` | `config.h` | Step 9 / can strategy |
| `CREEP_PWM`, `CREEP_ZONE_M`, `BRAKE_PULSE_MS` | `config.h` | Step 8 |
| `Kh`, `Kc` steering gains | `steer.cpp` | Step 9 |
| `HEADING_SIGN` | `pose.cpp` | Step 3 |
| encoder direction | `odo.cpp` | Step 4 |
| `DIR_SIGN` (dial) | `ui.cpp` | Step 2 |
| steering feedback sign | `steer.cpp` | Step 7 |

When these are set and the spread is tight, you're competition-ready. Log what you
change — that log is gold at the tournament when you're re-tuning in the 8-minute
window.
