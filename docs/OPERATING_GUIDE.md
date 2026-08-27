# EV — Operating Guide

How to use the firmware once it's flashed: what to calibrate, in what order, and how
to fire a run. For building/flashing see `BUILD_AND_FLASH.md`; for the electrical
build see `WIRING_GUIDE.md`.

> ⚠️ **A freshly flashed build cannot complete a run.** `M_PER_COUNT` and
> `WHEELBASE_L_M` ship at `0.0` on purpose. Until you calibrate them the car reads
> zero distance and never reaches its stop condition. **Part 1 is not optional.**

---

## The two boot modes

Everything hangs off one decision at power-on:

| Power on… | You get |
|---|---|
| **normally** | The run sequence — SETUP → ARMED → run |
| **holding SET** | The bench calibration menu |

Calibration mode runs *before* the IMU is initialised, so it works with the IMU
unwired. Normal boot does **not** — it halts with `IMU FAIL / check wiring`.

**Controls:** rotate the dial to change a value · press the dial to cycle the step
size · **SET** confirms · **START** (the pencil trigger) fires the run.

Every screen is also mirrored to the serial monitor, so the whole calibration
sequence can be driven with the buttons alone if the LCD isn't wired yet.

---

# Part 1 — Calibration (once, on the bench)

Hold **SET** while powering on:

```
Bench menu
> 1 Encoder read
  2 Motor test
  3 Servo jog
  4 PWM->speed
```

Open the serial monitor before you start — most output goes there, not to the LCD.
Do these **in order**; each one depends on the last.

## 1.1 Encoder direction and `M_PER_COUNT`  ← the most valuable number in the project

Menu **1 Encoder read**. Wheels on the ground.

1. Push the car **forward** a few cm. The count must go **UP**.
   *Counting down? Flip `ENCODER_SIGN` in `config.h` §3 (or swap the green and
   white wires — either works).*
2. Mark a **5.000 m** line on the floor. Line the car up.
3. Press **START** to zero.
4. Push the car by hand to the far mark — smoothly, no wheel skid.
5. Read the count. Serial also prints the constant for you:

```
[ENC] counts=+52310   M_PER_COUNT if this was 5.000 m = 0.00009558
```

**Sanity check: expect roughly 52,000 counts.** Around 26,000 means 2× decoding and
~13,000 means 1× — in both cases check the B-phase wiring before trusting anything.

Put the value in `src/config.h`, rebuild, reflash:

```c
constexpr float M_PER_COUNT = 0.00009558f;
```

Then come back and repeat step 4 — the readout should now show ≈ `5.0000 m`.

## 1.2 Wheelbase

No firmware involved. Tape-measure **front axle centre → rear axle centre**, in metres:

```c
constexpr float WHEELBASE_L_M = 0.xxx f;
```

While it's `0.0` the steering feedforward `atan(L·κ)` is identically zero and the
controller runs on feedback alone.

## 1.3 Motor and driver

Menu **2 Motor test**. **Wheels off the ground for the first run.**

Pick Forward, dial ~30%, press START. One 1.5 s burst reports:

```
[MOT] FWD pwm= 30%  counts=+7240  dist=+0.692 m
```

Check all three:

- The car drives **forward** — if not, swap the motor's M+/M− leads
- Counts are **positive** — should already be right from §1.1
- Reverse also works — `driveBrake()` uses that same half-bridge, so if reverse is
  dead, your stop is dead

**Then find the dead-zone floor.** Walk the PWM down — 25, 20, 15, 12, 10 — until:

```
[MOT] FWD pwm= 10%  counts=+0     dist=+0.000 m   <-- NO MOVEMENT
```

Set `CREEP_PWM` (`config.h` §2) a couple of points **above** that threshold. It
ships at `10.0f`, which is a guess.

## 1.4 Servo centre and `SERVO_US_PER_RAD`

Menu **3 Servo jog**. The dial moves the servo live; the dial button cycles the step
(10 / 1 / 100 µs); SET exits and re-centres.

> ⚠️ The linkage allows only **±7° of beam**. With the drag link attached, stop the
> moment it stiffens — holding against a mechanical stop stalls the servo.

**With the horn OFF the drag link:**

1. Jog to where the output shaft is truly centred → that's `SERVO_CENTER_US`
2. Fit the horn at the spline tooth closest to correct
3. Attach the drag link, take out the rest with the **turnbuckle** (the 20T spline
   indexes in 18° steps ≈ 3.2° of beam — you cannot land centre on a tooth)

**Then measure the scale.** From centre, jog a known offset (say +200 µs), measure the
beam angle with a protractor, and:

```
SERVO_US_PER_RAD = Δµs ÷ Δbeam_angle_in_radians
```

Expect roughly **3500**. It ships at `500.0f`, which is ~7× too small — that value
carries no knowledge of the 1:5.6 drag-link reduction. Measure near ±3° of beam; the
four-bar map is non-linear at the extremes.

Last, confirm the **steering sign**: push the car off the arc by hand and check the
servo steers it *back*, not further out. Flip `STEER_FB_SIGN` in `config.h` §3 if
wrong.

## 1.5 PWM → speed table

Menu **4 PWM->speed**. Needs ~3.5 m of clear floor per point and `M_PER_COUNT` done.

Dial a duty, press START. The car accelerates through 1 m (discarded — that's the
launch transient), then times exactly 2.00 m:

```
[CAL] pwm= 30%  v=0.842 m/s   (2.00 m in 2.38 s)
```

Collect 20/30/40/50/60/80/100%. `pwmForSpeed()` currently just multiplies by
`PWM_PER_MPS` (`config.h` §2, ships at `40.0f`) — a guess, **not** a measurement.
Send me the table and I'll replace it with interpolation over your real data.

Two things to note from the table:

- **The duty that gives ~0.7 m/s** is your mid-range cruise. `CREEP_PWM` must sit
  comfortably below it.
- The rules envelope needs **0.42–1.17 m/s**. If those land below ~20% duty, you're
  in the dead zone — see `BOM.md` §Gearing.

## 1.6 The remaining signs

All four signs live together in **`config.h` §3**, each with its bench test in the
comments:

- **`HEADING_SIGN`** — turn the car left by hand; heading should increase
- **`DIAL_DIR_SIGN`** — turn the dial clockwise; the value should go up
- (`ENCODER_SIGN` and `STEER_FB_SIGN` are covered in §1.1 and §1.4 above)

---

# Part 2 — Firing a run

Normal power-on (don't hold SET). Requires the IMU wired and working.

## 2.1 Enter the targets

Four screens, dial to set, **SET** to advance:

| Screen | What | Range |
|---|---|---|
| `Dist:` | Target Distance, announced after impound | 7.00–10.00 m |
| `Time:` | Target Time | 10.0–20.0 s |
| `Arc preset` | Gate bulge — automatic, just shows the value | — |
| `Ofs:` | **Stop correction. `0.000` on run 1.** | ±0.50 m |

Press the dial to cycle the step size — coarse to get close, fine for the last
digit. Distance goes down to 0.01 m for nationals.

Serial confirms the plan:

```
[SM] SETUP D=8.50 m T=15.0 s h=0.92 m -> R=9.68 L=8.766 k=0.1033
[SM] stopOffset=+0.000 m -> stopTarget=8.766 m; timeout 35.0 s
```

## 2.2 Arm

The LCD shows:

```
ARMED
waiting START
```

Pose is zeroed and heading reference captured **at this moment** — so the car must be
sitting in its final start position, pointed down the track, before you press START.
The servo pre-aims to the arc tangent.

## 2.3 Fire

**Press START with the #2 unsharpened pencil, vertical motion only.** That is the
rules-legal trigger; starting it by hand is a Competition violation (+150/run).

The car launches, follows the arc, creeps the last 20 cm, and brakes.

**SET aborts at any point during the run.**

## 2.4 Read the result

```
[SM] DONE  finalArc=8.771 m (target 8.766)  time=14.82 s (target 15.0)  abort=none
[SM] next run: measure MP->Target, enter negated as Ofs (now +0.000 m)
```

> ⚠️ **`finalArc` does not tell you your score.** It's the encoder reading, and the
> car stopped *because* the encoder hit the target — so it always lands near
> `stopTarget` no matter where the vehicle physically is.

## 2.5 Run 2 — use run 1 as calibration

The rules give you two runs and keep the better. Run 1 is a free measurement.

1. **Measure the physical error with a tape**: Measurement Point → Target Point
2. Power-cycle or let it loop back to SETUP
3. Re-enter the same D and T
4. At `Ofs:`, enter that error **negated**

| Run 1 finished | Enter |
|---|---|
| 3 cm **past** the target | `-0.030` |
| 3 cm **short** | `+0.030` |

The offset adjusts the stop distance **only** — it does not re-plan the arc, so your
line through the can gate is unchanged. One number absorbs every fixed error at once:
MP offset, encoder track offset, and brake overshoot.

The offset carries over between runs, so you nudge rather than re-entering from zero.

---

## Quick reference

| Situation | Do this |
|---|---|
| Enter calibration | Hold **SET** while powering on |
| Abort a run | Press **SET** |
| Zero the encoder readout | Press **START** |
| Car won't stop | `M_PER_COUNT` is still `0.0` |
| `IMU FAIL` at boot | IMU not wired — use calibration mode meanwhile |
| Boot banner reappears mid-run | Brownout, not a firmware bug — see the servo rail cap |
| Dial counts backwards | Flip `DIR_SIGN` in `ui.cpp` |
| Encoder counts backwards | Swap the green and white wires |
| Car drives backwards | Swap motor M+/M− |

## Safety

- **Wheels off the ground** until `M_PER_COUNT` is calibrated. Until then the run
  loop cannot exit on distance and only the watchdog stops the car
  (`2 × targetT + 5 s`).
- **SET is the kill switch.** Know where it is before you power up.
- Don't dry-steer with the wheels down until the servo rail has its capacitor —
  stationary scrub is the servo's peak-current moment.
