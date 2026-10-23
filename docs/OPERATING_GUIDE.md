# EV — Operating Guide (2027 bottle task)

How to use the firmware once it's flashed: what to calibrate, in what order, and how
to fire a run. For building and flashing, see `BUILD_AND_FLASH.md`. For the electrical
build, see `WIRING_GUIDE.md`. The run itself is specified in
`design/EV_Detailed_Design.md` and the rules in `rules/EV_Rules_2027.pdf`.

> ⚠️ **A freshly flashed build will not arm.** `M_PER_COUNT`, `L_P_M`,
> `ENC_FROM_MP_M` and `SERVO_US_PER_RAD` (`config.h` §1) ship at `0.0` on purpose.
> Until you measure them, SET at "EV ready" shows `NOT CALIBRATED` and names the
> missing one. **Part 1 is not optional.**

---

## What one run does

```
START (pencil) → start delay → forward on the centerline → slow through the bottle
→ push it past the Bottle Line → stop → reverse → rear MP stops on the Target
```

- **The start delay** makes the total equal the Target Time. The rules count a
  pre-motion pause as Run Time and allow it to exceed 3 s (§8.o.i).
- **The final stop is on net encoder distance**, so pushing the bottle a little too
  far moves the bottle (1 pt/cm) but not the final MP (2 pt/cm).
- **If the IMU drops out mid-run**, the car keeps going with the steering centred,
  using the encoder only. A stop mid-track would end the run (§8.p).

---

## The two ways into the bench menu

| You do… | You get |
|---|---|
| Power on **normally** | `EV ready` → SET runs, **hold SET 2 s** = bench menu |
| Power on **holding SET** | The bench menu straight away |

If the IMU fails at boot, the bench menu still works. Runs go ahead too, but with the
steering centred, after a `NO IMU` warning you confirm with SET. A run without
steering still scores, while no run at all scores 2500.

**Controls:** rotate the dial to change a value · press the dial to cycle the step
size · **SET** confirms · **START** (the pencil trigger) fires the run. Every screen is
mirrored to the serial monitor.

---

# Part 1 — Calibration (once, on the bench)

```
Bench menu
> 1 Encoder read     5 Input test
  2 Motor test       6 IMU heading
  3 Servo jog        7 Steer response
  4 PWM->speed       8 Exit to run
```

Open the serial monitor first; most output goes there. Do these **in order**.

## 1.1 Encoder: direction, `M_PER_COUNT`, `M_PER_COUNT_REV` ← sets the 2 pt/cm score

Menu **1 Encoder read**. Wheels on the ground. The LCD shows forward counts (`F`),
reverse counts (`R`) and the net.

1. Push the car **forward** a few cm. `F` must go **up**. If `R` moves instead,
   flip `ENCODER_SIGN` (`config.h` §3) or swap the green and white wires.
2. Mark a **5.000 m** line. Press **START** to zero. Push the car forward to the far
   mark, smoothly, with no skid. Serial prints `M_PER_COUNT=…`.
3. Press **START** again at the far mark, then **pull it back** 5.000 m to the start
   mark. Serial prints `M_PER_COUNT_REV=…`. Both values come from the *net* count since
   the last START, so overshooting a mark and easing back doesn't skew them.
4. Repeat 3–5 times and average each one.

**Sanity check: expect roughly 52,000 counts.** Around 26,000 means 2× decoding, and
around 13,000 means 1×. In either case, check the B-phase wiring first.

```c
constexpr float M_PER_COUNT     = 0.00009558f;
constexpr float M_PER_COUNT_REV = 0.00009561f;   // or 0 to use M_PER_COUNT
```

**Both scales set your score, and the forward one matters more.** The final MP is
forward counts × `M_PER_COUNT` minus reverse counts × `M_PER_COUNT_REV`. The forward
leg is the long one (6–12 m) and the reverse leg the short one (0.2–2.6 m). So a 0.2 %
error in `M_PER_COUNT` puts the final MP about 2 cm off, while the same error in
`M_PER_COUNT_REV` costs only a few mm. Average several rolls for each.

## 1.2 Car dimensions

No firmware involved. Tape measure, in metres, along the car's centerline:

| Constant | From → to |
|---|---|
| `L_P_M` | Pusher **contact face** (the deepest point of the mouth, where a seated bottle touches) → **rear MP tip** |
| `ENC_FROM_MP_M` | Free-roller **contact patch** → **rear MP tip** |
| `WHEELBASE_L_M` | Front axle centre → rear axle centre. **Only needed if you switch to pure pursuit** (§1.6) |

Also check the whole car, pusher and MP included, is **≤ 80.0 cm** long (§3.d) and
**≤ 35.0 cm** wide (§3.e).

## 1.3 Motor and driver

Menu **2 Motor test**. Do the **first** burst with the rear wheels off the ground, just
to see the motor spin the right way without the car driving off. The encoder is on the
free-rolling front wheel, so it reads 0 while the car is lifted. For the count and
stiction checks below, put the car **on the floor** with clear space ahead. Pick a
direction, dial ~30 %, START. Each 1.5 s burst reports the counts:

```
[MOT] FWD pwm= 30%  counts=+7240  dist=+0.692 m
```

- Forward drives **forward** with **positive** counts; reverse drives backward with
  negative counts. If not, swap motor M+/M−.
- **Find stiction:** walk the PWM down until `NO MOVEMENT` appears in each direction.
  Set `KICK_PWM` (`config.h` §2) comfortably above the higher of the two.

## 1.4 Servo centre, `SERVO_US_PER_RAD`, `STEER_DIR_SIGN`

Menu **3 Servo jog**. The dial moves the servo live, the dial button cycles the step,
and SET exits and re-centres.

> ⚠️ The linkage allows only **±7° of beam**. With the drag link attached, stop the
> moment it stiffens.

1. **Horn OFF the drag link:** jog to the true centre → `SERVO_CENTER_US`. Fit the horn,
   attach the link, and trim the rest with the turnbuckle.
2. **Scale:** from centre, jog a known offset (say +200 µs) and protractor the beam:
   `SERVO_US_PER_RAD = Δµs ÷ Δbeam_rad`. Expect ~3500. It ships at 0, and the car won't
   arm until it's set: a wrong value doesn't just steer weakly, it also throws off the
   position estimate, because the estimator assumes the wheels turned as commanded.
3. **Heading sign first:** menu **6 IMU heading**. Turn the car **left** by hand; the
   heading must **increase**. If not, flip `HEADING_SIGN`. Do this before step 4: the
   steer test can't tell a wrong heading sign from a wrong servo sign.
4. **Direction:** menu **7 Steer response**, **FWD** mode. The LCD names the law that
   config.h selects: `FWD STAN` or `FWD PP`. Turn the car **left** by hand.
   The wheels must steer **right** (back toward the line). If they steer left, flip
   `STEER_DIR_SIGN`.
5. Press the dial for **REV** mode and turn the car left again. The wheels must steer
   **left**. In reverse a left wheel turns the car right, back toward the aim. If FWD
   was right but REV is wrong, **don't flip a sign**. That points at the reverse
   steering or the rear geometry: `ENC_FROM_MP_M`, or `WHEELBASE_L_M` with pure
   pursuit. The LCD shows `REV AIM`, `REV PP` or `REV STAN`; the check is the same
   for all three.

## 1.5 Speed feedforward

Menu **4 PWM->speed**. Needs `M_PER_COUNT` done and ~3.5 m of clear floor per point.
Pick **Forward** or **Reverse**, dial a duty, START. It times exactly 2.00 m after a
1 m warm-up:

```
[CAL] FWD pwm= 30%  v=0.842 m/s   (2.00 m in 2.38 s)
```

Collect 5–6 points in each direction across 0.1–1.5 m/s. Fit a straight line,
`PWM = offset + slope × v`, and enter `SPEED_FF_OFFSET_PCT` and
`SPEED_FF_PCT_PER_MPS`. The PI speed loop corrects whatever the line misses, so it
doesn't need to be perfect. If 0.1–0.3 m/s lands in the dead zone, regear: see
`BOM.md` §Gearing.

## 1.6 Bottle and run tuning (floor tests)

**Choosing the steering laws** (`config.h` §4). Each leg has its own switch:

| Leg | Setting | Options |
|---|---|---|
| Forward | `STEER_FWD_LAW` | `SteerLaw::Stanley` (default): lines up quickly, two gains. `SteerLaw::PurePursuit`: aims at a point `PP_LOOKAHEAD_M` ahead; smoother, one knob. |
| Reverse | `STEER_REV_LAW` | `RevSteerLaw::AimAtTarget` (default): turns the car so its rear points at the Target; freezes the wheels for the last 15 cm. `RevSteerLaw::PurePursuit`: goal point `PP_LOOKAHEAD_M` **behind**; keeps steering right up to the stop. `RevSteerLaw::Stanley`: Stanley adapted for reverse, same gains as forward. |

To run one method end to end, set both switches the same:

| Run | `STEER_FWD_LAW` | `STEER_REV_LAW` |
|---|---|---|
| All Stanley | `SteerLaw::Stanley` | `RevSteerLaw::Stanley` |
| All pure pursuit | `SteerLaw::PurePursuit` | `RevSteerLaw::PurePursuit` |

A forward law can't just run backwards: in reverse a left wheel turns the car right.
Reverse Stanley is the forward law with two changes: it measures the sideways error
at the rear MP (the end that leads when backing up), and its heading term is flipped.
Pure pursuit on either leg needs `WHEELBASE_L_M`, and the lookahead (0.8 m to start,
shared by both legs) must be longer than the wheelbase; the build refuses otherwise.
Stanley and aim-at-target don't need the wheelbase. All of them land within about
1 cm in simulation, so try them on the floor and keep whichever gives the smaller
sideways error at the bottle (forward) and at the Target (reverse).

These follow `design/EV_Detailed_Design.md` §5, steps 4–10. The constants are all in
`config.h` §4/§5:

| Test | Sets |
|---|---|
| Straight-line hold forward, lateral error at 2/3/4 m | `STEER_KH`, `STEER_KC` (Stanley) or `PP_LOOKAHEAD_M` (pure pursuit) |
| Capture at 2/3/4 m, many bottle brands | `V_CAPTURE`, `CAPTURE_LEAD_M`, pusher width |
| Push at high speed without tipping | `V_PUSH_MAX`, `A_PUSH`, `A_RELEASE` |
| Bottle slide after the stop | `S_SLIDE_M`, `A_PAST_M` |
| Reverse stop scatter | `STEER_KR` (aim-at-target) or `PP_LOOKAHEAD_M` (pure pursuit), `V_REV_END`, `FINAL_BRAKE_PWM` |
| Motion time over many runs | `T_BIAS_S` (printed after each run) |

---

# Part 2 — Firing a run

## 2.1 Enter the targets

Short-press **SET** at `EV ready`, then six screens. Dial to set, SET to advance. Every
value starts from what the last run used.

| Screen | What | Range |
|---|---|---|
| `D:` | Target Distance | 5.00–9.00 m |
| `B:` | Bottle Line Distance past the Target | 1.00–3.00 m |
| `T:` | Target Time | 10.0–20.0 s (0.5 s steps) |
| `Bottle level` | Regional 2.0 m / State 3.0 m / National 4.0 m | menu |
| `MPofs:` | Final MP correction. **`0.000` on run 1** | ±0.50 m |
| `BtlOfs:` | Bottle push correction. **`0.000` on run 1** | ±0.50 m |

Press the dial to cycle the step size (0.25 regional → 0.10 state → 0.01 nationals).
The LCD then shows the forward stop, reverse length and start delay, and serial prints
the full plan:

```
[SM] PLAN D=7.00 B=2.00 T=15.0 bottle@2.0  mpOfs=+0.000 btlOfs=+0.000
[SM]   contact@1.417  slow@1.267  push@1.447  xF=8.500  xFinal=7.000  rev=1.500  path=10.00 m
[SM]   v cruise=1.60 push=1.06 rev=0.80 m/s  motion~15.00 s  delay=0.00 s
```

(This example uses `L_P_M = 0.55`.) The plan uses the nominal speeds when they fit.
Otherwise it speeds up only as much as needed to finish on T: cruise and reverse
first (up to their `_MAX`), and the push only if that isn't enough, because the push
carries the bottle. Capture speed never changes.

`T SHORT` on the LCD means the run can't finish inside T even at the maximum safe
speeds. It runs anyway. Being late costs only 0.5 pt/s, about 1 pt on average.

## 2.2 Arm

```
ARMED  SET=back
D7.00B2.00T15.0
```

Place the car with the **rear MP over the Start dot** (within ~1 cm) and the body on
the Target side of it (§4.d.vii.1), pointed along the centerline. Encoder, pose and
heading are zeroed **at the pencil press**, so adjusting the car while ARMED is fine.
SET backs out to `EV ready`. Short-press SET there to re-enter the targets (your
values are kept).

From the pencil press until the car moves, the motor is braked, so the car can't
roll off the Start dot during the start delay.

If the top line changes to `ARMED  NO IMU!`, the heading sensor isn't answering. The
run still works, but with the steering centred. Back out with SET and check the IMU
wiring if you have time.

## 2.3 Fire

**Press START with the #2 unsharpened pencil, vertical motion only.** Run Time starts
now. The LCD counts the delay down. Then the car drives, captures, pushes, turns around
(0.4 s) and reverses onto the Target. The LCD stays still while the car is moving;
that's deliberate, so a screen write never delays a stop.

**SET aborts at any point.** The car brakes and holds.

After every stop the car **stays braked, with the wheels where they ended**, until you
start the next run. That's on purpose: it may still be on the track waiting to be
measured, and releasing the drive or re-centring the steering could nudge the MP.

## 2.4 Read the result

The LCD shows the encoder's MP error and the run time, then the forward-stop error:

```
MP +0.4cm t15.03
F +1.2cm ok
```

Serial adds the planned and reached stops, the motion time (with a suggested
`T_BIAS_S`) and a timeline of every state change.

> ⚠️ **The encoder numbers are not your score.** The car stopped *because* the encoder
> hit its target. Only a tape measure tells you where the MP and the bottle really are.

## 2.5 Run 2 — use run 1 as calibration

The better of the two runs counts, so run 1 is a free measurement. Measure with a tape,
SET at the result screen, and adjust the two offsets (the targets carry over). Each
correction is a **change to the value already on the dial**, which starts at what the
last run used. Serial prints the new values for you after every run.

| Last run finished | Change the offset by | From 0 that is |
|---|---|---|
| Rear MP 3 cm **beyond** the Target (bottle side) | `MPofs` − 0.030 | `-0.030` |
| Rear MP 2 cm **short** (Start side) | `MPofs` + 0.020 | `+0.020` |
| Bottle's near side 2 cm short of where you want it | `BtlOfs` + 0.020 | `+0.020` |
| Bottle not fully past the line | `BtlOfs` + the shortfall + 0.020 | |

Example: run 1 used `MPofs -0.020` and the MP still ended 1 cm beyond the Target.
Enter `-0.030`, not `-0.010`.

Each offset moves only its own stop: `MPofs` never changes the push, and `BtlOfs` never
changes the final MP position.

---

## Quick reference

| Situation | Do this |
|---|---|
| Bench menu | Hold **SET** 2 s at `EV ready`, or hold SET at power-on |
| Abort a run | Press **SET** |
| `NOT CALIBRATED` | Measure the constant it names (`config.h` §1) |
| `PLAN ERROR` | It names the problem (e.g. `L_P > 80cm`) |
| `ABORT by SET` | You pressed SET |
| `ABORT wrong way` | Encoder sign flipped, or the car rolled backward at launch |
| `ABORT no motion` | No encoder count for 0.5 s while driving, even after a re-kick: the encoder is unplugged or dead, **or** the car is stuck (check both) |
| `ABORT stall` | Counting, but less than 1 cm of progress in 2 s: motor too weak, car stuck |
| `NO COUNTS` / `ask ES: moved?` | The motor drove but the encoder saw almost nothing, so the firmware can't tell whether the car moved. Ask the event supervisor whether it counts as a run, then check the encoder |
| `NO RUN (no move)` | Aborted before the car moved. Under the rules no run occurred (§4.d.vii.4.a): fix it and go again |
| `IMU!` after a run | Heading dropped mid-run; it finished encoder-only |
| Boot banner reappears mid-run | Brownout, not a firmware bug — see the servo rail cap |
| Dial counts backwards | Flip `DIAL_DIR_SIGN` (`config.h` §3) |

## Safety

- **Keep the wheels off the ground until `M_PER_COUNT` is calibrated.** The firmware
  refuses to arm without it, but the bench tools still drive.
- **SET is the kill switch.** Watchdogs also stop the car if it goes the wrong way,
  loses its encoder (0.5 s), stalls (2 s), or overruns its planned time. If it sticks
  for 0.15 s at a slow speed, it re-applies the stiction kick first.
- No AI tools during Event Time (§4.c.vii), and no programming tools between START and
  retrieving the car (§4.d.vii.5).
