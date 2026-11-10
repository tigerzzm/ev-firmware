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

- **There is no start delay** (`START_SETTLE_S` = 0): the motor starts right after the
  press. The speeds are chosen so the driving fills the Target Time: slower when there
  is time to spare, faster when T is tight. Only if the car is at its slowest speeds
  and still early does it wait at the start.
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
> 1 Encoder read     6 IMU heading
  2 Motor test       7 Steer response
  3 Servo jog        8 Sim dry run
  4 PWM->speed       9 Exit to run
  5 Input test
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
   arm until it's set. Neither servo number has to be exact: the position estimate
   uses only the encoder and the IMU heading, never the servo command, so a wrong
   value just makes the steering a little softer or sharper. Within ~20 µs of centre
   and ±20% on the scale is fine (host sim: ~1 cm off the line).
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

Menu **4 PWM->speed**. Needs `M_PER_COUNT` done and ~5 m of clear floor per point.
Pick **Forward** or **Reverse**, dial a duty, START. It counts down 5 s (step back;
SET cancels), then times exactly 2.00 m after a 1 m warm-up, then eases down like a
run's push end (`A_RELEASE` to `V_PUSH_END`, about 1.6 m from 1.4 m/s) and stops. The
speed stays on the LCD until you press SET or START:

```
[CAL] FWD pwm= 30%  v=0.842 m/s   (2.00 m in 2.38 s)
```

Do it on a **charged pack**: a tired one needs ~10% more power for the same
speed. Collect points from 11% up to the top speed and enter each one as
`{speed, PWM}` in `SPEED_FF_TABLE`, speeds increasing; the car interpolates
between them. It's a table, not a line, because the speed flattens hard above
~1.2 m/s (top ~2 m/s). Forward and reverse share it. The PI speed loop corrects
whatever the table misses, so it doesn't need to be perfect. If 0.1–0.3 m/s lands in the dead zone, regear: see
`BOM.md` §Gearing.

**Catch test.** Because of the gentle ending, menu 4 also tests catching the bottle
at speed: stand the bottle about 1.3 m ahead of the pusher, in line with the mouth,
and film the hit from the side in slow motion. The bottle must stay upright, on the
floor and in the mouth (§3.l). Menu 4 does not steer, so a bottle that hits the
edge of the mouth means re-aim, not a failed catch.

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

## 1.7 Bench dry run (menu 8), wheels up

A whole run with the car on a stand: target entry, plan, ARMED, the start delay,
cruise, capture, push, turnaround, reverse, the stop and the report. With the
wheels up the encoder can't turn (it rides the undriven front wheels), so a normal
run would abort `no motion` half a second after launch. The dry run replaces the
encoder with a model of the car's speed driven by the motor command (`config.h`
§7, `SIM_*`). Everything else is the real run code.

1. **Lift the car so no wheel touches anything.** The motor really runs.
2. Menu **8 Sim dry run**. Enter the targets as for a real run (they carry over to
   the next run). The plan screen follows.
3. `SIM: WHEELS UP?` → **SET** to go on, hold SET to back out.
4. `SIM ARMED` → press **START**. Watch the delay count down, the motor drive
   forward, slow, brake for the turnaround, and drive in reverse to the stop.
   **SET** aborts at any time.
5. The result shows `SIM…` on the LCD and `SIM DRY RUN DONE` on serial with the
   event log. SET returns to the bench menu.

What it shows: the sequence, the motor directions, the timing against T, and the
event log. What it doesn't: anything about accuracy, since the distance is a
model. Its times are never used for `T_BIAS_S`. Turn the car by hand while it
"drives" and the steering corrects, as it would on the floor.

If the **real** encoder sees 2 cm of travel (`SIM_REAL_MOTION_M`), the car is on
the floor: it brakes and shows `ABORT on floor?`.

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
| `BtlPast:` | How far past the Bottle Line to aim the bottle's near side (the margin), **in cm**. Starts at `0.0`: aimed at the line, the bottle still ends a little past it (it carries on past the pusher) | 0.0–15.0 cm |
| `Aim:` | Aim trim, **degrees, + = left**: the car steers onto a line this far left of where it pointed at START. Corrects a fixed sight or thrust-line angle. Starts at `0.55`. After a run, add 0.115° per cm (at D 5 m; 0.573/D in general) the MP ended **right** of the Target, subtract per cm **left**; the report prints the number | −3.00 to +3.00° |

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
The motor starts right away. (`START_SETTLE_S` > 0 would make it stand braked that
long and zero the heading **again** before driving; logged runs showed the press
doesn't rock the car, so it is 0.) **Pull the pencil clear as soon as you press:**
the car leaves immediately.
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

Before that, serial prints the **odometry + steering record**: one row every 50 ms
of the motion (`TELEM_PERIOD_MS`) between `[TEL] BEGIN CSV` and `[TEL] END CSV`.
Copy it into a spreadsheet or plot it:

| Column | Meaning |
|---|---|
| `t_ms`, `state` | time since the pencil press; CRUISE / CAPTURE / PUSH / TURN / REVERSE / STOP |
| `x_mp_m`, `v_mps`, `vref_mps`, `pwm_pct` | encoder MP position, measured speed, speed target, motor duty |
| `roller_x_m`, `y_m`, `heading_deg` | the pose: where the car thinks it is, + y = left of the line |
| `steer_deg` | steering command, + = wheels left |
| `steer_ok`, `stuck`, `launch` | 0 = steering off (no usable heading), stall boost active, launch kick active |

Under it, a summary says whether the steering ran the whole time, how far off the
line the pose went, and suggests a `SERVO_CENTER_US` trim from the average steering
command on the steady cruise.

**Getting the logs after untethered runs.** The car keeps the **last 5 runs** in RAM
(`RUN_HISTORY`), each with its settings, record and event log, until the car is
switched off. So you can do several runs unplugged and read them together. At the
result screen (or `EV ready`), plug in USB — the Pico does not reset — and run,
from the repo:

```
python3 tools/capture_run.py --watch
```

then press **START** on the car. The firmware prints every stored run, oldest
first, and the script saves each as `run_logs/<time>-r<N>.log` (everything),
`.csv` (the record) and `.html` (the plots), where N is the run's number since
power-on. A run it already has is skipped, so pressing START again never makes
duplicates. With `--watch` it keeps running: start it once per session. The Pico
only sends while a program has the port open, so the script (or a serial
monitor) must be running at that moment. **Don't switch the car off** between
the runs and the read-out, and **write down the tape numbers per run** — the 6th
run overwrites the 1st.

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

`BtlPast` is where you **want** the bottle; `BtlOfs` corrects where it actually went.
After a run: new `BtlOfs` = old `BtlOfs` + (`BtlPast` − measured bottle past the line),
in metres (`BtlPast` 2.0 cm = 0.020 m).
Change `BtlPast` only to take more or less risk with the −20 bonus.

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
