# Electric Vehicle — Design Review & Modification Plan

**Build reviewed:** Unphayzed *Electric Champion Kit 2025–2026* — Arduino UNO, BTS7960 motor driver, 1200-PPR wheel encoder, I2C LCD, rotary dial for entry, mechanical arc-steer for the can bonus, motor/reverse-pulse braking.
**Files reviewed:** `code/main_code.rtf`, `code/encoder_test.rtf`, `CAD/` parts, materials list.
**Goal (per Ming):** improve **stopping accuracy**, **timing accuracy**, **run-to-run consistency**, and **reduce implementation complexity**.

> Reminder from the scoring model: **Distance = 2 pt/cm, Time = 1 pt/sec, better of 2 runs kept.** So the ranked payoff is: a repeatable, precise **stop** >> timing precision. Fixes below are ordered by points-per-effort.

---

## How the current code drives (three phases)

1. **Cruise** — `motor.rotate(50, CW)` at a fixed 50% until `counter >= slowDownEncoderValue` (hard-coded **1 m before** the target arc length).
2. **Calibrate** — one 100 ms reverse brake, then **5 test pulses** (`rotate(10,CW)`, 200 ms on / 200 ms off) to measure average encoder counts per pulse.
3. **Snail** — compute `pulses = remainingDistance / countsPerPulse` **once**, then fire exactly that many fixed pulses, spacing them by `pulseDelay` to consume the leftover time budget.

---

## Critical findings (fix these first)

### 1. The final approach is OPEN-LOOP → this is your #1 accuracy *and* consistency problem
In the "snail" phase the pulse count is computed once and then executed blind — the `for` loop **never re-checks the encoder** against `targetEncoderValue`. Any per-pulse variation (battery sag, a dusty track, a slightly grippier tire) accumulates with zero correction, so you land in a different spot every run. It also explains why **run 2 is often worse than run 1**: the battery is weaker, each pulse travels less, and the fixed pulse count now falls short.

**Fix (biggest single win):** make the final approach *closed-loop*. Creep toward the target while continuously polling the encoder, and brake the instant `counter >= targetEncoderValue`. The car then stops at the **same encoder count every time**, independent of battery state or friction. Stopping error drops to roughly one creep-step of travel.

This also lets you **delete the entire calibration phase** — no more 5 test pulses, no `countsPerPulse` math. Simpler *and* more accurate at the same time.

### 2. A 10% "creep" pulse can fail to move the car → runaway / dead run
`motor.rotate(10, CW)` is near the motor's stall deadband. Under load or a low battery a pulse may produce **~0 encoder counts**, which makes `encoderChange ≈ 0` and then `pulses = remainingDist / encoderChange` **blows up** (divide-by-tiny → huge/instant pulse train). At best a wasted run, at worst it drives off the end.

**Fix:** never divide by a possibly-zero measured value; and if a creep step yields fewer than N counts, **step the power up** until the wheel actually turns. (Closing the loop per finding #1 removes the division entirely.)

### 3. Time is only controlled in the last meter → target time is unreliable, especially long times
The first *(distance − 1 m)* is driven at a **fixed 50% power**, so most of the run's duration is whatever the hardware happens to give. Only the final meter's pulse spacing absorbs time. Consequences:
- **Short target times (~10 s):** if the 50% cruise already used more than the target, `nonPulseTime` clamps to 0 — you physically can't go faster, so you run long.
- **Long target times (~20 s):** all the waiting is crammed into the last meter as many spaced pulses — workable but fragile.

**Fix:** set the **cruise speed from the target time** (via a small calibrated speed table: PWM → cm/s), so the bulk of the timing is handled during cruise, and reserve the closed-loop creep purely for the precise stop. This *decouples* distance from time — tuning one no longer disturbs the other.

> Strategic note: Time is only 1 pt/sec, so don't over-engineer this. Getting within ~0.3–0.5 s is plenty. The closed-loop stop (finding #1) is where the medals are.

---

## Secondary findings (reliability & correctness)

4. **Fixed 1 m deceleration zone** regardless of speed/target. Too short at high speed (overshoot into the creep), wasteful at low speed. Scale the slowdown distance with cruise speed.
5. **`pulseDelay = 1000*(nonPulseTime/(pulses-1))`** divides by `pulses-1` → divide-by-zero when only 0–1 pulses remain. Guard it.
6. **Wheel circumference is hard-coded** (`7.3025 cm`). Loaded tire compression makes the *effective* rolling circumference differ, giving a systematic distance bias. Calibrate it empirically: drive a known 5.00 m, then set `effectiveCircumference = measured_counts / 5.00 m`. Make it the single most important tuning constant.
7. **Dead `#include <Servo.h>` (twice), no servo used.** Remove it. On the UNO the Servo library, once a servo is attached, seizes Timer1 and kills PWM on pins 9 & 10 — which are your `LPWM`/`RPWM`. It's inert today because nothing calls `attach()`, but it's a loaded gun sitting next to your motor PWM pins. Delete it.
8. **`counter` is `unsigned long` but the ISR decrements it.** Any spurious reverse edge wraps it to ~4 billion. Use a `signed long` and, since the run is always forward, you can also ignore direction in the ISR for robustness.
9. **Everything is `delay()`-blocking.** Fine for this sequential task, but it caps timing granularity at the pulse size. A closed-loop creep (finding #1) removes the dependence on blocking pulse widths.

---

## Recommended target architecture (simpler AND better on all four goals)

Replace the **cruise → calibrate → open-loop snail** with a clean **two-phase closed-loop**:

```
Phase A — CRUISE (handles TIME)
    run at cruisePWM (looked up from targetTime & distance)
    until counter >= (targetCount - creepCount)

Phase B — CREEP-TO-TARGET (handles DISTANCE, closed-loop)
    run at a low, guaranteed-to-move creepPWM
    while counter < targetCount:   // <-- polls encoder every pass
        (if a step stalls, bump power)
    motor.stop(); brief reverse brake; motor.stop();

Then: report finalDistance & runTime on LCD for run-2 offset.
```

Why this hits every goal:
- **Stopping accuracy:** the stop is triggered by the encoder count itself, not a pre-computed guess → sub-cm, and it's *self-correcting* against battery/friction drift.
- **Timing:** owned by cruise speed from a calibration table, decoupled from the stop.
- **Consistency:** identical stop condition every run; run 2 no longer degrades as the battery weakens.
- **Complexity:** deletes the calibration phase, the per-pulse averaging, and the fragile `pulses`/`pulseDelay` math — noticeably *less* code.

Keep the mechanical **arc-steer for the can bonus** and the `getArcLength()` chord→arc conversion — that part is sound and worth the points, as long as the stop is reliable.

*(Superseded by the active-tracking redesign: the arc is now driven by a servo + IMU closed loop instead of a fixed mechanical angle. See `EV_Active_Tracking_Proposal.md`.)*

---

## Two calibrations to run on the bench (do these regardless of code changes)

1. **Effective circumference:** drive a marked **5.00 m** straight, read counts, set the constant. Repeat 5×; the spread tells you your best-case distance precision.
2. **Speed table:** at PWM = 30/40/50/60/70/80/100%, measure steady cm/s over a fixed distance. This table is what lets you pick `cruisePWM` from a target time.

---

*Scoring basis: 2026 Div C Rules Manual (Electric Vehicle). Re-verify against the 2027 manual when released. See `EV_Rules_Scoring_Model.md` in this project.*
