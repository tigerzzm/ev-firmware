# EV — Detailed Design (Pico + BNO085, servo heading-hold) · 2027 revision

*Revised 2026-10-18 for the 2027 rules. The task changed from "arc through a can gate, stop the front MP at the target" to "capture a bottle → push it past a line → reverse until the **rear** MP is on the target." Electronics are unchanged from the Aug 26 bring-up. Track geometry, run plan, state machine, control laws, calibration and test plan are rewritten. **Firmware rewritten for 2027 on 2026-10-18** (`plan`, `steer_law`, `speed`, `sm`, `ui`, `odo` fwd/rev split, `drive` hold; see `docs/OPERATING_GUIDE.md`). Not yet run on hardware. Deviations from this doc: PUSH_END and CREEP_REV are folded into the PUSH/REVERSE speed profiles (constant-decel ramps), and the plan raises cruise/push/reverse speed only when the run would not fit inside T.*

Companions: `../rules/EV_Rules_2027.pdf` (official rules), `EV_Rules_Scoring_Model.md` (2027 rules & strategy), `EV_Active_Tracking_Proposal.md`, `EV_Steering_Linkage_Design.md`, `EV_Hardware_Build_Guide.md`, `../OPERATING_GUIDE.md`.

**Hardware facts (from the Aug 26 bench session — see git log `0dbef69`, `a131e60`):** the board is a **Pico 1 (RP2040)**, `PICO_BOARD "pico"`. **The pin map lives in `src/config.h` and `docs/WIRING_GUIDE.md`** — this doc doesn't repeat it. Motor leads are 16 AWG. Encoder is a 600 P/R open-collector unit (no level shifter needed). Wheels are 73.03 mm. Gearbox is 11.31:1. Motor is a 550 **35T** (since 2026-10-23; was 21T), so full duty is ~3.5 m/s.

---

## 0. Track geometry & run plan

**Frame:** x is forward along the centerline (Start = 0, Target = D); y is left of center. All positions below are the **rear MP's x** unless noted.

**Inputs announced after impound:** D (5–9 m), B (1–3 m past the Target), T (10–20 s). The bottle position X_b (2.0 / 3.0 / 4.0 m by level) is entered as a setting.

**Vehicle constants (measured):**
- `L_P` — distance along the car from the pusher's contact face back to the MP. Overall length ≤ 80 cm includes pusher + MP.
- `R_BOTTLE` — about 0.033 m (6.5 cm bottle).

**Tuning constants (calibrated):**
- `A_PAST` — aim the bottle's near side this far past the line (default 0.05 m).
- `S_SLIDE` — how far the bottle slides after release (≈ 0 if braking is gentle; see §6.4).

```
Capture:   pusher face meets bottle when  x_MP = X_b - R_BOTTLE - L_P     (X_b is the bottle centre, §7.c.ii.5)
Fwd stop:  X_F = D + B + A_PAST - S_SLIDE - L_P
Reverse:   R   = X_F - D = B + A_PAST - S_SLIDE - L_P        (always > 0: B >= 1.00, L_P < 0.80)
Path:      P   = D + 2R      (≈ 5.5 m … ~14.6 m over the full rule range)
```

**Key simplification: the final stop depends only on net displacement.**
The free-rolling front encoder counts in both directions. The MP's final x is therefore `net_counts × m_per_count`, whatever happened on the forward leg. A forward overshoot of 3 cm moves the **bottle** 3 cm (1 pt/cm) but does **not** move the final MP position (2 pt/cm). The reverse stop target is simply:

```
N_FINAL = D / M_PER_COUNT        (net signed count; refine with separate fwd/rev scales, §7)
```

**Path shape:** a **straight line on y = 0** for both legs. There is no can gate any more. Steering is now **heading/lateral hold**. Its value: lateral error at capture (narrow pusher) and lateral error of the MP at the Target (point-to-point distance).

Sideways tolerance at capture is about X_b · sin(heading error). At 4.0 m, 0.5° → 3.5 cm. A 9 cm mouth with a 6.5 cm bottle leaves only ±1.25 cm. Active heading hold is what makes a narrow, high-bonus pusher safe.

---

## 1. System overview (electronics unchanged)

```
 8x AA NiMH (~9.6 V) ──16 AWG──> BTS7960 ──> 550 35T motor ──> 11.31:1 gearbox ──> rear wheels   (fwd + REV)
        └──> 5 V buck ──> servo, BTS7960 logic, Pico VSYS, encoder supply
 Pico (RP2040): core1 = pose estimator 500 Hz · core0 = state machine
   I2C bus A: BNO085 (Game Rotation Vector)      I2C bus B: 16x2 LCD
   PIO: front free-roller encoder A/B (signed, both directions)
   PWM: BTS7960 RPWM/LPWM (+ split enables) · servo → drag link (≈1:5.6) → front pivot
   GPIO: dial, START (pencil), SET
 Mechanical (new for 2027): rigid front Bottle Pusher (frontmost) · rigid rear MP (rearmost)
```

Pins: `src/config.h` §1 and `docs/WIRING_GUIDE.md`. Wiring rules learned on the bench still apply: never route motor current through a breadboard; never put a regulator in the motor path.

**Battery reminder (2027):** no coin cell may power anything (an RTC or backup cell on a breakout counts). Everything runs from the AA pack.

### Gearing matters more in 2027
**Update 2026-10-23:** the motor is now a 35T on the stock 13T pinion (11.31:1), so normal speeds sit at ~10–46 % duty; the slow segments are still in the dead zone and rely on the speed loop and stall kick (a 9T pinion would help). See `docs/BOM.md` §Gearing. Original note: `docs/BOM.md` flagged that 11.31:1 with the old 21T put the 2026 speed envelope at ~7–20 % PWM, in the motor's dead zone, and recommended 26–44:1 (e.g. 35T motor + 9T pinion). The 2027 run adds **slow capture, slow final push and a slow reverse creep**, all at the bottom of the speed range, in both directions. Regearing moves from "nice to have" to "do before calibration." After regearing, check the opposite end too: the worst case (D = 9, B = 3, T = 10 s) needs a path of ~14.6 m in under 10 s, an average of ~1.5 m/s.

---

## 2. Firmware modules (changes marked ▲)

- `imu` — BNO085 Game Rotation Vector → `heading()` zeroed at ARM. Unchanged.
- `odo` ▲ — **signed** count (`int32_t`) from PIO. `distance_m()` = net signed distance. Optional separate `M_PER_COUNT_FWD/REV`.
- `pose` — integrate (x, y, θ) with **signed** `ds` (`x += ds·cosθ` works unchanged in reverse).
- `plan` ▲ — replaces the arc `path`. From D, B, T, X_b → X_F, R, N_FWD, N_FINAL, capture window, motion-time estimate, start delay.
- `steer` ▲ — straight-line hold, forward and reverse laws (§4.2).
- `speed` ▲ — segment speeds (cruise / capture / push / reverse / creep) and gentle accel limits. No longer responsible for hitting T.
- `drive` ▲ — `forward(pwm)`, `reverse(pwm)`, `brake()`, `coast()`.
- `ui` ▲ — enter **D, B, T** (plus level / X_b preset); keep the existing stop-offset entry (run 1 calibrates run 2); post-run readout of net distance, forward-stop error and run time.
- `sm` ▲ — the new state machine (§3).
- `cal` — existing bench menu; add a reverse-creep probe, a bottle push test, and a turnaround timer.
- `config.h` — every new constant (`L_P`, `A_PAST`, `S_SLIDE`, `X_B_*`, `V_CAPTURE`, `TURN_MS`, …) goes here, per the repo convention.

Control tick: 500 Hz. The servo actuates at 50 Hz (20 ms frame).

---

## 3. State machine

```
BOOT → SETUP
SETUP
  enter D, B, T; level preset → X_b
  plan(): X_F, R, N_FWD, N_FINAL, capture window, t_motion_est, t_delay = max(0, T - t_motion_est - T_BIAS)
  check: P fits inside T at max safe speeds — else warn on LCD
  SET → ARMED
ARMED
  zero encoder, pose = (0,0,0), IMU heading ref; servo centred; motor coast
  START (pencil) → DELAY            // Run Time starts NOW
DELAY                               // legal: pre-motion pause counts toward Run Time, may exceed 3 s
  hold still (motor coast, servo centred) for t_delay → CRUISE
CRUISE        fwd, heading hold on y=0, ramp to V_CRUISE (accel ≤ A_MAX)
  until x_MP ≥ capture_start (≈ contact − CAPTURE_LEAD, e.g. 0.15 m) → CAPTURE
CAPTURE       fwd at V_CAPTURE (slow; bottle must not be kicked ahead)
  until x_MP ≥ contact + SEAT_MARGIN → PUSH
PUSH          fwd, heading hold, ramp gently to V_PUSH (accel ≤ A_PUSH_MAX: anti-tip)
  until x_MP ≥ X_F − decel_dist(V_PUSH) → PUSH_END
PUSH_END      closed-loop decel to stop at count N_FWD, decel ≤ A_RELEASE (< μg, so the bottle stays seated, S_SLIDE ≈ 0)
  at count ≥ N_FWD → TURNAROUND
TURNAROUND    brake; dwell TURN_MS (≈ 300–500 ms); hard cap so total stationary time < 2.0 s
  → REVERSE
REVERSE       reverse at V_REV, reverse steering law aiming the MP at (D, 0)
  until x_MP ≤ D + CREEP_ZONE → CREEP_REV
CREEP_REV     reverse at PWM_CREEP (stall guard: bump PWM if < MIN_COUNTS per window)
  until net_count ≤ N_FINAL → BRAKE
BRAKE         short opposite pulse (forward) ~80–120 ms, then hold → DONE
DONE          LCD: final net distance, X_F error, elapsed time; store for run-2 offset → SETUP
```

**Rules-driven guards**
- **No mid-run pause ≥ 3 s.** Any state that can sit still (TURNAROUND, a stalled CREEP) needs a timer well under 3 s. Otherwise the run ends with the MP in the wrong place.
- **IMU loss → degrade, don't stop.** The existing "IMU-loss abort" would stop the car mid-track. Under 2027 rules that is a stopped run, likely with the bottle short. Change it to: centre the servo and continue on encoder only.
- **Runaway watchdog** — keep it, but its action is "brake and stop," which now also ends the run. Set its limits from the 2027 path length (up to ~14.6 m), not the old arc length.
- **Never move backward at START.** Moving the wrong way from the Start is a Failed Run. Require `forward()` as the first motion command.

---

## 4. Control laws

### 4.1 Pose
Unchanged, but with signed `ds`. Track the MP: `x_MP = x_ref − (offset from pose reference to MP)·cosθ`, or simply use net encoder distance on these straight legs.

### 4.2 Steering
**Forward (straight line, y = 0) — Stanley-style:**
```c
e_head = wrap(0.0 - theta);
e_ct   = y;                                   // + = left of line
delta  = Kh*e_head - atan2(Kc*e_ct, v + EPS); // signs finalised on bench
delta  = clamp(delta, -DELTA_MAX, DELTA_MAX); // DELTA_MAX ≈ 0.12 rad (mechanical limit)
```
**Reverse (front axle trails; θ̇ = v·tanδ/L with v < 0, so the response sign flips):**
```c
theta_des = atan2(y_mp, x_mp - D);   // forward-heading that points the rear MP at the target (≈0 if on line)
delta     = -Kr * wrap(theta_des - theta);
```
Keep `Kr` low; the reverse leg is short (≈0.2–2.6 m) and slow. The real defence is ending the forward leg on the line. If reverse steering misbehaves on the bench, **centre the servo in reverse** and rely on forward accuracy.

### 4.3 Speed and time
- Time is worth only 0.5 pt/s, and a start delay absorbs any slack. Pick segment speeds for **accuracy and bottle safety**, not for T.
- `t_motion_est` comes from a calibration fit over (D, B). Then `t_delay = T − t_motion_est − T_BIAS`.
- Only if `P` cannot fit inside T at safe speeds (worst case: D = 9, B = 3, T = 10 s) do you raise V_CRUISE. Warn on the LCD in that case.

### 4.4 Bottle physics → firmware constants
- **Capture speed:** a heavy car hitting a light bottle can launch it at up to ~2× the car's speed. Keep V_CAPTURE low so the bottle seats instead of bouncing.
- **No tipping:** steady push tips the bottle if the contact height h > r/μ (≈ 8 cm at μ = 0.4). Accelerating lowers that threshold. Put the pusher contact at **2–4 cm** and limit A_PUSH_MAX.
- **Release:** if the car decelerates more slowly than μg (~3–4 m/s²), the bottle stays pressed against the pusher and stops with it, so S_SLIDE ≈ 0 and its rest position is repeatable. Hard braking instead gives a slide of v²/2μg.
- **No pulling on reverse:** the pusher geometry must release the bottle the instant the car backs away (§7).

---

## 5. Calibration procedure (bench, in order)

1. **M_PER_COUNT, forward and reverse separately.** Roll 5.000 m forward and back by hand, 5× each. If they differ, keep both constants.
2. **L_P, overall length, width, pusher opening W.** Measure exactly. L_P enters every plan.
3. **Servo map and steering signs** (unchanged procedure). DELTA_MAX ≈ 0.12 rad.
4. **Straight-line hold, forward.** Lateral error at 2, 3, 4 m and at 9 m.
5. **Bottle μ and slide.** Measure on a competition-like floor with bottles at 80 g and 100 g. Confirm S_SLIDE ≈ 0 at your A_RELEASE.
6. **Capture.** At X_b = 2/3/4 m and several V_CAPTURE values, record capture success and bottle seating. Find the narrowest W that is ~100 % reliable.
7. **Forward stop → bottle position.** Command several X_F values and fit bottle-near-side error vs command. Set A_PAST from the scatter.
8. **Reverse stop.** From known forward stops, measure final MP error vs N_FINAL. Fit separately from forward (weight transfer and drivetrain reversal differ).
9. **Turnaround.** Measure the actual stationary time; confirm it is ≪ 3 s.
10. **Motion time.** Record motion time over a grid of (D, B) and fit `t_motion_est`.

Build a lookup/fit so entering D, B, T on the day is a lookup, not a calculation.

---

## 6. Test plan

- **Static:** encoder counts cleanly in **both** directions with no wrap; IMU zeroing; servo; motor forward/reverse/brake.
- **Capture reliability:** 20+ runs at each bottle position, with different bottle brands (ridged, smooth, square-shouldered) at 80 g and 100 g. Count misses, tips, and contacts outside the mouth.
- **Reverse precision:** 10 runs. Log mean and SD of the MP error.
- **Full runs:** log D, B, T, battery voltage, MP distance, bottle distance (fully past?), run time and computed Run Score. Judge by **mean and SD of the score**, not the best run.
- **Event rehearsal:** check-in → enter targets → align → pencil start → retrieve → adjust one thing → run 2, against an 8-minute clock.
- **Host harness:** extend `host_check/test_math.cpp` with the plan() geometry (X_F, R, N_FINAL) and the reverse steering sign. `bash host_check/run.sh` must stay green.

---

## 7. Mechanical additions for 2027

**Bottle Pusher (frontmost, rigid, one only, self-built):**
- Shallow V or concave cup that centres the bottle.
- Contact height 2–4 cm. Lowest edge must stay clear of the bottle base: nothing may go under it.
- All foremost edges rounded to ≥ ~7 mm (pencil thickness).
- No inward-curling lips, so it never drags the bottle on reverse.
- Start wide (~15–20 cm). Narrow toward ~9–12 cm only as capture testing allows. Prototype in cardboard, final part in rigid material.
- Only the inside of the mouth may touch the bottle. Make sure wheels and body can never contact a bottle that slides sideways.

**Rear MP:** a stiff pointer (wire, dowel edge or wedge corner), ≤ 1.0 cm above the floor, and the rearmost point of the car. It must be visible from above for alignment over the Start dot.

**Length budget:** the whole car including pusher and MP must be ≤ 80.0 cm. A longer L_P shortens the reverse leg (less reverse error) and tracks straighter.

**Encoder:** the free-rolling front encoder is ideal for reversing. It measures the ground, not the motor, so drivetrain backlash at direction reversal does not corrupt the count.

---

## 8. What's reused, changed, removed

- **Reused:** Pico (RP2040), BNO085, BTS7960, 550 motor + gearbox (regear recommended), free-roller encoder (now signed, both directions), servo + drag link, LCD/dial/buttons, `cal` bench menu, pin map, power tree.
- **Changed:** the steering's job (can-gate arc → straight-line heading/lateral hold); the stop (front MP forward → rear MP in reverse, on net count); timing (speed table → start delay); UI (adds B); IMU-loss behaviour (abort → degrade).
- **Removed:** can-gate arc planner (`R, PHI, k, C`, `GATE_BULGE_H_M`), the can bonus, and the "encoder on steered axle over an arc" correction (moot at κ ≈ 0).
- **Added:** Bottle Pusher, rear MP, capture/push/turnaround/reverse states, Kit Modification Form.

*2027 baseline: straight-line two-leg run · bottle capture and push · rear-MP reverse stop on net encoder count · start-delay timing · RP2040 + BNO085 heading hold · MG90S on ≈1:5.6 drag link · free-roller encoder (signed).*
