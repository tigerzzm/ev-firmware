# EV Redesign — Session Log & Decision Record

*A running record of the conversation that produced the active-tracking redesign: what was asked, what we decided, why, and what was delivered. Read this to get back up to speed.*

Original design session: 2026-07-23 · Project: **EV for SO 2026**

---

## 2026-10-18 — 2027 rules released: the event changed (newest entry — read first)

**Finding.** The 2027 Electric Vehicle rules are out, and the task is substantially different. First drafted from the scio.ly study guide; **later the same day checked against the official manual, now at `docs/rules/EV_Rules_2027.pdf`.** The summary held up; the manual settled two open questions (bottle position is to its centre; kit 3D-printed parts are exempt) and added a few procedure rules (ready position, no programming tools during a run, no floor tape).
- The car must capture a **water bottle** on the centerline (at 2.0 / 3.0 / 4.0 m by level), push it **past a Bottle Line** 1–3 m beyond the Target, then **reverse** so the **rear** MP stops on the Target.
- D range is now **5–9 m**. **The can bonus is gone.**
- Scoring: 2.0 pt/cm vehicle + 1.0 pt/cm bottle + **0.5 pt/s** time. **−20** for a bottle fully past the line. **Pusher Bonus (W − 35)·1.5**. A lost or illegally moved bottle sets d_bottle = 400.
- A **start delay counts as Run Time and may exceed 3 s**. Any mid-run stop ≥ 3 s ends the run.
- Length ≤ 80 cm for the whole car including the pusher.
- Kits are legal with ≥ 2 documented modifications, a self-built pusher, and no custom PCBs.
- No AI during Event Time.

**Decisions / design impact**
1. **Path is now a straight line.** The arc-through-can-gate planner is removed. The servo steering stays, with a new job: straight-line heading/lateral hold for **narrow-pusher capture** (±1.25 cm slack at 4 m) and the rear MP's lateral error.
2. **The final stop is on net encoder count** (`N_FINAL = D / M_PER_COUNT`). Because the free roller counts both ways, forward overshoot moves the bottle but not the final MP.
3. **Timing by start delay.** The speed table is demoted to "make sure the motion fits inside T."
4. **Gentle release** (decel < μg) keeps the bottle seated, so slide ≈ 0. Low contact (2–4 cm) and a slow capture prevent tipping and kicking.
5. **IMU-loss abort → degrade to encoder-only.** A mid-track stop is now very costly.
6. **Regearing (BOM.md) becomes a priority:** slow capture, push and reverse creep all sit in the 11.31:1 dead zone.
7. **New mechanical items:** rigid frontmost pusher (one only, self-built) and a rigid rearmost MP.

**Effect on the design docs**
- `EV_Rules_Scoring_Model.md` — **rewritten** for 2027.
- `EV_Detailed_Design.md` — **rewritten**: run geometry, state machine, control laws, calibration, test plan, mechanical additions. Electronics unchanged (RP2040, pin map in `src/config.h`).
- `EV_Active_Tracking_Proposal.md` — sensor stack and servo-steered architecture still stand. Its **can-gate / arc rationale is superseded.**
- `EV_Steering_Linkage_Design.md` — the drag link still wins (with a ~0° working angle, lash matters even more). **§2's 1–4° arc-angle numbers and §10's arc track-offset bug are moot** at κ ≈ 0. DELTA_MAX ≈ 0.12 rad and the servo-calibration notes still apply.
- `EV_Design_Review.md` / `EV_How_It_Works.md` — describe the original 2026 kit code; historical.

**Open questions for the ES / clarifications:** whether the Pusher Bonus applies on a failed-bottle or Failed Run; whether the Unphayzed kit includes a custom PCB.

**Next steps**
- Update firmware for 2027: `plan` (replaces `path`), `sm`, `steer` (reverse law), signed `odo`, `ui` entry of B, guards.
- Design and prototype the pusher (cardboard first). Add the rear MP. Check the 80 cm length budget.
- Fill in the Kit Modification Form. The Pico/IMU/servo retrofit covers the ≥ 2 functional modifications.

---

## 2026-07-23 — Original redesign session

### Goal of the session
Deep-research the Science Olympiad **Electric Vehicle** event for the **2026–2027 school year (= 2027 season)**, fully understand the rules, and modify the current design. User priorities: **stopping accuracy, timing accuracy, run-to-run consistency, and lower implementation complexity.**

---

## Key findings & decisions (in order)

1. **Season / rules status.** "2026–2027 school year" = the **2027 season**. EV is on the official 2027 Division C list, but the **2027 rules manual isn't released yet** (as of July 2026). All work is based on the **2026 rules**. → **Re-verify against the 2027 manual when it posts.**

2. **Scoring model (drives every decision).** `Run Score = 100 + Distance(2 pt/cm) + Time(1 pt/s) + Bonuses + Penalties`, **lower wins, better of 2 runs.** Takeaways: **distance accuracy dominates**; **consistency > a single hero run**; the vehicle must be **re-tunable after impound**. See `EV_Rules_Scoring_Model.md`.

3. **Current build understood.** Unphayzed *Electric Champion Kit*: single **550 motor → 2WD gearbox → rear axle**; **free-rolling front axle** belt-drives an encoder (distance); **front pivot + adjustment arm** set a fixed steering angle, dialed in with a **digital caliper** (a measuring gauge, *not* a brake); braking is a motor reverse pulse. Arduino UNO + BTS7960 + LCD + dial. See `EV_How_It_Works.md`.

4. **Design review.** Root cause of the accuracy/consistency problems: the final approach is **open-loop** — it fires a pre-computed pulse count without re-checking the encoder, so battery sag/friction move the stopping point every run (and run 2 degrades). See `EV_Design_Review.md`.

5. **Redesign direction (user's call): active pose tracking.** Closed-loop **heading + encoder localization**, actively steering to a planned path. See `EV_Active_Tracking_Proposal.md`.

6. **Architecture decision: servo-steered front pivot (retrofit), NOT differential drive.** Differential drive needs two motors; the car already has a steerable front pivot — motorize it. Keeps the compact single-motor drivetrain, BTS7960, and free-roller encoder.

7. **Hardware locked: Raspberry Pi Pico 2 (RP2350) + Adafruit BNO085.** Use the **Game Rotation Vector** (mag-free) for heading. Pico 2: hardware FPU + PIO quadrature; 3.3 V, not 5 V-tolerant.

8. **Servo sizing.** ~0.3–0.8 kg·cm scrub torque; an MG90S-class metal-gear servo (~2.0–2.5 kg·cm) has 3–5× margin.

9. **Track geometry / can-bonus.** Cans on the Bonus Line halfway to target, out to the **left**; must pass **between** them; tighter gap = more bonus. ⇒ planned path is a **constant-curvature arc** bulging ~90–92.5 cm left.

---

## Follow-up work (later sessions)

- **Firmware implemented** (C/C++ pico-sdk, not the earlier Arduino sketch): harvested the proven low-level modules from the **robotour-pico** project (BNO08x driver, PIO encoder, motor PID/feedforward, dual-core estimator, utils, RAM logger) and built the EV control on top. All modules compile-clean under `-Wall -Wextra` in a host harness (`host_check/`), math unit-tested; cross-core I2C contention fixed; APPROACH stall-guard + IMU-loss abort implemented. Ported the LCD + dial UI from the existing kit. See `HARVEST_NOTES.md`.
- **Hardware docs** produced under `docs/`: START_HERE, BOM, WIRING_GUIDE (+ wiring_diagram.html), VEHICLE_BUILD, BUILD_AND_FLASH, BRINGUP_AND_TUNING.
- **CAD reviewed** — rendered the real STL parts; the current steering is a **center-pivot beam axle** set by a caliper-locked arm. Steering-slack analysis + the direct-coupling recommendation are in `docs/hardware/CAD_SUMMARY.md`.
- **Decision: drop the digital caliper.** With the servo + IMU closed loop, the caliper (and its mounts + the adjustment-arm hook) are no longer needed — the encoder+IMU measure the actual result instead of the steering input.
- **Repo:** pushed to GitHub `tigerzzm/ev-firmware`; the working clone is at `~/Documents/EV_2026/ev-firmware`.

---

## Open items / next steps

- **Bench bring-up + calibration** — every SIGN and CONSTANT (order in `EV_Detailed_Design.md` §7 and `docs/BRINGUP_AND_TUNING.md`). Firmware has never run on hardware.
- **Steering coupling decision** — confirm which feature is the true pivot axis, then finalize direct-coaxial vs arm-driven servo mount (see `docs/hardware/CAD_SUMMARY.md`).
- **Re-check everything against the 2027 rules manual** when Science Olympiad releases it.

---

*Sources: SO 2026 Div C Rules Manual (Electric Vehicle), official EV Track Diagram (2025), soinc.org EV event page/FAQ/checklist, the user's own CAD + code files.*
