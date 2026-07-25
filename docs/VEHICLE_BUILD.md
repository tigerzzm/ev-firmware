# EV — Vehicle Build (Mechanical)

How to turn your existing Unphayzed kit into the active-tracking car. The plan
(from `EV_Active_Tracking_Proposal.md` / `EV_Detailed_Design.md`) is a **retrofit**,
not a rebuild: you keep almost all the mechanics and add a steering servo + the Pico
electronics.

## What you keep vs. change

**Keep:** the chassis, the 550 motor + 2WD gearbox + rear drive, the single BTS7960,
the belt-driven **front free-rolling encoder**, and the LCD/dial/button UI.

**Add:** the Pico 2, the BNO085 IMU, one **MG90S-class metal-gear servo** on the
existing front pivot, a 5 V buck regulator, and (if the encoder is 5 V) a level
shifter.

**Remove:** the manual adjustment arm *as the steering setter*. (Keep the digital
caliper — it's still handy for setting the servo's straight-ahead center.)

## Rules envelope (check as you build)

- **Wheelbase** (front of front wheel → back of back wheel) ≤ **70.0 cm**.
- **Width** at any point ≤ **35.0 cm**.
- A defined **Measurement Point (MP)** at the **front** of the vehicle, **≤ 1.0 cm
  above the track** — this is the point distance is scored from. Make it a clean,
  repeatable front edge/tip.
- Everything moves as one unit — no tethers, no remote. Start is a **vertical push of
  a #2 unsharpened pencil** on the START button. Nothing else may trigger it.

---

## Step 1 — Motorize the front pivot (the core change)

The car already has a pivoting front axle held at a fixed angle by the adjustment
arm. You're replacing "fixed by hand" with "commanded by a servo."

1. Remove the manual adjustment arm from the pivot (keep the hardware).
2. Mount the **MG90S servo** rigidly to the chassis next to the pivot. A servo bracket
   or a printed/cut plate screwed to the frame is fine — it must not flex.
3. **Direct-couple** the servo horn to the pivot with the shortest, stiffest link you
   can (ideally the horn drives the pivot arm directly). **Backlash is the enemy** —
   any slop here shows up as steering wander. A metal horn and a tight pivot bearing
   help; the IMU feedback loop absorbs what's left.
4. Rough-center: with the servo at its center position (the firmware sends
   `SERVO_CENTER_US`, default 1500 µs), the front wheels should point **dead straight**.
   You'll fine-tune `SERVO_CENTER_US` and `SERVO_US_PER_RAD` on the bench
   (`BRINGUP_AND_TUNING.md` step 5) — use the caliper to check wheel angle.
5. Check steering **range**: command the servo to its limits and confirm the linkage
   doesn't bind or over-travel the pivot. Event paths are gentle arcs, so you don't
   need much throw.

**Torque sanity check (5 min, do it):** hook a small fish/luggage scale to the
steering arm a known distance from the pivot, on the real floor at final vehicle
weight, and pull until the wheels turn. Torque = force × arm-length. If it's under
~half the servo's rated stall torque, you're safe. If it's tight, add a linkage ratio
(short arm on the servo, longer on the pivot) — you have servo speed to spare. Steer
only while rolling (the firmware pre-aims before the start), which needs far less
torque than steering in place.

---

## Step 2 — Weight distribution

- Bias mass **rearward** (motor, gearbox, and the 8-AA pack in back). This loads the
  driven rear wheels for traction **and** lightens the front so the steering servo
  has an easy job.
- More total mass doesn't hurt steering, but it **does** make stopping harder
  (momentum). Since the score is dominated by where you **stop**, don't add weight for
  its own sake. There's no mass limit in the rules — pick weight for traction vs.
  stopping, not for steering.
- Keep the center of mass low and centered left-right so the car tracks straight.

---

## Step 3 — The free-rolling front encoder (distance)

This is your most important sensor — it measures true ground distance and triggers
the stop.

- It's already belt-driven off the **undriven front axle**. That's exactly right: an
  undriven wheel doesn't slip under acceleration/braking the way a driven wheel does,
  so its count tracks real ground travel.
- Verify the belt is snug (no skipping) and the roller has firm, consistent contact
  with the axle/wheel — any slip or bounce directly becomes distance error.
- Note its **operating voltage** (see `WIRING_GUIDE.md` §6): if it's 5 V, its A/B
  lines must be level-shifted before reaching the Pico.
- After wiring, you'll calibrate `M_PER_COUNT` by rolling a marked 5.00 m
  (`BRINGUP_AND_TUNING.md` step 4). This single constant sets your distance accuracy.

---

## Step 4 — Mount the electronics

- Put the **Pico 2**, the **buck regulator**, and the **level shifter** on a small
  perfboard/plate, mounted where wires reach everything and the USB port is
  accessible (you'll flash and read serial through it).
- Mount the **BNO085 flat and rigid**, aligned with the vehicle axes (its heading is
  relative to how it's bolted down — a wobbly IMU is a wandering heading). Keep it a
  little away from the motor if you can, though Game Rotation Vector is mag-free so
  motor magnetic fields don't corrupt it.
- Keep the **LCD + dial** reachable from the top for entering distance/time at the
  table, and the **START button** positioned so the pencil can press it with a clean
  **vertical** motion.
- **Strain-relieve** every connector, especially the motor leads and the servo cable.
  A wire that pops off mid-run is a failed run (2500 points).

---

## Step 5 — The can-gate / arc strategy (where the bonus lives)

- The planned path is a constant-curvature **arc** that bulges to the left to thread
  between two cans. The bulge height `h` is set in `config.h` as `GATE_BULGE_H_M`
  (default **0.925 m**, matching your kit's proven arc height = 100 − (width+margin)/2).
- **Can Bonus = −0.5 × (110 − gap_cm)** — a narrower gap earns more, bounded by your
  car's width. Only chase a tighter gap once your **stop** is already reliable; a
  failed run wipes out any bonus.
- Verify the car threads the gate repeatably at the current `h` before narrowing it.
  Because steering is now closed-loop, you can change the path in **software** (adjust
  `h`) instead of re-bending a mechanical arm.

---

## Step 6 — Impound safety

- The firmware's ARMED state **will not move** until the START button is pressed, and
  it zeroes pose + captures the IMR heading reference at arm time. Good.
- Before impound, confirm: motor doesn't creep at rest, servo holds center, and a
  single pencil press is the only thing that launches it.
- Bring a charged and a spare AA set — run 2 tends to be on a weaker battery, and the
  closed-loop stop is specifically designed so run 2 doesn't drift short as voltage
  sags (unlike the old open-loop pulse scheme).

---

Once the mechanics are together and solid, move to `WIRING_GUIDE.md`.
