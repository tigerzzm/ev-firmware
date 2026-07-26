# CAD Summary — mechanical parts, dimensions, steering mechanism

Text summary of the vehicle's mechanical design (the STL files are binary — this is
the readable version). Source CAD: `~/Documents/EV_2026/CAD/` (Unphayzed *Electric
Champion Kit 2026*), plus the analysis done in-session. Companion images live beside
this file in `docs/hardware/` (rendered from the real STLs).

## Parts list (3D-printed, with real bounding-box dimensions)

| Part | Dims (mm) | Role |
|---|---|---|
| **Front Axle** | 90 × 108 × 62 | Rigid beam carrying **both** front wheels + the central pivot barrel + a long steering-lever arm. Steering = rotating this whole beam. |
| **Front Pivot** | 50 × 86 × 12 | Flat plate fixed to the chassis; provides the vertical pivot the beam turns on. |
| **Adjustment Arm Hook** | 10 × 20 × 6 | Tiny tab the caliper-set arm grabs to **lock** the beam at a fixed angle. |
| **Back Axle** | 90 × 75 × 38.5 | Rear (driven) axle. |
| **Caliper Arm / Front / Back Mount** | ~20–25 mm | Hold the digital caliper used to *measure & set* the fixed steering angle. |
| Arduino UNO Mount, Arduino Chassis Mount, BTS7960 Mount, LCD Mount, Scope Holder/Mount | — | Electronics + accessory mounts (UNO mount superseded by a Pico mount in the retrofit). |

The parts export in a **shared assembly coordinate frame** (combined front-end
footprint ≈ 180 × 170 × 62 mm), so loading them together reproduces the real layout.

## How the current steering works (center-pivot beam axle)

Both front wheels sit on **one rigid beam** (the Front Axle). The beam pivots about a
**single vertical axis** (the central barrel, riding on the Front Pivot plate). A long
(~75 mm) **steering-lever arm** projects off the beam; you position its end to a spot
**measured with the digital caliper**, then lock it via the Adjustment Arm Hook. Result:
the wheels hold one fixed angle and the car drives a fixed **arc** (this is how it
threads the can gate for the bonus). It's rigid and play-free *because* it's locked.

This is **not** Ackermann steering — it's a whole-axle center pivot.

## The retrofit (what changes)

- **Add** a metal-gear micro servo to command the beam angle live.
- **Remove** the digital caliper **and** its three mounts **and** the Adjustment Arm
  Hook — with the servo + IMU closed loop, the encoder+IMU measure the *actual result*,
  so nothing needs to be pre-measured/locked. (Fewer parts on the front end = less
  weight and fewer slack sources.) The caliper is at most an optional bench aid for
  finding the servo's dead-straight center (`SERVO_CENTER_US`).
- **Keep** everything else: beam, pivot, wheels, rear drive, BTS7960, front encoder.

## Steering slack — the key mechanical risk, and the fix

Today the beam is *locked*, so there's zero play. A servo introduces backlash from
five sources: (1) servo gear lash, (2) horn spline/screw, (3) linkage/pushrod ball
joints, (4) the central pivot bushing. Backlash matters because a dead-zone around the
commanded angle makes a closed-loop steerer **limit-cycle** (weave) instead of holding
the arc — which costs the can gate and adds stopping scatter.

**Recommended coupling: direct, coaxial with the pivot.** Mount the servo so its output
is **coaxial with the central pivot** → servo angle = beam angle 1:1, deleting all
linkage backlash (only servo gear lash remains). **Critical rule:** let the **pivot
bushing carry the load** and the servo carry **torque only** — do not hang the axle's
weight on the servo's output bearing (it will wear and develop play). Because 1:1
leaves servo gear-lash as the sole slack, **use a good low-backlash metal-gear/digital
servo** and **skip any servo saver** (it adds compliance). Torque is fine at 1:1
(~3–5× margin), and resolution/range are ample for gentle arcs.

*Alternative:* drive the **end of the ~75 mm lever arm** → built-in reduction (divides
servo backlash down at the wheel, more torque) but adds one linkage joint. Prefer this
only if stuck with a lashy servo.

**Also help it:** a light **preload** (spring/band) biasing the beam toward the arc
side keeps every joint loaded on one flank — and since the event path curves one
direction the whole run, the backlash essentially never flips through zero. The
firmware can add **backlash feed-forward** on any steering-direction reversal for
whatever remains.

**Open item:** confirm which feature is the *true* vertical pivot axis (the central
barrel vs. where the beam meets the Front Pivot plate) — that decides whether the
servo mounts coaxially or drives the arm. A photo of the assembled front end settles it.

## Rules envelope (check as built)

Wheelbase ≤ 70.0 cm · width ≤ 35.0 cm · a defined front Measurement Point ≤ 1.0 cm
above the track · starts only by a #2-pencil vertical push. See
`docs/design/EV_Rules_Scoring_Model.md`.

## Images in this folder

- `steering_current.html` — clean labeled top-view illustration of the current mechanism.
- `steering_assembly_iso.png`, `steering_assembly_top.png` — the real STLs rendered together (shaded).
- `part_front_pivot.png`, `part_front_axle.png`, `part_adjustment_arm_hook.png` — individual parts with dimensions.
