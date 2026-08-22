# EV — Steering Linkage Design Record

*Session date: 2026-08-16. Supersedes the steering-mount guidance in
`docs/hardware/CAD_SUMMARY.md` (see §11). Geometry here is **measured from the STL
meshes**, not estimated from renders — earlier figures that disagreed were wrong.*

---

## 1. Measured geometry — ground truth

All values parsed directly from `CAD/3D Parts/EV-front-axle-v2.stl` (the user's v2
part). Coordinates are in that file's frame; **+Y is forward**, the lever arm points
toward −Y (rearward).

| Feature | Value |
|---|---|
| Pivot bore (vertical) | **(15.63, 36.01)**, Ø12.97, Z 0–4 |
| Lever end hole (vertical) | **(25.63, −52.52)**, Ø3.50, Z 8.5–20 |
| **r_lever** (pivot centre → lever hole centre) | **89.09 mm** |
| Lever offset from longitudinal | Δx = 10.00 mm → **6.4° tilt** |
| Wheel axle bores | Ø13.2, Y = 35.98, Z = 17, X = −24.37 and 55.63 |
| **Track** (axle bore centres) | **80.0 mm** |
| **Trail** (pivot Y − axle Y) | **+0.03 mm — i.e. exactly zero** |
| Beam body depth | Y 19.98 → 47.98 (28 mm) |
| Wall ahead of pivot bore | 5.48 mm |
| Encoder boss (lateral) | Ø40 + Ø20.5 step, 3 × Ø4 on 28.5 mm PCD, Y = −20.02, Z = 37 |
| Drag-link plane | Z ≈ 14 (set by the lever hole, Z 8.5–20) |

**Do not confuse r_lever with the visible arm.** The stalk sticking out behind the beam
measures ~72 mm, but the pivot sits ~20 mm further forward, *inside* the beam. The
mechanically relevant radius is 89.09 mm. `CAD_SUMMARY.md`'s "~75 mm" is the visible
arm; an earlier "92 mm" was an eyeball error off a render.

**The stalk is built off-axis.** Its midline is X = 25.63 while the pivot is X = 15.63,
and the hole is already centred in the stalk (8.25 mm edge distance both sides).
Aligning them would mean moving the whole stalk, not the hole. **Not worth doing** —
see §5.

**The encoder rides on the steering assembly.** Ø40 + 3 × M4 on a 28.5 mm PCD is the
standard mount for a Ø40 incremental encoder, so the 600 PPR unit appears to sit on the
rotating beam (which keeps the belt span constant as it steers — good design). *Confirm
this before finalising the servo mount; it occupies the space behind the axle.*

---

## 2. The number that drives everything

With `h = 0.925 m` and the rules' D range of 7–10 m, `path.cpp` gives R = 7.1–14.0 m,
so κ = 0.072–0.141 /m and the required steering angle is:

**δ = atan(L · κ) → 1–4° of beam angle**, depending on the (still unmeasured) wheelbase.

Every decision below follows from the working angle being that small.

---

## 3. Architecture: drag link, not coaxial

**Decision: keep the drag-link retrofit. This reverses `CAD_SUMMARY.md`, which
recommends coaxial direct-drive as primary.**

Reasoning: an MG90S has 1–1.5° of gear lash. At 1:1 coaxial that lash passes straight to
the beam, where it is **~50% of the entire 1–4° working angle**. A reduction linkage
divides it down.

| Criterion | Coaxial 1:1 | Drag link ≈1:5.6 |
|---|---|---|
| Lash at beam | 1.5° | **0.27°** |
| …as share of working angle | ~54% | ~12% |
| Servo torque required | 0.8 kg·cm | **0.17 kg·cm (11× margin)** |
| Vertical load on servo bearing | risk | **none** |
| Packaging | must sit on the pivot axis | anywhere on chassis |
| Mechanical centring | software trim only | **adjustable link** |

Coaxial only becomes competitive with a servo specified under ~0.3° backlash.

---

## 4. The ratio model

**ρ = r_horn ÷ (r_lever × sin θ)**  where θ = angle between the drag link and the lever.

- Beam range ≈ ±45° × ρ (small-angle); the exact four-bar solution gives slightly less.
- Lash at beam = 1.5° × ρ
- Deadband at beam = 0.45° × ρ
- Link force = T_beam ÷ (r_lever × sin θ)
- **Link length does not appear.** It affects nothing first-order.

Design point: r_horn = 16 mm, r_lever = 89.09 mm, θ = 90° → **1:5.6**, lash 0.27°,
link force 0.88 N. Exact four-bar solution: **±45° of servo → ≈ ±7° of beam** (the
small-angle formula predicts ±8.1°; the difference is the horn's cosine droop).

---

## 5. Link angle θ — the critical rule

**θ must be ≥ 60°, target 90°. This is the single easiest thing to get badly wrong.**

Only the perpendicular distance from the pivot to the link's line of action does work.
Pulling along the lever's own axis cannot rotate it — that's a singularity.

| θ | Effective arm | Effective ratio | Lash at beam | Link force |
|---|---|---|---|---|
| 16° | 24.6 mm | 1:1.5 | 0.97° | 3.40 N |
| 30° | 44.5 mm | 1:2.8 | 0.54° | 1.76 N |
| 45° | 63.0 mm | 1:3.9 | 0.38° | 1.25 N |
| **60°** | 77.2 mm | 1:4.8 | 0.31° | 1.02 N |
| **90°** | 89.1 mm | **1:5.6** | **0.27°** | **0.88 N** |

At θ = 16° the effective ratio collapses to 1:1.5 — worse than coaxial, *plus* two extra
ball joints and 4× the link force. Worse still, θ swings with the beam: at 16° it ranges
8°→24° across ±8° of travel, so the **gain varies ~3×** — sluggish at one end, twitchy at
the other, and miserable to tune. At 90° the gain is stationary (82°→98° changes it <1%).

**Practical consequence: the servo goes BESIDE the lever end, not behind it.** "Behind
the front axle cradle" is still satisfied — the servo lands around Y ≈ −30, well aft of
the cradle at Y 20–48 — it just needs lateral offset so the link crosses the car.

Because sin is flat near 90°, the lever's 6.4° tilt is irrelevant: running the link
straight across the car gives θ = 83.6°, costing **0.6%**. Leave the stalk alone.

---

## 6. Component selection

**Servo — MG90S (confirmed owned).** ~1.9 kg·cm and ~0.09 s/60° at 5 V; analog, so
**~5 µs deadband**; **20T spline**. Torque margin 11×, speed margin 20–60× — neither
constrains anything. The binding spec is deadband, which sets a **~0.1° beam resolution
floor** that no mechanism recovers.

**Horn — the stock one is near-optimal.** Shorter horn = more reduction = less lash and
finer resolution, at the cost of range. ~16 mm gives ±7–8° of beam, which is ~4× the
curvature the path demands (2.55 m min turn radius vs a 10.2 m path). Do not exceed
~20 mm. If buying: **20T fitment only — 25T horns are for standard-size servos and will
not fit.** Prefer a **clamping (pinch-bolt) horn**; it removes spline slop, which is a
real backlash source.

**Ball links — M2 (or match the horn's thread), turnbuckle style, 50–60 mm.** Length is
free; pick it so the servo lands somewhere it can be braced stiffly. The turnbuckle
matters more than the length: the 20T spline indexes in 18° steps, which at a 16 mm horn
is 5.0 mm of ball travel = **3.2° of beam** — more than the whole working range. Without
in-place length adjustment, one tooth of indexing error costs that much centre trim.
Need ≥ ±5 mm of thread travel. Use **heat-set brass inserts** in the printed lever, not
bare PLA.

**Mount stiffness matters more than link length.** 0.1 mm of flex under the 0.88 N link
load = 0.064° of beam error — same as a sloppy joint. Favour a short stiff mount over a
long cantilever reaching for a nicer link length.

---

## 7. Rejected: adding trail

**Decision: keep the pivot at Y = 36.01. Do not add trail.**

Trail was proposed to seat the linkage backlash. It fails, for a structural reason
rather than a tuning one: **its restoring force is proportional to tracking error.**
When the car is tracking the arc properly the front wheels point along their direction
of travel, so there is no sideways drag and no push — zero, at exactly the condition
where preload is needed. No achievable amount of trail changes that.

Magnitudes for the record: at 10 mm of trail the restoring moment is ~0.8 mN·m from
rolling drag against ~10 mN·m of pivot friction — 10× short. (At *large* misalignment,
e.g. a detached link, lateral grip of ~2.5 N gives ~25 mN·m and it would straighten —
a real but narrow failsafe.)

It also isn't free: the pivot bore has only 5.48 mm of wall ahead of it, so +10 mm of
trail would break through by 4.5 mm and require growing the beam from 28 to 36 mm deep.
Max trail without adding material is +2.5 mm, which is within the contact-patch length
and does effectively nothing.

**And note the direction, if it is ever revisited:** the pivot must move to the side of
the axle line *opposite* the lever arm (+Y here, 36.01 → 46.01). Moving it toward the
lever gives negative trail, which is destabilising.

---

## 8. Deferred: the preload band

**Decision: add anchor features to the CAD now; fit the band only if bring-up shows
weave.**

Purpose: a *constant* sideways force keeps every joint squashed against one face of its
slack, so the servo never has to cross a dead zone before the beam responds. Unlike
trail, it doesn't care whether the car is on line.

- Force: **0.3–0.5 N** (~30–50 gf) at the lever end → ~30 mN·m, about 3× pivot friction.
- Cost to the servo: **0.055 of 1.9 kg·cm — 2.9%**.
- Direction: pull the lever end **away from the servo**, keeping the drag link in tension.
- Geometry: roughly perpendicular to the lever; make it **long** (~84 mm gives ±15%
  tension variation over travel; a short band varies far more).

**Why it can be deferred:** the loop closes on IMU-measured heading, not on commanded
servo angle, so the dead zone sits *inside* the loop and the controller largely absorbs
it. Realistic dead zone is ~0.3–0.5° of beam → a few mm of lateral ripple; the stop is
encoder-triggered so it is unaffected, and the can gate has centimetres of margin.

Add two features regardless (they cost nothing): a post on the chassis and a small post
or hole near the lever end. Then watch at **BRINGUP step 9** while raising `Kh`/`Kc` — if
the arc weaves, or the gains behave differently on small errors than large ones, that is
the dead zone, and the band goes on.

---

## 9. Four-bar validation

The mechanism is a standard planar four-bar: chassis (ground), horn (crank), drag link
(coupler), lever (rocker).

- **Planar DOF = 3(4−1) − 2×4 = 1.** It moves; nothing is over-constrained.
- Link lengths 16 / 60 / 89.1 / 94.6 mm → **Grashof satisfied** (s+l = 110.6 ≤ p+q =
  149.1): crank-rocker, no toggle position anywhere near the working range.
- Solved positions: beam −8°/0°/+8° ↔ crank −135.7°/−83.6°/−33.8°, with the link
  swinging 6.4° → 11°. **That link rotation is the freedom** that absorbs the mismatch
  between two arms on different radii — the coupler is not constrained to stay parallel,
  and a parallelogram is not required.
- The link carries tension/compression as a two-force member. That is force
  transmission, not binding. What *would* seize it is rigid ends: weld both and the
  three links become one body pinned to ground twice, DOF → −1.
- **Spatial DOF with two ball joints = 6×3 − 2×5 − 2×3 = 2** (one useful + one idle spin
  of the link about its own axis). This is why ball links rather than pins — they
  tolerate out-of-plane misalignment without binding. Matching ball heights is therefore
  a *linearity* measure, not a lockup-prevention one.

---

## 10. Firmware implications

**`DELTA_MAX_RAD` is wrong and will stall the servo.** `config.h` ships 0.6 rad (34°).
The mechanism delivers about **±7° (0.12 rad)**. As written the controller can command
the servo hard into a mechanical stop, drawing stall current and risking the gear train.
Set it to the measured mechanical limit, ~**0.12–0.14 rad**.

**`SERVO_US_PER_RAD` is a linearisation about centre, not a global constant.** The
four-bar map is nonlinear (dominated by the horn's cosine droop). Calibrate it near the
operating point (±3° of beam), not at the range extremes. Storing a small lookup table
is an option if the fit is poor.

**Steering resolution floor ≈ 0.1° of beam**, from the MG90S's ~5 µs analog deadband
through the ~1:5.6 ratio. Don't chase control-loop precision below this.

**Effective servo update is 50 Hz, not `CONTROL_TICK_HZ`.** The PWM frame is 20 ms, so
calling `steerControl` at 500 Hz simply overwrites a pending frame. Harmless — the plant
timescale is L/v = 0.35–1.0 s — but don't mistake the tick rate for actuation bandwidth.

**The distance encoder is on the STEERED axle — this is a scoring bug.** `sm.cpp`
compares `odoDistanceM()` directly against `plan.L` (a centreline arc length), but the
encoder rides the beam that steers. Two errors:

1. *Track offset.* A wheel offset by t/2 = 40 mm from centreline travels (t/2)·φ further
   or shorter over the arc. At D = 8.5 m, R = 10.2 m, φ = 0.833 rad → **≈ 3.3 cm ≈ 6.7
   points** at 2 pt/cm. Fix at plan time: `L_enc = plan.L * (R + s*t/2) / R`, with
   s = +1 if the encoder wheel is outboard of the turn, −1 if inboard, 0 for a shared
   shaft. **Determine which case applies before implementing.**
2. *cos(δ) coupling.* The steered front wheels travel ~1/cos(δ) of the reference path —
   about 0.12% at δ = 2.8°, ≈ 1 cm over 8.5 m. Second-order; correct only after (1).

The clean alternative is a dedicated free-rolling wheel on the chassis centreline,
which removes both couplings entirely.

**Still blocking any hardware run:** `M_PER_COUNT = 0.0f` and `WHEELBASE_L_M = 0.0f`.
Until `M_PER_COUNT` is nonzero the car never reaches its stop condition; until
`WHEELBASE_L_M` is nonzero the steering feedforward `atan(L·κ)` is identically zero and
the loop runs on feedback alone. **These two are worth far more points than anything in
this document.**

---

## 11. Corrections to earlier docs

`docs/hardware/CAD_SUMMARY.md` contains three claims now known to be wrong:

1. **"Recommended coupling: direct, coaxial with the pivot."** Reversed — see §3. The
   drag link wins because the working angle is 1–4°.
2. **"A long (~75 mm) steering-lever arm."** That's the visible stalk. The radius about
   the pivot is **89.09 mm** — see §1.
3. **"A light preload biasing the beam toward the arc side."** That direction puts the
   drag link in *compression*. Bias the other way so it stays in tension — see §8.

`EV_Active_Tracking_Proposal.md` §3 likewise says "direct-couple the servo horn to the
pivot (no sloppy linkage)"; superseded by §3 here.

The "open steering question" in `CLAUDE.md` — which feature is the true pivot axis — is
**resolved**: the vertical bore at (15.63, 36.01), dead centre in the beam's X span and
on the wheel axle line.

---

*Geometry measured from `EV-front-axle-v2.stl` and `Front Axle.stl`. Arc parameters from
`src/path.cpp`. Servo figures are MG90S nominal at 5 V — measure actual lash and horn
radius on the bench before finalising the ratio.*
