# Electric Vehicle (Div C) — Rules & Scoring Model · 2027 season

*Rewritten 2026-10-18 for the 2027 rules. The previous version (2026 rules: 7–10 m, can bonus, front MP) is superseded and lives in git history — do not calibrate against it.*

**Source: the official 2027 Div C Rules Manual, Electric Vehicle C, pp. C26–C31 — `docs/rules/EV_Rules_2027.pdf`** (plain-text copy: `docs/rules/EV_Rules_2027.txt`). Verified against it 2026-10-18; rule numbers like (§3.f) refer to it. Clarifications posted on soinc.org count as part of the rules, so check the Rules Corrections page before each tournament.

**Removed in the latest version (per the coach, 2026-10-18):** §3.j (Design Knowledge) and §3.l (program authorship / single-source programs). The PDF in `docs/rules/` predates this and still contains them — ignore those two sections until the PDF is replaced with the latest version.

---

## 1. What the event is now

One autonomous run does three things:

1. Drive forward and **capture a plastic Water Bottle** standing on the centerline.
2. **Push the bottle past a taped Bottle Line**, which sits beyond the Target Point.
3. Stop, leave the bottle, and **reverse** so the **rear** Measurement Point (MP) stops as close as possible to the Target Point.

The run time is scored against a Target Time. The Target Distance, Bottle Line Distance and Target Time are all announced **after impound** and are the same for every team at a tournament.

**The car must always reverse.** The Bottle Line is at least 1.00 m past the Target and the car is at most 80 cm long. So when the bottle is delivered, the rear MP is always past the Target Point.

---

## 2. Track and targets

Coordinates: **x** is forward along the imaginary centerline from Start (0) toward Target (D); **y** is left of center.

| Quantity | Regional / Invitational | State | National |
|---|---|---|---|
| Target Distance **D** (Start → Target) | 5.00–9.00 m, 0.25 m steps | 5.00–9.00 m, 0.10 m | 5.00–9.00 m, 0.01 m |
| Bottle Line Distance **B** (past Target) | 1.00–3.00 m, 0.25 m steps | 1.00–3.00 m, 0.10 m | 1.00–3.00 m, 0.01 m |
| Bottle position (from Start, on centerline) | **2.0 m** | **3.0 m** | **4.0 m** |
| Target Time **T** | 10.0–20.0 s, 0.5 s steps | same | same |
| Event Time Bonus | — | — | yes (Final Score only) |

- Start and Target are small tape marks with a center dot. The centerline is **not drawn**.
- The Bottle Line is about 2.5 cm wide tape, perpendicular to the centerline, at least 1.0 m long. **The edge nearer the Target counts.**
- The track area is 2.0 m wide and extends at least 1.0 m past the Target. More room is likely but not guaranteed, even though B can be up to 3.00 m.
- If the ES sets up several tracks, you pick one and use it for both runs.
- **Bottle position is to the bottle's centre** — "placed centered at this location" (§7.c.ii.5).
- **Ready position:** MP over the Start Point within ~1 cm, body in front of it, toward the Target (§4.d.vii.1).
- Alignment devices are allowed (impounded, inside the Track area, not secured to the floor, no lasers). **No tape on the floor.** No rolling the car on the track floor except during runs.

---

## 3. Construction limits

| Item | 2027 rule |
|---|---|
| Length | ≤ **80.0 cm**, the **whole vehicle including the pusher** (no longer wheel to wheel) |
| Width | ≤ **35.0 cm** at any point (brackets, wires and pusher all count) |
| Batteries | ≤ **8 AA**, commercial, 1.2–1.5 V full, original label readable, each cell individually removable. **No lithium or lead** — this bans lithium AA primaries, **coin cells on boards**, and USB power banks. Batteries are impounded separately. |
| Energy storage | Cannot exceed what the batteries deliver instantaneously (no large capacitors or springs). Small filter capacitors are fine. |
| Sensors | **No cameras. No sensors containing lasers** (many ToF sensors and laser mice). Encoders and IMUs are fine. Lasers are banned everywhere, including alignment devices. |
| Electronics | Microprocessors allowed. Wi-Fi/BT/USB to the micro is allowed. |
| **Measurement Point** | At the **rear**, and it must be the **rearmost part** of the vehicle. ≤ 1.0 cm above the track. Easy for the ES to measure. |
| **Bottle Pusher** | At the front and the **frontmost part**. Exactly **one** impounded (no spare). **Rigid**: no flexing, no moving parts, no size change, not relocatable. Foremost edges rounded to about **pencil thickness (~7 mm) or more**. Must work with bottles that aren't perfectly round. The bottle does not have to fit entirely inside. If you used a kit, the pusher must be your own design and build. |
| Control | Autonomous. No RC, tethers, anchors or separate pieces. Only wheels (and parts already touching in the ready position) may touch the floor. **Anything falling off is a construction violation.** |
| Start | Unsharpened #2 pencil, vertical motion. Only the pencil touches the car. |
| **Kits** | Allowed **if**: (1) ≥ 2 **functional modifications**, recorded on the **Kit Modification Form** (impounded); (2) you build your own pusher; (3) the kit supplies only common parts — **no custom PCBs**. Breaking any of these is a **+50/run** Non-modification Penalty. **Kit-supplied 3D-printed plastic parts are exempt** from the common-parts limit (§3.k.iii). |
| **Program / AI** | AI may help design/program before competition day. **No AI during Event Time**, for people or the device. Internet may be used only for programming during Event Time. **From actuation until retrieval, students may not use or view their programming tools** (§4.d.vii.5). |

---

## 4. Scoring (lowest wins)

**Final Score = min(Run 1, Run 2) + Final Score Penalties** (+ Event Time Bonus at Nationals)

**Run Score = 100 + Distance Score + Time Score + Bonuses + Run Penalties**

| Term | Formula |
|---|---|
| **Distance Score** | **2.0 pt/cm × d_veh** (rear MP → Target dot, point to point) **+ 1.0 pt/cm × d_bottle** (Bottle Line → bottle's nearest side). If the bottle didn't get past the Target or the bottle-moving rules were broken, **d_bottle = 400.0 cm**. A Failed Run gets a Distance Score of **2500**. |
| **Time Score** | **0.5 pt/s × \|T − Run Time\|** (Failed Run time = 0.00) |
| **Bottle Bonus** | **−20** if the bottle is **fully** beyond the Bottle Line, otherwise 0. If it straddles, the smaller of its front/back distances to the line is used, with no bonus. |
| **Pusher Bonus** | **(W − 35.0 cm) × 1.5 pt/cm**, where W is the widest foremost opening, measured at check-in to 0.1 cm. No pusher means W = 35.0 and a bonus of 0. |
| Event Time Bonus | (Event Time used − 480)/30, Nationals only, Final Score only |
| Run penalties | Competition violation **+150**, Construction violation **+300**, Non-modification **+50** |
| Final penalty | Not impounded **+5000** |

**Ties:** (1) better Distance Score of the scored run (bottle term included), (2) lower Time Score of the scored run, (3) better Vehicle Distance of the scored run, (4) smaller pusher opening.

**Failed Run:** the car travels the wrong direction from the Start Point, starts before the ES is ready, can't be measured, or is pushed by the team. If you only get one successful run in 8 minutes, the second is scored as failed (no motion at all → 2 Failed Runs). If the car fails to actuate when triggered, no run has occurred and you may keep working. A team may now **ask** for a run to be scored as failed; if it's the second run, Event Time stops there.

### Bottle-moving requirements (any breach → d_bottle = 400 cm)

1. The bottle always touches the floor, and nothing goes underneath it.
2. It stays upright.
3. It is only **pushed** — never pulled, lifted or carried. A cup that drags the bottle back on reverse breaks this.
4. Only the **pusher** touches it, and only **within the measured opening**. The body, a wheel, or the outside of the pusher arms touching it is a breach.

Bottle spec: single-use clear 16.9 oz (500 mL), 6–7 cm diameter, 18–22 cm tall, 80–100 g, empty (it may contain non-lead BBs). Brands and shapes vary between tournaments.

### Run Time

- Starts at actuation. Ends at a Complete Stop: still for **3 s** (those 3 s are not counted). **Recoil or oscillations count** as Run Time. Three timers; the middle reading is official.
- **NEW:** any pause between actuation and first motion counts as Run Time and **may be longer than 3 s**. A programmed start delay is legal and is the easiest way to hit T.
- ⚠️ After the car has moved, **any stop of ≥ 3 s ends the run**. The forward→reverse turnaround must be well under 3 s.

---

## 5. Exchange rates — the strategic reading

| Error / choice | Points | ≈ Vehicle-distance equivalent |
|---|---|---|
| 1 cm rear-MP error | 2.0 | 1 cm |
| 1 cm bottle error | 1.0 | 0.5 cm |
| 1 s time error | **0.5** | 0.25 cm |
| 1 cm narrower pusher | **−1.5** | 0.75 cm |
| Bottle fully past the line | **−20** | 10 cm |
| Bottle lost / tipped / touched illegally | **≈ +420** | ≈ 210 cm |
| Failed run | ≈ +2500 | — |

1. **Rear-MP stop precision is still the most valuable precision**, now achieved on a *reverse* leg.
2. **Never lose the bottle.** A lost bottle costs more than any amount of fine-tuning saves. Capture reliability comes before pusher narrowness.
3. **Aim the bottle past the line, not at it.** Anything from 0 to ~20 cm past beats every short result. With ±4 cm scatter, aim the near side ~5 cm past.
4. **The pusher width is worth a lot of points** (35 → 10 cm ≈ −37.5). Narrow it only as far as capture testing proves safe. Expected-value check: each +1 % miss rate costs ≈ 4.2 pts.
5. **Time is now cheap** (0.5 pt/s), and a start delay handles it. Don't spend design effort on speed control beyond "the motion fits inside T."
6. **Consistency beats a hero run** (better of 2). Make run 1 conservative and safe.

---

## 6. Open questions to confirm (ES / official clarifications)

Resolved by the official manual: bottle position is to its **centre**; kit **3D-printed** parts are exempt.

- Does the Pusher Bonus still apply on a run where the bottle failed, or on a Failed Run? The manual applies it as a Run Score term with no exception stated. (Its worked example uses a 16.0 cm pusher → −28.5.)
- Is the Bottle Bonus lost after a bottle-moving violation? Not stated; assume yes.
- Does the Unphayzed kit supply any **custom PCB**? If so, the build must not use it.

---

*Sources: Science Olympiad 2027 Div C Rules Manual, Electric Vehicle C (`docs/rules/EV_Rules_2027.pdf`); soinc.org Rules Corrections page for clarifications. First drafted from the scio.ly 2027 study guide, then checked against the manual.*
