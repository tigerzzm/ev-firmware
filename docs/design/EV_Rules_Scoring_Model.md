# Electric Vehicle (Div C) — Rules & Scoring Model

**Season note:** Science Olympiad names seasons by the national tournament year, so the **2026–2027 school year = the 2027 season**. Electric Vehicle is confirmed on the official **2027 Division C** event list, but as of July 2026 the **2027 rules manual is not yet released** ("2027 resources coming soon"). Everything below is from the **2026 rules manual**, which is the best available basis. EV is a returning event, so expect the 2027 rules to be very close, with at most minor numeric tweaks. Re-verify against the 2027 manual the moment it posts (typically early fall).

---

## 1. What the event is

Design, build, and test **one** vehicle that uses **electrical energy as its sole means of propulsion** to travel down a track, **arrive at a Target Point at a Target Time**, and **stop as close to the Target Point as possible**.

Both the **Target Distance** and the **Target Time** are announced **after impound** — so the vehicle must be **re-tunable on the spot** across the full range, not built for a single fixed distance.

---

## 2. Construction parameters (hard limits)

| Parameter | Limit |
|---|---|
| Wheelbase (front of front wheel → back of back wheel) | ≤ **70.0 cm** |
| Width (at any point) | ≤ **35.0 cm** |
| Batteries | ≤ **8 AA**, 1.2–1.5 V each; **no lithium, no lead-acid** |
| Energy storage | Only instantaneous battery output — **no large capacitors / springs / flywheels** storing extra energy |
| Propulsion | Electric, from the batteries only |
| Measurement Point (MP) | A defined point at the **front** of the vehicle, **≤ 1.0 cm above the track** |
| Remote control / tethers / anchors | **Prohibited** — the whole vehicle moves as one unit |
| Start trigger | A **#2 unsharpened pencil**, **vertical motion** only (no lasers, no electronic remote start) |
| Components | May be purchased or made; microprocessors and electronics **allowed** |

---

## 3. Target ranges (announced after impound)

- **Target Distance:** 7.00 – 10.00 m
  - Interval by level: **0.25 m regionals · 0.10 m states · 0.01 m nationals**
- **Target Time:** 10.0 – 20.0 s, in **0.5 s** steps

---

## 4. Scoring — the part that should drive every design decision

**Final Score = better of 2 Run Scores + Final Score Penalties.** **Lowest score wins.**

**Run Score = 100 + Distance Score + Time Score + Bonuses + Run Penalties**

| Term | Formula | What it means |
|---|---|---|
| **Distance Score** | **2.0 pt/cm × Vehicle Distance** (MP → Target Point) | Every **1 cm** you stop away from the target = **2 points**. Dominant term. |
| **Time Score** | **\|Target Time − Run Time\|** | Every **1 second** off the target time = **1 point**. |
| **Can Bonus** | **−0.5 × (110 − Inside Can Distance in cm)** | Passing between two cans; narrower gap = bigger negative (better) bonus. Risk/reward. |
| **Event Time Bonus** (nationals only) | (Event Time Used − 480) / 30 | Small bonus for finishing under the 8-minute window. Minor. |
| **Failed run** | Distance Score = **2500**, Run Time recorded as 0.00 | Effectively a wasted run — avoid at all costs. |

**Penalties:** Competition violation **+150/run**, Construction violation **+300/run**, Not impounded **+5000** (Tier 3).

---

## 5. Strategic reading of the math (the key insight)

1. **Distance accuracy is ~2× more valuable per unit than time accuracy.** 1 cm error = 2 pts; 1 s error = 1 pt. Realistic time errors are a fraction of a second (< 1 pt), while distance errors of several cm are common. **Stopping precision is the #1 score driver.** Getting from ±5 cm to ±1 cm is worth ~8 points — usually the difference between medaling and not.

2. **"Better of 2 runs" rewards consistency over a single hero run.** You get two attempts and keep the better. The goal is: make **both** runs safe (never a 2500 failure) and make at least **one** excellent. A design whose spread is ±1 cm beats one that hits dead-center once and misses by 10 cm the other time.

3. **The vehicle must be TUNABLE, fast.** Distance and time are revealed only after impound, and you have an **8-minute** window for up to 2 runs including adjustment. Whatever sets distance and speed must be **quick and repeatable to dial in** — this is where implementation complexity earns or loses you the event. A design that needs fiddly re-tuning wastes the window and adds variance.

4. **Decouple distance from time.** Ideally the mechanism/algorithm that controls **where** it stops is independent of the one that controls **how fast** it goes, so tuning one doesn't disturb the other.

5. **Can Bonus is the main way below the ~100 floor,** but only worth it if your stopping is already reliable — don't chase the bonus at the cost of a failed run.

---

## 6. Design implications (to apply to the current build)

- **Closed-loop distance control** (wheel encoder / hall-effect counts) is the cleanest path to sub-cm stopping across the 7–10 m range without physical rebuilds: set target counts, brake hard on reaching them.
- **Speed/time control via PWM** lets you hit the target time by choosing an average speed = distance / target time, independent of the distance stop.
- **A strong, repeatable brake** (active motor braking + a positive mechanical stop) matters more than top speed. Consistency of the *stop* is the score.
- **Low, consistent rolling friction** (quality bearings, aligned axles, straight tracking) reduces run-to-run variance — the enemy of "better of 2."
- **Keep tuning to a single number or two** (e.g., enter distance + time on a small display/dials) so the 8-minute window is spent running, not rebuilding.

---

*Sources: Science Olympiad 2026 Division C Rules Manual (Electric Vehicle section); soinc.org Electric Vehicle event page, FAQ, and 2026 Team Checklist; NC Science Olympiad.*
