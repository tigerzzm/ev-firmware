# Electric Vehicle — How the Current Design Works

*Plain-language walkthrough of the Unphayzed Electric Champion Kit 2025–2026 build (hardware + software), as currently built. Companion to `EV_Design_Review.md` and `EV_Rules_Scoring_Model.md`.*

---

## Hardware

An **Arduino UNO** is the brain. It drives a single **DC gear motor** through a **BTS7960 H-bridge driver** (pins RPWM=10, LPWM=9, plus enable pins), which lets the Arduino control both **speed** (via PWM) and **direction** (forward / reverse).

A **quadrature encoder** on the wheel (**1200 counts/rev**, on interrupt pins 2 and 3) reports exactly how far the car has rolled. With the **7.30 cm** wheel, that's about **52 counts per cm** — sub-millimeter distance resolution.

Power is up to **8 AA batteries** (rules max; no lithium/lead-acid).

The driver interface is an **I2C LCD** plus a **rotary dial** and two buttons (**Set**, **Start**): you dial in the target distance and target time and confirm.

Steering is **not powered** — the front axle sits at a fixed angle (the "front pivot" / adjustment-arm parts), so the car drives a gentle **arc**. That arc is how it threads between the two cans for the **can bonus**.

Braking is done by the **motor itself** (a short reverse pulse), not a separate brake mechanism.

---

## Software

**Setup — turn the target into an encoder count.** It takes the straight-line target distance, uses `getArcLength()` to convert that chord into the longer **arc** the car actually drives, then converts the arc into a **target encoder count**.

**The run happens in three phases:**

1. **Cruise** — run the motor at a fixed **50% power** until the encoder shows the car is about **1 m short** of the target.

2. **Calibrate** — one short **reverse brake**, then **5 small test pulses** (10% power, 200 ms each) while watching the encoder, to measure how far **one pulse** moves the car.

3. **Snail** — using that measurement, compute **once** how many small pulses are needed to cover the last meter, then fire exactly that many, **spacing them out in time** to burn off the remaining seconds so the car arrives near the **target time**. Then it stops and shows the final distance and time on the LCD.

**In short:** distance is controlled by **counting encoder pulses**, and time is controlled almost entirely in that final meter by **how far apart the last pulses are spaced**.

---

## The key characteristic to remember

Phase 3 runs those final pulses **open-loop** — it commits to a pulse count up front and does **not** re-check the encoder as it finishes. That's why the stopping point drifts from run to run (and why run 2, on a weaker battery, tends to fall short). See `EV_Design_Review.md` for the fix.
