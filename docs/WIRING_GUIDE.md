# EV — Electrical Wiring Guide

Wire in this order: **power tree first, then grounds, then signals, then the
pre-power checklist, and only then connect the battery.** A visual is in
`wiring_diagram.html` (open it in a browser).

> ⚠️ **The one rule that saves parts:** the Pico 2 (RP2350) is 3.3 V logic and is
> **NOT 5 V-tolerant.** Any 5 V on a GPIO can damage the chip. The two places this
> bites you are the **encoder** (if it's a 5 V unit) and the **LCD** (PCF8574
> backpacks are often run at 5 V). Both are handled below.

---

## 1. Power tree (build this first)

```
            8×AA NiMH pack  (~9.6 V nominal, rules max 8 cells)
                 │  +                                   │  −
                 ├───────────────────────────┐         │
                 │                            │         │
        ┌────────▼─────────┐        ┌─────────▼──────┐  │
        │  BTS7960  B+      │        │  Buck reg IN+  │  │
        │  (motor supply)   │        │  → OUT 5.0 V   │  │
        │  B− ───────────────────────┼── IN−          │  │
        └────────┬──────────┘        └───┬───┬────┬───┘  │
             M+/M− │                     │5V │5V  │5V     │
                   ▼                     ▼   ▼    ▼        │
                550 motor          Servo  BTS   Pico      │
                                    V+   VCC   VSYS(pin39) │
                                                          │
   Pico 3V3(OUT, pin36) ──► BNO085 VIN, LCD VCC, KY-040 +, shifter LV
                                                          │
   ALL GROUNDS COMMON  ◄──────────────────────────────────┘
   (pack −, buck −, Pico GND, BTS7960 GND, servo GND, BNO GND, LCD GND, encoder GND)
```

**Rules of the power tree**
- **Motor power** goes battery → BTS7960 `B+`/`B−` on its own **thick leads —
  16 AWG is plenty** (14–18 all fine; 12 AWG is unnecessary here). Both legs must be
  thick: current flows in a loop, so the return carries the same amps as the feed.
  **This path must NEVER go through a breadboard** — contacts are rated ~1 A each and
  their resistance alone will stop the motor starting. Bring its ground back to the
  pack, not through the Pico's ground.
  *Sizing:* 8×AA NiMH have ~240 mΩ internal resistance, capping the starting surge
  near 20–25 A with 2–5 A continuous. Over a 30 cm run that is 0.16 V of drop at
  16 AWG; going to 12 AWG saves 0.1 V and costs you a stiff, hard-to-route lead.
- **5 V rail** comes from the buck (≥2 A). It feeds three things: the **servo V+**,
  the **BTS7960 `VCC`** (logic supply — this is 5 V), and the **Pico `VSYS`** (pin 39).
  Do **not** exceed 5.5 V into VSYS.
- **3.3 V rail** is the Pico's own regulator, `3V3(OUT)` pin 36 (good for ~100 mA).
  It powers the BNO085, the LCD (see §4), the KY-040 dial, and the level shifter's
  LV side. Don't hang the motor driver or servo off it.
- **Common ground** is non-negotiable: every ground ties together. A floating ground
  between the Pico and the BTS7960 is the classic "motor twitches, logic resets" bug.
- **Bench power:** while flashing/testing you can power the Pico from **USB** (VBUS)
  for the serial monitor. Powering VSYS from the buck at the same time is fine (the
  Pico OR-s them through a diode). For a real run, the pack + buck powers everything.

---

## 2. Motor driver — BTS7960 / IBT-2

| BTS7960 pin | Connect to | Note |
|---|---|---|
| B+ / B− | Battery pack + / − | Motor supply, **16 AWG, direct — never via breadboard or a regulator**. 6–27 V, so 9.6 V pack is fine. |
| M+ / M− | 550 motor terminals | If the motor spins backward, swap these two. |
| VCC | **5 V** rail (buck) | Logic supply. |
| GND | common ground | |
| RPWM | Pico **GP16** (pin 21) | Forward PWM. 3.3 V drive is accepted. |
| LPWM | Pico **GP17** (pin 22) | Reverse / brake PWM. |
| R_EN | Pico **GP18** (pin 24) | Right half-bridge inhibit. |
| L_EN | Pico **GP26** (pin 31) | Left half-bridge inhibit. Separate pin, but firmware always writes both together. |
| R_IS / L_IS | leave unconnected | Optional current sense (not used). |

Add a **100 µF+** electrolytic across B+/B− near the driver to absorb motor spikes.

> ⚠️ **Never regulate the motor supply.** B+ goes straight to the pack. Putting a buck
> converter in this path current-limits the motor to the converter's rating (2–3 A),
> which is far below the starting surge — the motor then hums and will not start, but
> spins fine if you help it by hand. The BTS7960 takes 6–27 V directly; speed is set
> by PWM duty, not by regulating the supply.

---

## 3. IMU — Adafruit BNO085 (heading)

| BNO085 pin | Connect to | Note |
|---|---|---|
| VIN | **3.3 V** (Pico pin 36) | Regulates internally; 3.3 V is cleanest. |
| GND | common | |
| SDA | Pico **GP20** (pin 26) | I2C0 — **shield's "I2C0" connector** |
| SCL | Pico **GP21** (pin 27) | |
| INT | Pico GP6 (pin 9) | Optional; firmware works without it. |
| RST | Pico GP7 (pin 10) | Optional; clean reset. |

The shield's 4-pin `I2C0` connector carries SDA/SCL/3V3/GND together — use it
rather than the 3-pin rows, which would need power and ground duplicated.

Address is **0x4A** (leave the address pin alone). The breakout already has **10 K
pull-ups** on SDA/SCL, so you likely don't need to add any.

---

## 4. LCD — 16×2 with PCF8574 I2C backpack  ← read this carefully

**The LCD is on its own bus: I2C1, GP14/GP15** — the shield's `I2C1` connector.
It does NOT share with the BNO085 (I2C0, GP20/GP21). Two separate controllers, so the two devices can never collide —
and the display stays live during a run.

The catch: many PCF8574 backpacks are run at **5 V**, which would put 5 V on
GP14/GP15 and **damage the Pico.** (It can no longer reach the BNO085, which is one
more reason for the split.) Two safe options:

- **Option A (simplest, recommended): power the LCD backpack at 3.3 V.** PCF8574
  modules are typically rated ~2.5–6 V, so at 3.3 V the logic is bus-safe. The only
  downside is a dimmer display — turn the little **contrast pot** on the back up. If
  it's readable, you're done.
- **Option B (if it's too dim at 3.3 V): run the LCD at 5 V behind a bidirectional
  I2C level shifter.** Put the BNO085 + Pico on the shifter's **LV (3.3 V)** side and
  the LCD on the **HV (5 V)** side. Never wire a 5 V LCD directly to the Pico bus.

| LCD pin | Connect to |
|---|---|
| VCC | 3.3 V (Option A) **or** 5 V via level-shifter HV (Option B) |
| GND | common |
| SDA | Pico **GP14 (pin 19)** — direct (A) or shifter LV (B) |
| SCL | Pico **GP15 (pin 20)** — direct (A) or shifter LV (B) |

The shield's `I2C1` connector is exactly these pins.

Address **0x27** (some are 0x3F — if the LCD stays blank, that's the first thing to
check, along with the contrast pot). No conflict with the BNO085 at 0x4A.

---

## 5. Steering servo (MG90S-class)

| Servo wire | Connect to | Note |
|---|---|---|
| Signal (usually orange/white) | Pico **GP5** (pin 7) | 50 Hz, 1000–2000 µs. |
| V+ (red) | **5 V** rail (buck) | **Not** a Pico pin — servos spike >1 A. |
| GND (brown/black) | common | |

Add a **470 µF** cap across the servo's 5 V rail to ride out stall spikes.

---

## 6. Free-rolling front encoder (distance)

**Confirmed part: Signswise incremental optical encoder, DC 5–24 V, 6 mm shaft,
NPN open-collector output** (Amazon B085ZLCYS1). Wiring below is specific to it.

| Encoder wire | Connect to | Note |
|---|---|---|
| **Green** = A | Pico **GP10** (pin 14) | + 4.7 k pull-up to **3.3 V** |
| **White** = B | Pico **GP11** (pin 15) | + 4.7 k pull-up to **3.3 V**. Must be consecutive with A (PIO reads A on pin, B on pin+1). |
| **Red** = V+ | **5 V** rail (buck) | Needs ≥5 V — it will **not** run at 3.3 V. |
| **Black** = V0 | common ground | |
| **Shield** | ground at the **Pico end only** | One end only, or you make a ground loop. |

**No level shifter needed.** The output is **NPN open-collector**: the transistor only
ever pulls the line down to GND, it never sources voltage. So the pull-up resistor —
not the supply — sets the logic high level. Pull up to **3.3 V** and the line swings
0 ↔ 3.3 V, which is Pico-safe even though the encoder body runs on 5 V.

> ⚠️ **Pull up to 3.3 V, NEVER to 5 V.** A 5 V pull-up puts 5 V on GP10/GP11 and
> damages the Pico. This is the whole trick — get it wrong and the "no level shifter
> needed" logic stops holding.

> ⚠️ **Never connect A or B directly to VCC** — that burns the output transistor
> (manufacturer's own warning).

The PIO enables the Pico's internal pull-ups (~50–80 kΩ) at init, so it may work with
no external resistors at all. Fit the 4.7 k anyway: the internal ones are weak, and
weak pull-ups on an open-collector line give slow rise times and poor noise immunity
right next to a 550 motor.

---

## 7. Dial + buttons (UI)

- **Dial (KY-040 or bare EN12):** `+` → 3.3 V, `GND` → common, `CLK` → **GP2** (4),
  `DT` → **GP3** (5), `SW` (push) → **GP22** (29). A bare encoder needs no external
  pull-ups — the firmware enables the internal ones.
- **START button:** one leg → **GP4** (6), other leg → **GND**. (Firmware enables
  the internal pull-up; pressed = LOW. This is the pin the **#2-pencil trigger**
  presses.)
- **SET button:** one leg → **GP19** (25), other leg → **GND**.

---

## 8. Decoupling (don't skip)

- 100 µF+ across the BTS7960 motor supply (B+/B−).
- 470 µF near the servo 5 V rail.
- 0.1 µF ceramic across VCC/GND at each IC (Pico, BNO085, LCD backpack).

---

## 9. PRE-POWER CHECKLIST (before the battery goes in)

Go through every line with a multimeter. Do **not** skip this.

1. **Continuity / shorts:** with power OFF, check there is **no** short between the
   5 V rail and GND, the 3.3 V rail and GND, or B+ and B−.
2. **Buck output:** power just the buck from the pack, meter its output = **5.0 V**
   (±0.2) *before* anything else is connected to it. Adjust the pot if it's a
   variable buck.
3. **No 5 V on the 3.3 V bus:** confirm the BNO085 VIN and LCD VCC (Option A) read
   3.3 V, not 5 V. Confirm nothing 5 V (LCD Option B, 5 V encoder) touches GP10/GP11/
   GP14/GP15/GP20/GP21 without going through the level shifter.
4. **VSYS ≤ 5.5 V:** meter the wire going to Pico pin 39; it should be ~5 V.
5. **Grounds common:** continuity between pack−, buck−, Pico GND, BTS7960 GND, servo
   GND, BNO GND, LCD GND, encoder GND — all should beep.
6. **Polarity:** battery +/− to B+/B− and buck IN correct; servo and BNO not reversed.
7. **Enables:** BTS7960 R_EN → GP18 (pin 24), L_EN → GP26 (pin 31). Two separate
   wires — do **not** jumper them together on the module. If your board has a
   factory jumper linking R_EN and L_EN, remove it.
8. **Wheels off the ground** for first power-on.

Only after all eight pass: connect the battery, then move to `BUILD_AND_FLASH.md`.

---

*Verified electrical facts — sources:*
- *Adafruit BNO085 pinouts (addr 0x4A, VIN 3–5 V, onboard 10 K I2C pull-ups, level-shifted I2C):* https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/pinouts
- *BTS7960 / IBT-2 module (VCC 5 V logic, inputs accept 3.3 V, motor 6–27 V / 43 A):* https://docs.cirkitdesigner.com/component/d7d3f050-2543-47b2-872e-dab4f5f1d532/bts7960-ibt-2-motor-driver
- *Raspberry Pi Pico 2 pinout (physical pin numbers):* https://pico2.pinout.xyz/
