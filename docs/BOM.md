# Bill of Materials & Pin Map

Confirm you have all of this before wiring. "Reuse" = already on your existing
Unphayzed EV kit; "New" = the active-tracking retrofit adds it.

## Electronics

| Part | Qty | Notes | Source |
|---|---|---|---|
| Raspberry Pi Pico 2 (RP2350) | 1 | The brain. 3.3 V logic, **not** 5 V-tolerant. | New |
| Adafruit BNO085 (BNO08x) IMU | 1 | Heading. I2C addr **0x4A**. VIN 3–5 V, onboard 10 K I2C pull-ups. | New |
| MG90S-class metal-gear micro servo | 1 | Steering. ~2.0–2.5 kg·cm. 5 V. | New |
| BTS7960 (IBT-2) motor driver | 1 | Motor 6–27 V / 43 A; **VCC logic = 5 V**, inputs accept 3.3 V. | Reuse |
| 550 DC motor — OGRC 550 **21T** brushed | 1 | Rear drive. ~1778 Kv -> ~17,000 RPM no-load on the 9.6 V pack. ⚠️ **Over-geared for this event — see §Gearing.** | Reuse |
| Gearbox — RZXYLRC full-metal, Axial SCX10 I/II | 1 | 48P. Pinion/spur **4.35:1** x internal **2.6:1** = **11.31:1 total**. Output drives the wheel shaft directly — **no axle reduction in the driveline.** | Reuse |
| Rear wheels | 2 | **2-7/8 in = 73.03 mm** OD (circumference 0.2294 m). | Reuse |
| Free-rolling front encoder — Signswise 600 P/R optical | 1 | Belt-driven off the undriven front axle = distance. DC 5–24 V, NPN open-collector; power at 5 V, pull A/B up to **3.3 V** (no level shifter). | Reuse |
| 16×2 I2C LCD (PCF8574 backpack) | 1 | Addr **0x27**. Run at **3.3 V** (see wiring). | Reuse |
| Rotary encoder dial (KY-040) + 2 buttons | 1 set | Dial = value entry; buttons = Start (pencil) + Set. | Reuse |
| 5 V buck regulator, ≥2 A | 1 | Battery → 5 V for servo, BTS7960 logic, Pico VSYS. | New |
| Bidirectional I2C/logic level shifter (BSS138) | 1 | **Encoder does NOT need one** (open-collector, pull up to 3.3 V). Only needed if the LCD must run at 5 V — see wiring §4 Option B. | New (maybe) |
| 8× AA NiMH pack (1.2 V each ≈ 9.6 V) | 1 | Rules: ≤8 AA, 1.2–1.5 V, no Li/lead-acid. ≥6 V keeps BTS7960 happy. | Reuse |
| Caps: 470 µF (servo rail), 100 µF+ (motor supply), 0.1 µF × several | — | Decoupling / spike absorption. | New |
| 4.7 kΩ resistors ×2 | opt | I2C pull-ups — likely **not needed** (BNO085 + LCD have their own). | — |
| 4.7 kΩ resistors ×2 | **req** | Encoder A/B pull-ups to **3.3 V**. Internal pull-ups are too weak next to the motor. | New |
| Perfboard / protoboard, wire, headers | — | | — |
| **16 AWG stranded wire** (14–18 fine) | ~1 m | **Motor supply only**, pack → BTS7960 B+/B−. Both legs. Never via breadboard. | New |

## Mechanical

550 motor · 2WD gearbox · rear wheels · **front pivot** (the existing steering
pivot — this is what the servo will drive) · belt + free-roller front encoder ·
chassis · LCD/dial mount. **Add:** a servo horn/mount to couple the MG90S to the
front pivot, and a plate for the Pico + level shifter + buck. **Remove:** the manual
adjustment arm as the steering *setter* (keep the caliper to set servo center).

## Pin map (Raspberry Pi Pico 2 — physical pin numbers)

Pico 2 is pin-compatible with the original Pico. Physical pins in parentheses.

| Function | Signal → | Pico GP | Phys pin | Notes |
|---|---|---|---|---|
| I2C0 SDA | **BNO085 only** | GP20 | **26** | shield `I2C0` connector |
| I2C0 SCL | **BNO085 only** | GP21 | **27** | |
| I2C1 SDA | **LCD only** | GP14 | **19** | shield `I2C1` connector |
| I2C1 SCL | **LCD only** | GP15 | **20** | separate bus — no contention |
| BNO085 INT | (optional) | GP6 | 9 | wired or not; UNUSED by firmware |
| BNO085 RST | (optional) | GP7 | 10 | clean reset |
| Front encoder A | distance | GP10 | 14 | pull up to **3.3 V**, power at 5 V |
| Front encoder B | distance | GP11 | 15 | must be consecutive with A |
| *(opt) drive-enc A/B* | — | GP12/13 | 16/17 | unused by default |
| BTS7960 RPWM | motor fwd | GP16 | 21 | 3.3 V PWM OK |
| BTS7960 LPWM | motor rev/brake | GP17 | 22 | |
| BTS7960 R_EN | enable (right) | GP18 | 24 | separate pins, written together |
| BTS7960 L_EN | enable (left) | GP26 | 31 | |
| Servo signal | steering | GP5 | 7 | 50 Hz, 1–2 ms; V+ from 5 V rail |
| Dial CLK / DT | value entry | GP2/3 | 4/5 | |
| Dial button | inc cycle | GP22 | 29 | |
| START button | pencil trigger | GP4 | 6 | to GND, active-low |
| SET button | confirm/abort | GP19 | 25 | to GND, active-low |
| 3V3(OUT) | → BNO085, LCD, dial, shifter LV | — | **36** | ≤100 mA draw |
| VSYS | ← 5 V from buck | — | **39** | do not exceed 5.5 V |
| VBUS | USB 5 V (bench) | — | 40 | |
| GND | common | — | 3,8,13,18,23,28,38 | tie all grounds |

*Sources for the electrical facts: Adafruit BNO085 pinouts (0x4A, 3–5 V VIN, 10 K
pull-ups); BTS7960/IBT-2 module docs (VCC 5 V logic, 3.3 V-compatible inputs, 6–27 V
/ 43 A motor); Raspberry Pi Pico 2 pinout. See WIRING_GUIDE.md for links.*


---

## Gearing — open issue (2026-08-22)

The rules envelope needs **0.42–1.17 m/s** (trapezoid peak, D 7–10 m over T 10–20 s).
At the current **11.31:1** with the 21T motor and 73.03 mm wheels, full duty is
**5.77 m/s** — so the whole operating range sits at **7–20% PWM**, in the brushed
motor's dead zone. Target total ratio is **26:1–44:1**.

| Option | Total | v@100% | Duty band |
|---|---|---|---|
| Current (21T, 13T/56T) | 11.3:1 | 5.77 m/s | 7–20% ❌ |
| 21T + 9T pinion (stock spur) | 16.2:1 | 4.03 m/s | 10–29% ⚠️ |
| **35T + 9T pinion (stock spur)** | **16.2:1** | **2.42 m/s** | **17–48%** ✅ |
| 21T + 9T/87T spur | 25.1:1 | 2.60 m/s | 16–45% ✅ (spur may not fit) |

**Recommended: 35T motor + 9T pinion.** Works with the stock spur, so it does not
depend on gearbox case clearance — measure that before buying any large spur.

**Do NOT** pair a 35T with a 3.75:1 axle reduction: that caps the car at 0.92 m/s and
makes D=10 m / T=10 s physically unreachable.

**This does not block bring-up.** The encoder-triggered stop works at any gearing, so
the dominant 2 pt/cm distance term is unaffected; only the 1 pt/s time term and
low-speed consistency suffer. Run the `cal.cpp` PWM sweep on the current gearing first
— measured data beats the Kv estimate above.
