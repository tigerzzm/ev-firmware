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
| 550 DC motor + 2WD gearbox | 1 | Rear drive. | Reuse |
| Free-rolling front encoder (≈1200 PPR) | 1 | Belt-driven off the undriven front axle = distance. Check its logic voltage. | Reuse |
| 16×2 I2C LCD (PCF8574 backpack) | 1 | Addr **0x27**. Run at **3.3 V** (see wiring). | Reuse |
| Rotary encoder dial (KY-040) + 2 buttons | 1 set | Dial = value entry; buttons = Start (pencil) + Set. | Reuse |
| 5 V buck regulator, ≥2 A | 1 | Battery → 5 V for servo, BTS7960 logic, Pico VSYS. | New |
| Bidirectional I2C/logic level shifter (BSS138) | 1 | For a **5 V encoder** and/or a 5 V LCD. Skip if both are 3.3 V. | New (maybe) |
| 8× AA NiMH pack (1.2 V each ≈ 9.6 V) | 1 | Rules: ≤8 AA, 1.2–1.5 V, no Li/lead-acid. ≥6 V keeps BTS7960 happy. | Reuse |
| Caps: 470 µF (servo rail), 100 µF+ (motor supply), 0.1 µF × several | — | Decoupling / spike absorption. | New |
| 4.7 kΩ resistors ×2 | opt | I2C pull-ups — likely **not needed** (BNO085 + LCD have their own). | — |
| Perfboard / protoboard, wire (incl. 12 AWG for motor), headers | — | | — |

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
| I2C0 SDA | BNO085 + LCD | GP4 | **6** | 3.3 V bus |
| I2C0 SCL | BNO085 + LCD | GP5 | **7** | |
| BNO085 INT | (optional) | GP6 | 9 | data-ready |
| BNO085 RST | (optional) | GP7 | 10 | clean reset |
| Front encoder A | distance | GP10 | 14 | level-shift if 5 V |
| Front encoder B | distance | GP11 | 15 | |
| *(opt) drive-enc A/B* | — | GP12/13 | 16/17 | unused by default |
| BTS7960 RPWM | motor fwd | GP16 | 21 | 3.3 V PWM OK |
| BTS7960 LPWM | motor rev/brake | GP17 | 22 | |
| BTS7960 R_EN+L_EN | enable | GP18 | 24 | tie both together |
| Servo signal | steering | GP15 | 20 | 50 Hz, 1–2 ms |
| Dial CLK / DT | value entry | GP20/21 | 26/27 | KY-040 |
| Dial button | inc cycle | GP22 | 29 | |
| START button | pencil trigger | GP14 | 19 | to GND, active-low |
| SET button | confirm | GP19 | 25 | to GND, active-low |
| 3V3(OUT) | → BNO085, LCD, dial, shifter LV | — | **36** | ≤100 mA draw |
| VSYS | ← 5 V from buck | — | **39** | do not exceed 5.5 V |
| VBUS | USB 5 V (bench) | — | 40 | |
| GND | common | — | 3,8,13,18,23,28,38 | tie all grounds |

*Sources for the electrical facts: Adafruit BNO085 pinouts (0x4A, 3–5 V VIN, 10 K
pull-ups); BTS7960/IBT-2 module docs (VCC 5 V logic, 3.3 V-compatible inputs, 6–27 V
/ 43 A motor); Raspberry Pi Pico 2 pinout. See WIRING_GUIDE.md for links.*
