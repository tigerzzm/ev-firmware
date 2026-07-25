# ev-firmware

Science Olympiad **Electric Vehicle** firmware for the **Raspberry Pi Pico 2 (RP2350)** —
single 550 rear drive (BTS7960), servo-steered front pivot, BNO085 heading,
free-rolling front encoder for distance, constant-curvature arc path, and an
encoder-triggered closed-loop stop.

Seeded by harvesting the proven low-level modules from the **robotour-pico**
project (IMU driver, PIO encoder, motor PID+feedforward, dual-core estimator,
utils, RAM logger) and building the EV-specific control on top. See
**`HARVEST_NOTES.md`** for provenance, what changed, and what's left to write/tune.

## Layout (`src/`)

```
main.cpp        core0 = state machine, core1 = pose estimator
config.h        pin map + calibration constants  (SET THESE)
sm.*            state machine: SETUP→ARMED→LAUNCH→CRUISE→APPROACH→BRAKE→DONE
imu.*           BNO085 Game Rotation Vector (harvested+adapted)
odo.*           free-roller distance via PIO
pose.*          (x,y,θ) dead reckoning, IMU-zeroed at arm
path.*          constant-curvature arc geometry
steer.*         servo output + lateral control law
speed.*         trapezoid + PWM→speed table (+ optional MotorController loop)
drive.*         BTS7960 helpers
ui.*            LCD + dial + buttons  (STUB — port from existing EV kit)
BNO08x/         vendored Adafruit BNO08x + sh2 library (verbatim)
pid.* motor_controller.* utils.* position.* ram_*   harvested support
quadrature_encoder.pio                              harvested PIO program
```

Build instructions and the calibration checklist are in `HARVEST_NOTES.md`.
