/**
 * Manage output for the drive motor: PID + feedforward velocity controller.
 *
 * Harvested from robotour-pico (src/components/Motor.h), unchanged logic.
 *
 * EV usage note: in robotour this regulated each driven wheel's RPM. In the EV
 * it regulates the vehicle's GROUND SPEED (cm/s) measured from the free-rolling
 * front encoder (see odo.h / speed.h). Same controller, different plant + units,
 * so Ks/Kv/Ka must be re-calibrated for cm/s -> PWM% on the EV.
 *
 * Designed for multicore use: one core sets the target, the other updates PID/ff.
 *
 * @author - Derock X (derock@derock.dev)  @license - MIT
 */
#pragma once

class MotorController {
 public:
  MotorController(float Kp, float Ki, float Kd, float Ks = 0, float Kv = 0,
                  float Ka = 0);

  void setTargetVelocity(float target);
  float update(float actual);
  float getTargetVelocity() const { return target; }

 private:
  // PID constants
  float Kp, Ki, Kd;
  // Feedforward constants
  float Ks = 0, Kv = 0, Ka = 0;
  // runtime state
  float target = 0;
  float previousError = 0;
  float integral;
};
