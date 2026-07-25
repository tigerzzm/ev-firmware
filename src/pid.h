#pragma once

// Harvested verbatim from robotour-pico (src/components/PID.h).
// Generic scalar PID. Used here for the steering heading loop (and anywhere
// else a simple PID is handy). The drive speed loop uses MotorController
// (motor_controller.h), which adds feedforward.
class PIDController {
 public:
  PIDController(double kP, double kI, double kD);
  PIDController(double kP, double kI, double kD, bool debug);
  double update(double error);
  void reset();

 private:
  bool debug;
  double _kP, _kI, _kD;
  double _previousError = 0;
  double _integral = 0;
};
