// Harvested verbatim from robotour-pico (src/components/PID.cpp).
// Only change: include path "PID.h" -> "pid.h".
#include "pid.h"

#include <cstdio>

// Init-list order matches member declaration order (debug first) to avoid -Wreorder.
PIDController::PIDController(double kP, double kI, double kD)
    : debug(false), _kP(kP), _kI(kI), _kD(kD) {}

PIDController::PIDController(double kP, double kI, double kD, bool debug)
    : debug(debug), _kP(kP), _kI(kI), _kD(kD) {}

double PIDController::update(double error) {
  _integral += error;
  double derivative = error - _previousError;

  double kPOutput = _kP * error;
  double kIOutput = _kI * _integral;
  double kDOutput = _kD * derivative;

  if (debug)
    printf("PID: err=%.3f, kP=%.3f, kI=%.3f, kD=%.3f\n", error, kPOutput, kIOutput, kDOutput);

  double output = kPOutput + kIOutput + kDOutput;

  _previousError = error;
  return output;
}

void PIDController::reset() {
  _previousError = 0;
  _integral = 0;

  if (debug) printf("PID reset\n");
}
