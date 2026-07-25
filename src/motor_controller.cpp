// Harvested from robotour-pico (src/components/Motor.cpp).
// Only change: include paths ("Motor.h" -> "motor_controller.h",
// "../utils.h" -> "utils.h").
#include "motor_controller.h"
#include "utils.h"
#include <cstdio>

MotorController::MotorController(float Kp, float Ki, float Kd, float Ks,
                                 float Kv, float Ka)
    : Kp(Kp), Ki(Ki), Kd(Kd), Ks(Ks), Kv(Kv), Ka(Ka){};

void MotorController::setTargetVelocity(float target) {
  this->target = target;
}

float MotorController::update(float actual) {
  // error
  float error = target - actual;

  // --- PID
  integral += error;
  float derivative = error - previousError;
  float output = Kp * error + Ki * integral + Kd * derivative;

  // --- Feedforward
  float expected = Ks * utils::sgn(target) + Kv * target + Ka * derivative;

  previousError = error;

  // combine FF + PID
  return output + expected;
}
