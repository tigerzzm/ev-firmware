#include "speed.h"
#include "config.h"
#include "motor_controller.h"
#include <algorithm>

// Harvested MotorController regulating GROUND SPEED (m/s) from the free roller.
// Ks/Kv/Ka are for m/s -> PWM% and MUST be re-calibrated for the EV drivetrain.
// TODO(bench): tune these.
static MotorController s_speedCtl(/*Kp*/0.0f, /*Ki*/0.0f, /*Kd*/0.0f,
                                  /*Ks*/0.0f, /*Kv*/0.0f, /*Ka*/0.0f);

// Trapezoid parameters — bench-tune accel/decel to taste.
static constexpr float ACCEL_FRAC = 0.15f;   // fraction of L spent accelerating
static constexpr float DECEL_FRAC = 0.10f;   // fraction of L spent at the very end

void speedInit() {}

float speedTrapezoid(float s, float L, float T) {
  if (L <= 0.0f || T <= 0.0f) return 0.0f;
  float vCruise = L / T;                       // simple average; trapezoid peaks a bit higher
  float aZone = ACCEL_FRAC * L;
  float dZone = (1.0f - DECEL_FRAC) * L;
  // scale cruise up so the average still ~= L/T despite ramp zones
  float peak = vCruise / (1.0f - 0.5f * (ACCEL_FRAC + DECEL_FRAC));
  // Floor the ramps at a fraction of peak so the car never coasts toward v=0
  // (which would stall cruise before the closed-loop creep takes over, and makes
  // the time integral ∫ds/v diverge). The precise stop is handled in APPROACH.
  const float lo = 0.25f * peak;
  if (s < aZone) return lo + (peak - lo) * (s / std::max(aZone, 1e-3f));    // ramp up
  if (s < dZone) return peak;                                              // cruise
  float t = (L - s) / std::max(L - dZone, 1e-3f);                          // ramp down
  return lo + (peak - lo) * std::clamp(t, 0.0f, 1.0f);
}

float pwmForSpeed(float mps) {
  // TODO(bench): replace with interpolation over the calibrated PWM->speed table.
  // Placeholder linear guess so the structure compiles and runs (NOT accurate).
  float pwm = mps * 40.0f;                      // <-- CALIBRATE
  return std::clamp(pwm, 0.0f, 100.0f);
}

float speedControl(float s, float L, float T, float measuredMps) {
  float vRef = speedTrapezoid(s, L, T);
  float pwmFf = pwmForSpeed(vRef);              // feedforward from table
  // Optional inner loop (enable once s_speedCtl gains are tuned):
  //   s_speedCtl.setTargetVelocity(vRef);
  //   pwmFf += s_speedCtl.update(measuredMps);
  (void)measuredMps;
  return std::clamp(pwmFf, 0.0f, 100.0f);
}
