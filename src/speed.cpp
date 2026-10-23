#include "speed.h"
#include "config.h"
#include <algorithm>
#include <cmath>

// v reached after decelerating at `a` over `dist` metres down to `vEnd`, read
// backwards: the fastest you can be going `dist` before the stop and still make it.
static float rampSpeed(float vEnd, float a, float dist) {
  return std::sqrt(vEnd * vEnd + 2.0f * a * std::max(dist, 0.0f));
}

float speedProfileForward(float x, const RunPlan &p) {
  if (x < p.captureStart) {
    // Cruise, but arrive at the capture zone already down at V_CAPTURE.
    return std::min(p.v.vCruise, rampSpeed(V_CAPTURE, A_DECEL, p.captureStart - x));
  }
  if (x < p.pushStart) return V_CAPTURE;          // first contact: seat, don't kick
  // Push, ramping down at A_RELEASE (< mu*g) so the bottle stays seated.
  return std::min(p.v.vPush, rampSpeed(V_PUSH_END, A_RELEASE, p.xF - x));
}

float speedProfileReverse(float x, const RunPlan &p) {
  return std::min(p.v.vRev, rampSpeed(V_REV_END, A_REV_STOP, x - p.xFinal));
}

float speedRateLimit(float target, float current, float aUp, float dt) {
  if (target <= current) return target;           // slowing: the profile owns it
  return std::min(target, current + aUp * dt);
}

float pwmFeedforward(float v) {
  if (v <= 0.0f) return 0.0f;
  return SPEED_FF_OFFSET_PCT + SPEED_FF_PCT_PER_MPS * v;
}

void SpeedLoop::reset() { i_ = 0.0f; }

float SpeedLoop::update(float vTarget, float vAlong, float dt, bool freezeIntegrator) {
  if (vTarget <= 0.0f) { i_ = 0.0f; return 0.0f; }
  const float err = vTarget - vAlong;
  const float ff  = pwmFeedforward(vTarget);
  const float pOut = SPEED_KP * err;

  // Integrate, then clamp. Conditional integration (anti-windup): don't let the
  // integrator keep growing while the output is already pinned at a limit in the
  // same direction — otherwise it overshoots badly once the car breaks free.
  const float iNext = std::clamp(i_ + SPEED_KI * err * dt, -SPEED_I_MAX, SPEED_I_MAX);
  const float raw   = ff + pOut + iNext;
  const bool pinnedHigh = raw > PWM_MAX_PCT && err > 0.0f;
  const bool pinnedLow  = raw < 0.0f && err < 0.0f;
  if (!pinnedHigh && !pinnedLow && !freezeIntegrator) i_ = iNext;

  return std::clamp(ff + pOut + i_, 0.0f, PWM_MAX_PCT);
}
