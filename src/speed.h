#pragma once
//==============================================================================
// speed — longitudinal control. Pure math (no hardware), tested in host_check.
//
// Two parts:
//   1. Speed PROFILES: target ground speed as a function of MP position, one per
//      leg. Decel ramps are v = sqrt(v_end^2 + 2*a*distance_left), i.e. constant
//      deceleration, which is exactly what plan.cpp's time model assumes.
//   2. SpeedLoop: feedforward + PI on the free roller's measured speed. Output is
//      a PWM MAGNITUDE for the commanded direction — never negative — so a forward
//      leg can slow to zero drive but can never be driven backward (§8.r.i).
//
// Time is NOT this module's job any more: the start delay (plan.cpp) absorbs it.
//==============================================================================
#include "plan.h"

// Forward leg target speed (m/s, >= 0) at MP position x: cruise, slow for the
// capture zone, push, then a gentle constant-decel ramp into xF.
float speedProfileForward(float x, const RunPlan &p);

// Reverse leg target speed MAGNITUDE (m/s, >= 0) at MP position x: cruise back,
// then a constant-decel ramp into xFinal.
float speedProfileReverse(float x, const RunPlan &p);

// Acceleration limit for a leg segment: the target may RISE no faster than this
// (falls are already shaped by the profile). Returns the limited target.
float speedRateLimit(float target, float current, float aUp, float dt);

class SpeedLoop {
 public:
  void  reset();
  // vTarget: wanted speed magnitude (m/s, >= 0) in the leg's direction.
  // vAlong:  measured speed along that direction (m/s; negative = going backward
  //          relative to the leg). Returns PWM %, in [0, PWM_MAX_PCT].
  // freezeIntegrator: hold the I term (car stuck on stiction; see sm.cpp).
  float update(float vTarget, float vAlong, float dt, bool freezeIntegrator = false);

 private:
  float i_ = 0.0f;
};

// Feedforward map: speed magnitude (m/s) -> PWM %. 0 for a zero target.
float pwmFeedforward(float v);
