#include "steer_law.h"
#include "config.h"
#include "utils.h"
#include <algorithm>
#include <cmath>

// Heading error a - b wrapped to [-pi, pi], so it never takes the long way round.
static float headingError(float a, float b) { return (float)utils::angleError(a, b, true); }

static float clampDelta(float d) { return std::clamp(d, -DELTA_LIMIT_RAD, DELTA_LIMIT_RAD); }

// Points on the car's centreline behind the roller, from the roller pose: the
// rear axle is WHEELBASE_L_M behind it, the rear MP ENC_FROM_MP_M behind it.
static float behindX(float xRoller, float theta, float dist) { return xRoller - dist * std::cos(theta); }
static float behindY(float yRoller, float theta, float dist) { return yRoller - dist * std::sin(theta); }

//------------------------------------------------------------------------------
// Plain `if` on a constant, not `if constexpr`: both branches stay compiled in
// every configuration (so the unselected law can't rot), and the compiler drops
// the dead one anyway.
float steerLawForward(float yRoller, float theta, float speedAbs) {
  if (STEER_FWD_LAW == SteerLaw::PurePursuit) {
    return steerLawPurePursuit(behindY(yRoller, theta, WHEELBASE_L_M), theta,
                               PP_LOOKAHEAD_M, WHEELBASE_L_M);
  }
  return steerLawStanley(yRoller, theta, speedAbs);
}

float steerLawReverse(float xRoller, float yRoller, float theta, float speedAbs,
                      float xTarget, float lastDelta) {
  if (STEER_REV_LAW == RevSteerLaw::PurePursuit) {
    return steerLawPurePursuitReverse(behindY(yRoller, theta, WHEELBASE_L_M), theta,
                                      PP_LOOKAHEAD_M, WHEELBASE_L_M);
  }
  if (STEER_REV_LAW == RevSteerLaw::Stanley) {
    return steerLawStanleyReverse(behindY(yRoller, theta, ENC_FROM_MP_M), theta, speedAbs);
  }
  return steerLawAimAtTarget(behindX(xRoller, theta, ENC_FROM_MP_M),
                             behindY(yRoller, theta, ENC_FROM_MP_M),
                             theta, xTarget, lastDelta);
}

//------------------------------------------------------------------------------
float steerLawStanley(float yFront, float theta, float speedAbs) {
  const float eHead = headingError(0.0f, theta);
  // MINUS: y > 0 means the car is left of the line, which needs a RIGHT (negative)
  // steer. The 2026 code added this term, which made the loop unstable — both
  // terms must push the same way. host_check's closed-loop test guards it.
  const float delta = STEER_KH * eHead -
                      std::atan2(STEER_KC * yFront, speedAbs + STEER_SPEED_EPS);
  return clampDelta(delta);
}

float steerLawPurePursuit(float yRear, float theta, float lookahead, float wheelbase) {
  // Goal point: where a circle of radius Ld around the rear axle crosses the line,
  // on the far side. Out of reach (|y| > Ld): aim square at the line.
  const float dx    = std::sqrt(std::max(lookahead * lookahead - yRear * yRear, 0.0f));
  const float alpha = headingError(std::atan2(-yRear, dx), theta);
  // Curvature of the arc from the rear axle, tangent to the heading, through the
  // goal point is 2 sin(alpha) / Ld; the bicycle model turns that into a steer.
  return clampDelta(std::atan(2.0f * wheelbase * std::sin(alpha) / lookahead));
}

float steerLawAimAtTarget(float xMp, float yMp, float theta, float xTarget, float lastDelta) {
  const float ahead = xMp - xTarget;          // distance still to back up
  if (ahead < REV_STEER_HOLD_M) return clampDelta(lastDelta);
  const float thetaDes = std::atan2(yMp, ahead);
  return clampDelta(-STEER_KR * headingError(thetaDes, theta));
}

float steerLawStanleyReverse(float yMp, float theta, float speedAbs) {
  // Forward Stanley's heading term is KH * (0 - theta); here it is negated.
  const float eHead = headingError(0.0f, theta);
  const float delta = -STEER_KH * eHead -
                      std::atan2(STEER_KC * yMp, speedAbs + STEER_SPEED_EPS);
  return clampDelta(delta);
}

float steerLawPurePursuitReverse(float yRear, float theta, float lookahead, float wheelbase) {
  // Goal point on the line, Ld BEHIND the rear axle (world offset -dx, -yRear).
  // Out of reach (|y| > Ld): dx = 0, which aims square at the line.
  const float dx   = std::sqrt(std::max(lookahead * lookahead - yRear * yRear, 0.0f));
  // Its sideways offset in the car's frame (+ = left of the car's centreline).
  const float yCar = dx * std::sin(theta) - yRear * std::cos(theta);
  return clampDelta(std::atan(2.0f * wheelbase * yCar / (lookahead * lookahead)));
}
