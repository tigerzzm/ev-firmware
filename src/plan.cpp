#include "plan.h"
#include "config.h"
#include <algorithm>
#include <cmath>

VehicleGeom vehicleFromConfig() { return VehicleGeom{L_P_M}; }

float bottleXForLevel(int level) {
  switch (level) {
    case 1:  return BOTTLE_X_STATE_M;
    case 2:  return BOTTLE_X_NATIONAL_M;
    default: return BOTTLE_X_REGIONAL_M;
  }
}

float segmentTime(float s, float v0, float vc, float v1, float aUp, float aDown) {
  if (s <= 0.0f) return 0.0f;
  vc = std::max(vc, std::max(v0, v1));            // cruise is never below the ends
  const float sUp = (vc * vc - v0 * v0) / (2.0f * aUp);
  const float sDn = (vc * vc - v1 * v1) / (2.0f * aDown);
  if (sUp + sDn <= s) {                           // trapezoid: reaches cruise
    return (vc - v0) / aUp + (vc - v1) / aDown + (s - sUp - sDn) / vc;
  }
  // Triangle: accelerate to a peak vp, then decelerate. From
  //   (vp^2 - v0^2)/2aUp + (vp^2 - v1^2)/2aDown = s
  const float vp2 = (2.0f * aUp * aDown * s + aDown * v0 * v0 + aUp * v1 * v1) /
                    (aUp + aDown);
  const float vp = std::sqrt(std::max(vp2, 0.0f));
  return std::max(vp - v0, 0.0f) / aUp + std::max(vp - v1, 0.0f) / aDown;
}

float planMotionTime(const RunPlan &p, const SegmentSpeeds &v) {
  // Mirrors the profiles in speed.cpp segment by segment.
  float t = 0.0f;
  t += segmentTime(p.captureStart, 0.0f, v.vCruise, V_CAPTURE, A_ACCEL, A_DECEL);
  t += (p.pushStart - p.captureStart) / V_CAPTURE;
  t += segmentTime(p.xF - p.pushStart, V_CAPTURE, v.vPush, V_PUSH_END, A_PUSH, A_RELEASE);
  t += TURN_MS / 1000.0f;
  t += segmentTime(p.rev, 0.0f, v.vRev, V_REV_END, A_ACCEL, A_REV_STOP);
  return t;
}

// Speeds with cruise and reverse scaled by kCR and push by kP, each capped at its
// _MAX. Capture speed never scales — that one protects the bottle.
static SegmentSpeeds scaled(float kCR, float kP) {
  return SegmentSpeeds{std::min(V_CRUISE_NOM * kCR, V_CRUISE_MAX),
                       std::min(V_PUSH * kP, V_PUSH_MAX),
                       std::min(V_REV * kCR, V_REV_MAX)};
}

// Smallest k in [1, kMax] for which fits(k) holds, given fits(kMax) holds.
template <typename F> static float smallestFit(float kMax, F fits) {
  float lo = 1.0f, hi = kMax;
  for (int i = 0; i < PLAN_BISECT_ITERS; i++) {
    const float mid = 0.5f * (lo + hi);
    if (fits(mid)) hi = mid; else lo = mid;
  }
  return hi;
}

RunPlan planRun(const RunInputs &in, const VehicleGeom &veh) {
  RunPlan p{};
  p.in = in;
  p.valid = false;

  // ---- refuse to plan with unmeasured or illegal inputs ---------------------
  if (veh.lP <= 0.0f)                    { p.error = "L_P_M=0";         return p; }
  if (veh.lP > VEHICLE_LEN_MAX_M)        { p.error = "L_P > 80cm";      return p; }
  if (in.D < TARGET_DIST_MIN_M || in.D > TARGET_DIST_MAX_M) { p.error = "D out of range"; return p; }
  if (in.B < BOTTLE_LINE_MIN_M || in.B > BOTTLE_LINE_MAX_M) { p.error = "B out of range"; return p; }
  if (in.T < TARGET_TIME_MIN_S || in.T > TARGET_TIME_MAX_S) { p.error = "T out of range"; return p; }

  // ---- geometry ---------------------------------------------------------------
  p.xContact     = in.xBottle - R_BOTTLE_M - veh.lP;
  p.captureStart = std::max(p.xContact - CAPTURE_LEAD_M, 0.0f);
  p.pushStart    = p.xContact + SEAT_MARGIN_M;
  p.xF           = in.D + in.B + A_PAST_M + in.btlOfs - S_SLIDE_M - veh.lP;
  p.xFinal       = in.D + in.mpOfs;
  p.rev          = p.xF - p.xFinal;
  p.path         = p.xF + p.rev;

  if (p.xContact <= 0.0f)                { p.error = "bottle too close"; return p; }
  if (p.xF < p.pushStart + PUSH_MIN_M)   { p.error = "push too short";   return p; }
  if (p.rev < REV_MIN_M)                 { p.error = "rev too short";    return p; }

  // ---- speeds and timing ------------------------------------------------------
  // Nominal speeds if the run fits inside T. Otherwise speed up only as much as
  // needed, and in order of risk: cruise and reverse first (no bottle on board),
  // and the PUSH only if those alone can't make it — a tipped bottle costs ~420
  // pts, a second late costs 0.5.
  const float budget = in.T - T_BIAS_S;
  auto fitsAt = [&](float kCR, float kP) { return planMotionTime(p, scaled(kCR, kP)) <= budget; };
  const float kCRMax = std::max(V_CRUISE_MAX / V_CRUISE_NOM, V_REV_MAX / V_REV);
  const float kPMax  = V_PUSH_MAX / V_PUSH;
  float kCR = 1.0f, kP = 1.0f;
  if (!fitsAt(kCR, kP)) {
    if (fitsAt(kCRMax, 1.0f)) {
      kCR = smallestFit(kCRMax, [&](float k) { return fitsAt(k, 1.0f); });
    } else {
      kCR = kCRMax;
      kP  = fitsAt(kCRMax, kPMax) ? smallestFit(kPMax, [&](float k) { return fitsAt(kCRMax, k); })
                                  : kPMax;   // best effort; time is only 0.5 pt/s
    }
  }
  p.v = scaled(kCR, kP);
  p.tMotion  = planMotionTime(p, p.v) + T_BIAS_S;
  p.timeFits = p.tMotion <= in.T + PLAN_TIME_TOL_S;
  p.delayS   = std::max(in.T - p.tMotion, 0.0f);

  p.valid = true;
  p.error = "";
  return p;
}
