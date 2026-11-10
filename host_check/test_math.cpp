// Host unit tests for the hardware-independent code: run planning (plan.cpp),
// speed profiles + speed loop (speed.cpp), steering laws (steer_law.cpp) and the
// angle/filter utils. Compiled natively; no Pico headers needed.
//
// The steering and speed tests are CLOSED-LOOP simulations: a bicycle-model car
// or a first-order motor driven by the real control code. A sign error (like the
// 2026 cross-track bug) makes them diverge, so they fail instead of passing
// quietly the way an open-loop formula check would.
#include "config.h"
#include "odo_sim.h"
#include "odo_split.h"
#include "plan.h"
#include "pose_track.h"
#include "speed.h"
#include "steer_law.h"
#include "utils.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

static int fails = 0;
static void chk(const char *n, double got, double want, double tol) {
  const bool ok = std::fabs(got - want) <= tol;
  printf("%-36s got=%9.4f want=%9.4f %s\n", n, got, want, ok ? "ok" : "FAIL");
  if (!ok) fails++;
}
static void chkTrue(const char *n, bool cond) {
  printf("%-36s %s\n", n, cond ? "ok" : "FAIL");
  if (!cond) fails++;
}

static const VehicleGeom kCar{0.55f};   // a plausible measured L_P for the tests
static const float kWheelbase = 0.25f;  // bicycle-model wheelbase for the sims

//------------------------------------------------------------------------------
// plan
//------------------------------------------------------------------------------
static void testPlanGeometry() {
  printf("\n-- plan geometry --\n");
  // T 18: fits inside T at this geometry even with the START_SETTLE_S settle.
  const RunPlan p = planRun(RunInputs{7.0f, 2.0f, 18.0f, 2.0f, 0.0f, 0.0f}, kCar);
  chkTrue("valid", p.valid);
  chk("xContact = xb - r - L_P", p.xContact, 2.0 - R_BOTTLE_M - 0.55, 1e-5);
  chk("xF = D + B + A_PAST - S - L_P", p.xF, 7.0 + 2.0 + A_PAST_M - S_SLIDE_M - 0.55, 1e-5);
  chk("xFinal = D", p.xFinal, 7.0, 1e-6);
  chk("rev = xF - xFinal", p.rev, p.xF - p.xFinal, 1e-6);
  chk("delay + motion = T", p.delayS + p.tMotion, 18.0, 1e-3);
  chkTrue("delay >= settle", p.delayS >= START_SETTLE_S);
  // Spare time is driven slower, not waited out: D5 B1 T19 has ~5 s to spare, and
  // the minimum speeds can still use it all (at T20 they finish ~0.1 s early).
  const RunPlan slow = planRun(RunInputs{5.0f, 1.0f, 19.0f, 2.0f, 0.0f, 0.0f}, kCar);
  chk("spare time: delay = settle", slow.delayS, START_SETTLE_S, 1e-3);
  chk("spare time: delay + motion = T", slow.delayS + slow.tMotion, 19.0, 1e-3);
  chkTrue("spare time: slower than nominal", slow.v.vCruise < V_CRUISE_NOM && slow.v.vPush < V_PUSH);
  chk("catch = min(V_CAPTURE, cruise, push)", p.v.vCatch,
      std::min({V_CAPTURE, p.v.vCruise, p.v.vPush}), 1e-6);
  chkTrue("slow run catches no faster than it cruises", slow.v.vCatch <= slow.v.vCruise + 1e-6f);
  chkTrue("spare time: not below the minimums",
          slow.v.vCruise >= V_CRUISE_MIN - 1e-4f && slow.v.vPush >= V_PUSH_MIN - 1e-4f && slow.v.vRev >= V_REV_MIN - 1e-4f);

  // Offsets move only their own stop.
  const RunPlan q = planRun(RunInputs{7.0f, 2.0f, 18.0f, 2.0f, -0.03f, +0.02f}, kCar);
  chk("mpOfs moves xFinal only", q.xFinal - p.xFinal, -0.03, 1e-5);
  chk("btlOfs moves xF only", q.xF - p.xF, +0.02, 1e-5);
  RunInputs mIn{7.0f, 2.0f, 15.0f, 2.0f, 0.0f, 0.0f};
  mIn.margin = 0.02f;
  const RunPlan m = planRun(mIn, kCar);
  chk("BtlPast moves xF only", m.xF - p.xF, 0.02f - A_PAST_M, 1e-5);
  chk("BtlPast leaves xFinal", m.xFinal - p.xFinal, 0.0, 1e-6);
  mIn.margin = A_PAST_MAX_M + 0.01f;
  chkTrue("refuses BtlPast > max", !planRun(mIn, kCar).valid);

  // Refusals.
  chkTrue("refuses L_P = 0", !planRun(RunInputs{7, 2, 15, 2, 0, 0}, VehicleGeom{0.0f}).valid);
  chkTrue("refuses L_P > 80 cm", !planRun(RunInputs{7, 2, 15, 2, 0, 0}, VehicleGeom{0.81f}).valid);
  chkTrue("refuses D = 4.9", !planRun(RunInputs{4.9f, 2, 15, 2, 0, 0}, kCar).valid);
  chkTrue("refuses B = 3.1", !planRun(RunInputs{7, 3.1f, 15, 2, 0, 0}, kCar).valid);
  chkTrue("refuses rev < REV_MIN_M", !planRun(RunInputs{7, 1, 15, 2, 0.5f, -0.5f}, kCar).valid);

  // Every refusal message goes on the 16-character LCD line.
  const RunInputs bad[] = {{7, 2, 15, 2, 0, 0}, {4.9f, 2, 15, 2, 0, 0}, {7, 0.5f, 15, 2, 0, 0},
                           {7, 2, 25, 2, 0, 0}, {7, 1, 15, 2, 0.5f, -0.5f}, {5, 1, 15, 4, -0.5f, -0.5f}};
  const VehicleGeom geoms[] = {VehicleGeom{0.0f}, VehicleGeom{0.81f}, kCar};
  size_t longest = 0;
  for (auto &g : geoms)
    for (auto &in : bad) {
      const RunPlan q = planRun(in, g);
      if (!q.valid) longest = std::max(longest, std::strlen(q.error));
    }
  chkTrue("all error strings fit 16 chars", longest <= 16);
}

// Every legal (D, B, T, level) with the longest legal car must plan, reverse,
// and either fit inside T or say it doesn't.
static void testPlanRulesGrid() {
  printf("\n-- plan over the full rules grid --\n");
  int n = 0, bad = 0, late = 0;
  float minRev = 1e9f;
  for (float D = 5.0f; D <= 9.0f + 1e-3f; D += 0.25f)
    for (float B = 1.0f; B <= 3.0f + 1e-3f; B += 0.25f)
      for (float T = 10.0f; T <= 20.0f + 1e-3f; T += 2.5f)
        for (int lvl = 0; lvl < 3; lvl++) {
          const RunPlan p = planRun(RunInputs{D, B, T, bottleXForLevel(lvl), 0, 0},
                                    VehicleGeom{VEHICLE_LEN_MAX_M});
          n++;
          if (!p.valid || p.delayS < 0.0f || p.xContact <= 0.0f ||
              p.pushStart >= p.xF || p.xFinal >= p.xF) { bad++; continue; }
          minRev = std::min(minRev, p.rev);
          if (!p.timeFits) late++;
          else if (std::fabs(p.delayS + p.tMotion - T) > 1e-3f) bad++;
        }
  printf("%d plans, %d cannot finish inside T at the _MAX speeds\n", n, late);
  chk("bad plans", bad, 0, 0);
  chkTrue("reverse leg always > REV_MIN_M", minRev >= REV_MIN_M);

  // The rules' worst corner. Pushing 8 m at a bottle-safe speed can't fit 10 s;
  // the plan must still be valid, use max cruise, and flag it.
  const RunPlan w = planRun(RunInputs{9, 3, 10, BOTTLE_X_REGIONAL_M, 0, 0}, kCar);
  printf("worst case D9 B3 T10: path %.2f m, motion %.2f s, fits=%d\n", w.path, w.tMotion, w.timeFits);
  chkTrue("worst case valid", w.valid);
  chk("worst case delay = settle", w.delayS, START_SETTLE_S, 1e-6);
}

static void testSegmentTime() {
  printf("\n-- segmentTime --\n");
  chk("trapezoid 10 m, vc 1, a 1", segmentTime(10, 0, 1, 0, 1, 1), 11.0, 1e-5);
  chk("triangle 1 m, a 1", segmentTime(1, 0, 5, 0, 1, 1), 2.0, 1e-5);
  chk("cruise only 2 m @ 0.5", segmentTime(2, 0.5f, 0.5f, 0.5f, 1, 1), 4.0, 1e-5);
}

// The plan's time model must match what the state machine actually commands.
// Simulate a car that tracks vRef perfectly through the same profiles,
// transitions and rate limits as sm.cpp, and compare the elapsed time.
static void testTimeModelMatchesProfiles() {
  printf("\n-- time model vs simulated profile --\n");
  const float cases[][4] = {{7, 2, 15, 2}, {9, 3, 20, 4}, {5, 1, 12, 3}};
  for (auto &c : cases) {
    const RunPlan p = planRun(RunInputs{c[0], c[1], c[2], c[3], 0, 0}, kCar);
    const float dt = 0.001f;
    float x = 0, vRef = 0, t = 0;
    // forward legs
    while (x < p.xF) {
      const float aUp = (x >= p.pushStart) ? A_PUSH : A_ACCEL;
      vRef = speedRateLimit(speedProfileForward(x, p), vRef, aUp, dt);
      x += vRef * dt; t += dt;
    }
    t += TURN_MS / 1000.0f;
    vRef = 0;
    while (x > p.xFinal) {
      vRef = speedRateLimit(speedProfileReverse(x, p), vRef, A_ACCEL, dt);
      x -= vRef * dt; t += dt;
    }
    char name[48];
    snprintf(name, sizeof name, "motion time D%.0f B%.0f (%%)", c[0], c[1]);
    const float model = p.tMotion - T_BIAS_S;
    chk(name, 100.0 * (t - model) / model, 0.0, 2.0);
  }
}

//------------------------------------------------------------------------------
// speed loop
//------------------------------------------------------------------------------
static void testSpeedLoop() {
  printf("\n-- speed loop (closed loop, mismatched plant) --\n");
  // Plant deliberately DIFFERENT from the feedforward: 12 % dead zone and
  // 50 %/(m/s) slope vs the FF's 5 % + 40 %/(m/s). The PI must close the gap.
  auto plantSS = [](float pwm) { return std::max(pwm - 12.0f, 0.0f) / 50.0f; };
  for (float target : {0.06f, 0.20f, 0.45f}) {
    SpeedLoop loop; loop.reset();
    float v = 0, minPwm = 1e9f, maxPwm = -1e9f;
    const float dt = 0.002f, tau = 0.15f;
    for (int i = 0; i < 4000; i++) {                    // 8 s
      const float pwm = loop.update(target, v, dt);
      minPwm = std::min(minPwm, pwm); maxPwm = std::max(maxPwm, pwm);
      v += (plantSS(pwm) - v) / tau * dt;
    }
    char name[48];
    snprintf(name, sizeof name, "settles to %.2f m/s", target);
    chk(name, v, target, 0.01 * target + 1e-3);
    chkTrue("  pwm stays in [0, PWM_MAX]", minPwm >= 0.0f && maxPwm <= PWM_MAX_PCT);
  }
  SpeedLoop loop; loop.reset();
  chk("zero target -> zero pwm", loop.update(0.0f, 0.3f, 0.002f), 0.0, 0.0);
  chk("overspeed never reverses", loop.update(0.1f, 2.0f, 0.002f), 0.0, 0.0);

  // Feedforward table: hits every point, interpolates between, extends the end
  // segments, and the inverse undoes it.
  {
    bool pts = true, inv = true, mono = true;
    for (int i = 0; i < SPEED_FF_N; i++) {
      const FfPoint &p = SPEED_FF_TABLE[i];
      if (p.v > 0.0f && std::fabs(pwmFeedforward(p.v) - p.pct) > 1e-4f) pts = false;
      if (i > 0 && std::fabs(ffSpeedForPwm(p.pct) - p.v) > 1e-4f) inv = false;
    }
    float last = 0.0f;
    for (float v = 0.01f; v < 2.5f; v += 0.01f) {
      const float pct = pwmFeedforward(v);
      if (pct <= last) mono = false;
      last = pct;
      if (std::fabs(ffSpeedForPwm(pct) - v) > 1e-3f) inv = false;
    }
    chkTrue("ff table: every point exact", pts);
    chkTrue("ff table: increasing over 0-2.5 m/s", mono);
    chkTrue("ff table: inverse undoes it", inv);
    const FfPoint &a = SPEED_FF_TABLE[2], &b = SPEED_FF_TABLE[3];
    chk("ff table: midpoint interpolates", pwmFeedforward(0.5f * (a.v + b.v)), 0.5f * (a.pct + b.pct), 1e-4);
    chk("ff: no motion at the dead-zone edge", ffSpeedForPwm(SPEED_FF_TABLE[0].pct), 0.0, 0.0);
  }

  // Frozen integrator (car stuck on stiction): the I term must hold still, so the
  // breakaway doesn't come with a wound-up surge. Output with a frozen integrator
  // stays exactly at feedforward + P; unfrozen, the same error grows it.
  {
    SpeedLoop a, b; a.reset(); b.reset();
    float outA = 0, outB = 0;
    for (int i = 0; i < 500; i++) {                    // 1 s stuck at v = 0
      outA = a.update(0.1f, 0.0f, 0.002f, /*freezeIntegrator=*/true);
      outB = b.update(0.1f, 0.0f, 0.002f, /*freezeIntegrator=*/false);
    }
    chk("frozen integrator holds (pwm = ff + P)", outA, pwmFeedforward(0.1f) + SPEED_KP * 0.1f, 1e-4);
    chkTrue("unfrozen integrator winds up", outB > outA + 1.0f);
  }
}

//------------------------------------------------------------------------------
// steering — closed loop in a kinematic bicycle model (rear-axle reference)
//------------------------------------------------------------------------------
static void testSteerForward() {
  printf("\n-- forward steering, Stanley (closed loop) --\n");
  struct Start { float y, th; } starts[] = {{0.10f, 0.0f}, {-0.10f, 0.0f}, {0.0f, 0.08f}, {0.05f, -0.06f}};
  for (auto s : starts) {
    // State at the rear axle; the law sees the FRONT axle (where the roller is).
    float yr = s.y - kWheelbase * std::sin(s.th), th = s.th;
    const float v = 0.5f, dt = 0.002f;
    float maxAbsY = 0;
    for (float d = 0; d < 5.0f; d += v * dt) {
      const float yf = yr + kWheelbase * std::sin(th);
      const float delta = steerLawStanley(yf, th, v);
      th += v * std::tan(delta) / kWheelbase * dt;
      yr += v * std::sin(th) * dt;
      maxAbsY = std::max(maxAbsY, std::fabs(yf));
    }
    const float yf = yr + kWheelbase * std::sin(th);
    char name[48];
    snprintf(name, sizeof name, "y0=%+.2f th0=%+.2f -> |y| @5m", s.y, s.th);
    chk(name, std::fabs(yf), 0.0, 0.005);
    chkTrue("  heading settles", std::fabs(th) < 0.01f);
    chkTrue("  never strays > 0.25 m", maxAbsY < 0.25f);
  }
  // Static sign checks, same words as the bench test.
  chkTrue("car LEFT of line -> steer RIGHT", steerLawStanley(0.05f, 0.0f, 0.5f) < 0.0f);
  chkTrue("car pointing LEFT -> steer RIGHT", steerLawStanley(0.0f, 0.05f, 0.5f) < 0.0f);
  chkTrue("clamped to DELTA_MAX", std::fabs(steerLawStanley(5.0f, 1.0f, 0.5f)) <= DELTA_LIMIT_RAD + 1e-6f);
}

static void testSteerPurePursuit() {
  printf("\n-- forward steering, pure pursuit (closed loop) --\n");
  const float Ld = PP_LOOKAHEAD_M;
  struct Start { float y, th; } starts[] = {{0.10f, 0.0f}, {-0.10f, 0.0f}, {0.0f, 0.08f}, {0.05f, -0.06f}};
  for (auto s : starts) {
    // Pure pursuit works from the rear axle, which is also the model's reference.
    float yr = s.y, th = s.th, maxAbsY = 0;
    const float v = 0.5f, dt = 0.002f;
    for (float d = 0; d < 5.0f; d += v * dt) {
      const float delta = steerLawPurePursuit(yr, th, Ld, kWheelbase);
      th += v * std::tan(delta) / kWheelbase * dt;
      yr += v * std::sin(th) * dt;
      maxAbsY = std::max(maxAbsY, std::fabs(yr));
    }
    char name[48];
    snprintf(name, sizeof name, "y0=%+.2f th0=%+.2f -> |y| @5m", s.y, s.th);
    chk(name, std::fabs(yr), 0.0, 0.005);
    chkTrue("  heading settles", std::fabs(th) < 0.01f);
    chkTrue("  never strays > 0.25 m", maxAbsY < 0.25f);
  }
  // A 1 deg start error: the design claim is a peak of ~5 mm at Ld = 0.8 m.
  {
    float yr = 0, th = 0.01745f, peak = 0;
    for (float d = 0; d < 5.0f; d += 0.001f) {
      const float delta = steerLawPurePursuit(yr, th, Ld, kWheelbase);
      th += std::tan(delta) / kWheelbase * 0.001f;
      yr += std::sin(th) * 0.001f;
      peak = std::max(peak, std::fabs(yr));
    }
    printf("1 deg start error: peak sideways %.1f mm (Ld %.2f m)\n", 1000 * peak, Ld);
    chkTrue("  1 deg start error stays under 1 cm", peak < 0.01f);
  }
  chkTrue("car LEFT of line -> steer RIGHT", steerLawPurePursuit(0.05f, 0.0f, Ld, kWheelbase) < 0.0f);
  chkTrue("car pointing LEFT -> steer RIGHT", steerLawPurePursuit(0.0f, 0.05f, Ld, kWheelbase) < 0.0f);
  chk("far off the line -> full lock toward it",
      steerLawPurePursuit(2.0f, 0.0f, Ld, kWheelbase), -DELTA_LIMIT_RAD, 1e-6);

  // steerLawForward must dispatch to whichever law config.h selects.
  const float yR = 0.03f, th = 0.02f;
  const float want = (STEER_FWD_LAW == SteerLaw::PurePursuit)
      ? steerLawPurePursuit(yR - WHEELBASE_L_M * std::sin(th), th, PP_LOOKAHEAD_M, WHEELBASE_L_M)
      : steerLawStanley(yR, th, 0.5f);
  chk("steerLawForward uses the selected law", steerLawForward(yR, th, 0.5f), want, 1e-7);
}

static void testSteerReverse() {
  printf("\n-- reverse steering, aim-at-target (closed loop) --\n");
  struct Start { float y, th; } starts[] = {{0.06f, 0.0f}, {-0.06f, 0.0f}, {0.0f, 0.05f}, {0.04f, 0.04f}};
  const float D = 7.0f;
  // d = how far the MP sits behind the rear axle. The real MP is the rearmost
  // point of the car, so d > 0 is the realistic case; d = 0 is the simplest.
  for (float d : {0.0f, 0.10f}) {
    for (auto s : starts) {
      // Rear axle is the model's reference. Back up from 1.5 m past the Target.
      float x = D + 1.5f, y = s.y, th = s.th, delta = 0;
      const float v = -0.3f, dt = 0.002f;
      auto xMp = [&] { return x - d * std::cos(th); };
      auto yMp = [&] { return y - d * std::sin(th); };
      while (xMp() > D) {
        delta = steerLawAimAtTarget(xMp(), yMp(), th, D, delta);
        th += v * std::tan(delta) / kWheelbase * dt;
        x  += v * std::cos(th) * dt;
        y  += v * std::sin(th) * dt;
      }
      char name[56];
      snprintf(name, sizeof name, "d=%.2f y0=%+.2f th0=%+.2f -> |y_MP| @ D", d, s.y, s.th);
      chk(name, std::fabs(yMp()), 0.0, 0.015);
    }
  }
  chkTrue("pointing LEFT on line -> wheels LEFT", steerLawAimAtTarget(8.0f, 0.0f, 0.05f, 7.0f, 0.0f) > 0.0f);
  chk("holds inside REV_STEER_HOLD_M", steerLawAimAtTarget(7.05f, 0.03f, 0.2f, 7.0f, 0.033f), 0.033, 1e-6);
}

static void testSteerPurePursuitReverse() {
  printf("\n-- reverse steering, pure pursuit (closed loop) --\n");
  const float Ld = PP_LOOKAHEAD_M, D = 7.0f;
  struct Start { float y, th; } starts[] = {{0.06f, 0.0f}, {-0.06f, 0.0f}, {0.0f, 0.05f}, {0.04f, 0.04f}};
  // The law steers the REAR AXLE onto the line, but the scored point is the rear
  // MP, d behind it: a heading left over at the stop puts the MP d*sin(theta)
  // off the line. So judge it at the MP, for d = 0 and a realistic 0.10 m.
  for (float d : {0.0f, 0.10f}) {
    for (auto s : starts) {
      float x = D + 1.5f, y = s.y, th = s.th;  // rear axle; back up past the Target
      const float v = -0.3f, dt = 0.002f;
      auto xMp = [&] { return x - d * std::cos(th); };
      auto yMp = [&] { return y - d * std::sin(th); };
      while (xMp() > D) {
        const float delta = steerLawPurePursuitReverse(y, th, Ld, kWheelbase);
        th += v * std::tan(delta) / kWheelbase * dt;
        x  += v * std::cos(th) * dt;
        y  += v * std::sin(th) * dt;
      }
      char name[56];
      snprintf(name, sizeof name, "d=%.2f y0=%+.2f th0=%+.2f -> |y_MP| @ D", d, s.y, s.th);
      chk(name, std::fabs(yMp()), 0.0, 0.015);
    }
  }
  chkTrue("pointing LEFT on line -> wheels LEFT", steerLawPurePursuitReverse(0.0f, 0.05f, Ld, kWheelbase) > 0.0f);
  chkTrue("LEFT of line -> wheels RIGHT", steerLawPurePursuitReverse(0.05f, 0.0f, Ld, kWheelbase) < 0.0f);
  chk("far off the line -> full lock toward it",
      steerLawPurePursuitReverse(2.0f, 0.0f, Ld, kWheelbase), -DELTA_LIMIT_RAD, 1e-6);

}

static void testSteerStanleyReverse() {
  printf("\n-- reverse steering, Stanley (closed loop) --\n");
  const float D = 7.0f;
  struct Start { float y, th; } starts[] = {{0.06f, 0.0f}, {-0.06f, 0.0f}, {0.0f, 0.05f}, {0.04f, 0.04f}};
  // d = how far the reference point (the rear MP) sits behind the rear axle. The
  // law is more stable the further back it is; d = 0 is the hardest case.
  for (float d : {0.0f, 0.10f}) {
    for (auto s : starts) {
      float x = D + 1.5f, y = s.y, th = s.th;   // rear axle (model reference)
      const float v = -0.3f, dt = 0.002f;
      float yRef = y;
      while (x - d * std::cos(th) > D) {
        yRef = y - d * std::sin(th);
        const float delta = steerLawStanleyReverse(yRef, th, std::fabs(v));
        th += v * std::tan(delta) / kWheelbase * dt;
        x  += v * std::cos(th) * dt;
        y  += v * std::sin(th) * dt;
      }
      yRef = y - d * std::sin(th);
      char name[56];
      snprintf(name, sizeof name, "d=%.2f y0=%+.2f th0=%+.2f -> |y_MP| @ D", d, s.y, s.th);
      chk(name, std::fabs(yRef), 0.0, 0.015);
    }
  }
  chkTrue("pointing LEFT on line -> wheels LEFT", steerLawStanleyReverse(0.0f, 0.05f, 0.3f) > 0.0f);
  chkTrue("LEFT of line -> wheels RIGHT", steerLawStanleyReverse(0.05f, 0.0f, 0.3f) < 0.0f);
  chk("far off the line -> full lock toward it", steerLawStanleyReverse(2.0f, 0.0f, 0.3f), -DELTA_LIMIT_RAD, 1e-6);
  // Forward Stanley run backwards must NOT be what this computes: on the line,
  // pointing left, forward Stanley steers right — the wrong way in reverse.
  chkTrue("differs from forward Stanley", steerLawStanley(0.0f, 0.05f, 0.3f) < 0.0f);
}

// steerLawReverse must dispatch to whichever law config.h selects, working out the
// rear axle / rear MP from the roller pose.
static void testReverseDispatch() {
  printf("\n-- reverse law selection --\n");
  const float xR = 8.0f, yR = 0.03f, thR = 0.02f, v = 0.3f, last = 0.01f;
  const float yAxle = yR - WHEELBASE_L_M * std::sin(thR);
  const float xMp = xR - ENC_FROM_MP_M * std::cos(thR), yMp = yR - ENC_FROM_MP_M * std::sin(thR);
  float want;
  switch (STEER_REV_LAW) {
    case RevSteerLaw::PurePursuit:
      want = steerLawPurePursuitReverse(yAxle, thR, PP_LOOKAHEAD_M, WHEELBASE_L_M); break;
    case RevSteerLaw::Stanley:
      want = steerLawStanleyReverse(yMp, thR, v); break;
    default:
      want = steerLawAimAtTarget(xMp, yMp, thR, 7.0f, last); break;
  }
  chk("steerLawReverse uses the selected law", steerLawReverse(xR, yR, thR, v, 7.0f, last), want, 1e-7);
}

//------------------------------------------------------------------------------
// the whole lateral loop with a MIS-CALIBRATED servo: the estimator (pose_track.h)
// sees only encoder distance and IMU heading, the real wheels point at
// scale * command + bias, and the selected laws steer from the estimate. If the
// estimator trusted the servo command, a 50 us centre error would walk the car
// ~13 cm off the line over 9 m; this checks it stays within ~1-2 cm.
//------------------------------------------------------------------------------
// Aim trim: the car is placed pointing `a` RIGHT of the real centreline. With the
// trim set to `a`, the pose starts at heading -a and the forward law steers onto
// the real line; without it the car holds the wrong heading and ends ~D*a right.
static double aimRun(double aimTrue, double trim) {
  const double Lm = WHEELBASE_L_M > 0 ? WHEELBASE_L_M : 0.40, WL = WHEELBASE_L_M;
  // True state of the rear axle, real-centreline frame; rear MP on the Start dot.
  const double dRear = ENC_FROM_MP_M - Lm;
  double th = -aimTrue, xr = dRear * std::cos(th), yr = dRear * std::sin(th);
  HeadingTrack est;
  const double th0 = -trim;
  est.reset(ENC_FROM_MP_M * std::cos(th0), ENC_FROM_MP_M * std::sin(th0), WL, th0);
  const double v = 0.4, dt = 0.002;
  while (xr - dRear < 5.0) {
    const double thEst = th + aimTrue - trim;            // IMU zeroed at the start, minus trim
    const float cmd = steerLawForward((float)est.rollerY(thEst, WL), (float)thEst, (float)v);
    const double ds = v * dt;
    th += ds * std::sin(cmd) / Lm;
    xr += ds * std::cos(cmd) * std::cos(th);
    yr += ds * std::cos(cmd) * std::sin(th);
    est.step(ds * std::cos(cmd), th + aimTrue - trim);
  }
  return yr - dRear * std::sin(th);                      // rear MP sideways, real frame
}

static void testAimTrim() {
  printf("\n-- aim trim --\n");
  const double a = 0.55 * M_PI / 180.0;
  const double off = aimRun(a, 0.0), on = aimRun(a, a);
  printf("0.55 deg aim error over 5 m: rear MP %.1f cm without the trim, %.1f cm with it\n",
         100 * off, 100 * on);
  chkTrue("without the trim it ends right (> 3 cm)", off < -0.03);
  chkTrue("with the trim it ends within 1 cm", std::fabs(on) < 0.01);
  HeadingTrack t;
  t.reset(0.5 * std::cos(-a), 0.5 * std::sin(-a), 0.4, -a);
  chk("reset with heading: roller x round-trips", t.rollerX(-a, 0.4), 0.5 * std::cos(-a), 1e-9);
  chk("reset with heading: roller y round-trips", t.rollerY(-a, 0.4), 0.5 * std::sin(-a), 1e-9);
}

static void testMiscalibratedServo() {
  printf("\n-- whole loop, mis-calibrated servo (estimator never sees the command) --\n");
  // The car's real wheelbase. The estimator uses WHEELBASE_L_M, which may still be
  // 0 (unmeasured): that must cost only a few mm.
  const double Lm = WHEELBASE_L_M > 0.0f ? WHEELBASE_L_M : 0.44;
  const double WL = WHEELBASE_L_M;
  const double bias = 50.0 / 3565.0;           // centre pulse 50 us off, rad of beam
  for (double scale : {0.77, 1.0, 1.3}) {      // SERVO_US_PER_RAD 30% high / right / 23% low
    // Forward 9 m. True state at the rear axle; the roller starts at ENC_FROM_MP_M.
    double xr = ENC_FROM_MP_M - Lm, yr = 0, th = 0;
    HeadingTrack est;
    est.reset(ENC_FROM_MP_M, WL);
    const double v = 0.6, dt = 0.002;
    double worstEst = 0, yAt4 = 0;
    while (xr + Lm < ENC_FROM_MP_M + 9.0) {
      const float cmd = steerLawForward((float)est.rollerY(th, WL), (float)th, (float)v);
      const double d = scale * cmd + bias;     // what the wheels really do
      const double ds = v * dt;                // roller (front axle) step
      th += ds * std::sin(d) / Lm;
      xr += ds * std::cos(d) * std::cos(th);
      yr += ds * std::cos(d) * std::sin(th);
      est.step(ds, th);
      const double yTrue = yr + Lm * std::sin(th);
      worstEst = std::max(worstEst, std::fabs(est.rollerY(th, WL) - yTrue));
      if (yAt4 == 0 && xr + Lm >= ENC_FROM_MP_M + 4.0) yAt4 = yTrue;
    }
    char name[64];
    snprintf(name, sizeof name, "scale %.2f: |y| at bottle (4 m)", scale);
    chk(name, std::fabs(yAt4), 0.0, 0.02);
    snprintf(name, sizeof name, "scale %.2f: |y| at 9 m", scale);
    chk(name, std::fabs(yr + Lm * std::sin(th)), 0.0, 0.02);
    snprintf(name, sizeof name, "scale %.2f: estimate error (worst)", scale);
    chk(name, worstEst, 0.0, 0.01);

    // Reverse 2 m onto the Target, carrying on from the forward leg's end state.
    // Skipped while ENC_FROM_MP_M is 0: the rear MP would then sit on the front
    // axle, which no reverse law can hold (run.sh re-runs with a measured car).
    if (ENC_FROM_MP_M <= Lm) continue;
    const double mpBehind = ENC_FROM_MP_M - Lm;            // rear MP behind the rear axle
    const double target = est.rollerX(th, WL) - ENC_FROM_MP_M - 2.0;
    float last = 0;
    while (est.rollerX(th, WL) - ENC_FROM_MP_M * std::cos(th) > target) {
      const float cmd = steerLawReverse((float)est.rollerX(th, WL), (float)est.rollerY(th, WL),
                                        (float)th, (float)V_REV, (float)target, last);
      last = cmd;
      const double d = scale * cmd + bias;
      const double ds = -V_REV * dt;
      th += ds * std::sin(d) / Lm;
      xr += ds * std::cos(d) * std::cos(th);
      yr += ds * std::cos(d) * std::sin(th);
      est.step(ds, th);
    }
    snprintf(name, sizeof name, "scale %.2f: |y| rear MP at Target", scale);
    chk(name, std::fabs(yr - mpBehind * std::sin(th)), 0.0, 0.02);
  }
}

//------------------------------------------------------------------------------
// bench dry run: the simulated roller (odo_sim.h)
//------------------------------------------------------------------------------
static void testSimWheel() {
  printf("\n-- dry-run simulated roller --\n");
  const float mF = 1.0e-4f, mR = 1.1e-4f, dt = 0.002f;
  auto run = [&](SimWheel &w, float secs, DriveMode m, float pct) {
    for (float t = 0; t < secs; t += dt) w.step(dt, m, pct, mF, mR);
  };
  // Steady drive settles on the feedforward line, scaled by SIM_SPEED_SCALE.
  const float vWant = 0.5f;
  const float pct = pwmFeedforward(vWant);
  SimWheel a;
  run(a, 2.0f, DriveMode::Drive, pct);
  chk("drive: settles on the ff line x scale", a.v, SIM_SPEED_SCALE * vWant, 1e-3);
  chkTrue("drive: counts roll forward", a.counts > 0);
  // Below the dead-zone edge nothing moves.
  SimWheel b;
  run(b, 1.0f, DriveMode::Drive, 0.5f * SPEED_FF_TABLE[0].pct);
  chk("below ff offset: stays put", b.counts, 0.0, 0.0);
  // Brake stops sooner than coast, and both come to rest (no sign flip).
  SimWheel h = a, c = a;
  float tH = 0, tC = 0;
  while (h.v > 0 && tH < 5) { h.step(dt, DriveMode::Hold, 0, mF, mR); tH += dt; }
  while (c.v > 0 && tC < 5) { c.step(dt, DriveMode::Coast, 0, mF, mR); tC += dt; }
  chkTrue("hold stops faster than coast", tH < tC);
  chk("hold stops at v = 0", h.v, 0.0, 0.0);
  chk("hold time ~ v / SIM_BRAKE_DECEL", tH, a.v / SIM_BRAKE_DECEL_MPS2, 2 * dt);
  // Reverse runs backward on the reverse scale.
  SimWheel r;
  run(r, 2.0f, DriveMode::Drive, -pct);
  chk("reverse: negative speed", r.v, -SIM_SPEED_SCALE * vWant, 1e-3);
  {
    SimWheel q = r;
    const double before = q.counts;
    q.step(dt, DriveMode::Drive, -pct, mF, mR);
    chk("reverse: counts use M_PER_COUNT_REV", q.counts - before, q.v * dt / mR, 1e-6);
  }
  // A long gap is capped, not integrated in one jump.
  SimWheel g = a;
  const double g0 = g.counts;
  g.step(10.0f, DriveMode::Drive, pct, mF, mR);
  chkTrue("gap capped at SIM_MAX_STEP_S", g.counts - g0 <= g.v * SIM_MAX_STEP_S / mF + 1);
  // The real speed loop closes on it: the dry run must be drivable, not just
  // a model that happens to sit on the feedforward line.
  {
    SimWheel w;
    SpeedLoop loop;
    loop.reset();
    for (float t = 0; t < 4.0f; t += dt) {
      const float pwm = std::max(loop.update(0.4f, w.v, dt), 0.0f);
      w.step(dt, pwm > 0 ? DriveMode::Drive : DriveMode::Coast, pwm, mF, mR);
    }
    chk("speed loop holds 0.4 m/s on the sim", w.v, 0.4, 0.01);
  }
}

//------------------------------------------------------------------------------
// encoder direction split (odo_split.h)
//------------------------------------------------------------------------------
static void testDirSplit() {
  printf("\n-- encoder direction split --\n");
  const int32_t H = ODO_DIR_HYST_COUNTS;
  const float mF = 1.00e-4f, mR = 1.05e-4f;   // deliberately very different scales

  // Edge jitter at a standstill must NOT be scaled twice.
  DirSplit a;
  a.add(1000, H);
  const float before = a.metres(mF, mR);
  for (int i = 0; i < 5000; i++) { a.add(-1, H); a.add(+1, H); }
  chk("jitter: net unchanged", a.net(), 1000, 0);
  chk("jitter: distance unchanged (um)", 1e6 * (a.metres(mF, mR) - before), 0.0, 0.01);
  chk("jitter: no reverse counts", a.rev, 0, 0);

  // A real reversal is split correctly and the net stays exact.
  DirSplit b;
  b.add(1000, H);
  for (int i = 0; i < 500; i++) b.add(-1, H);  // one count at a time, like slow travel
  chk("reversal: fwd", b.fwd, 1000, 0);
  chk("reversal: rev", b.rev, -500, 0);
  chk("reversal: nothing pending", b.pending, 0, 0);
  chk("reversal: metres", b.metres(mF, mR), 1000 * mF - 500 * mR, 1e-7);

  // Big samples (fast travel) and going back and forth.
  DirSplit c;
  c.add(300, H); c.add(-120, H); c.add(50, H); c.add(-5, H);
  chk("mixed: net exact", c.net(), 300 - 120 + 50 - 5, 0);
  chk("mixed: fwd + rev + pending = net", c.fwd + c.rev + c.pending, c.net(), 0);

  // Backing up from zero: pending counts below the hysteresis still count, at the
  // reverse scale.
  DirSplit d;
  d.add(-3, H);
  chk("small reverse from zero: net", d.net(), -3, 0);
  chk("small reverse from zero: metres", d.metres(mF, mR), -3 * mR, 1e-9);
}

//------------------------------------------------------------------------------
// utils
//------------------------------------------------------------------------------
static void testUtils() {
  printf("\n-- utils --\n");
  chk("angleError(350,10)", utils::angleError(350, 10, false), -20.0, 1e-6);
  chk("angleSquish(-90)",   utils::angleSquish(-90, false), 270.0, 1e-6);
  chk("slew clamps",        utils::slew(100, 0, 5), 5.0, 1e-6);
  chk("lowpass dt=0 holds",  utils::lowpass(1.0f, 9.0f, 0.0f, 0.1f), 1.0, 1e-6);
  chk("lowpass tau=0 passes",utils::lowpass(1.0f, 9.0f, 0.02f, 0.0f), 9.0, 1e-6);
  chk("lowpass dt==tau",     utils::lowpass(0.0f, 1.0f, 0.1f, 0.1f), 0.5, 1e-6);
  // A step held for 5 tau must settle close to the new value and never overshoot
  // it (a first-order lag has no overshoot). The discrete alpha = dt/(tau+dt) is
  // marginally slower than the continuous e^(-t/tau), so 5 tau lands at ~98.9%.
  {
    float y = 0.0f; const float dt = 0.02f, tau = SPEED_FILTER_TAU_S;
    bool overshot = false;
    for (int i = 0; i < (int)(5.0f * tau / dt); i++) {
      y = utils::lowpass(y, 1.0f, dt, tau);
      if (y > 1.0f) overshot = true;
    }
    chk("lowpass settles 5tau", y, 1.0, 0.02);
    chk("lowpass no overshoot", overshot ? 1 : 0, 0.0, 0.0);
  }
  // Constant-speed ramp: differentiating a straight distance ramp and filtering it
  // must converge on the true speed (exactly what odoSpeedTick does).
  const float trueV = 1.25f, dt = 0.02f;
  float v = 0.0f, dist = 0.0f, last = 0.0f;
  for (int i = 0; i < 100; i++) {
    dist += trueV * dt;
    v = utils::lowpass(v, (dist - last) / dt, dt, SPEED_FILTER_TAU_S);
    last = dist;
  }
  chk("speed est tracks ramp", v, trueV, 1e-3);
}

int main() {
  testPlanGeometry();
  testPlanRulesGrid();
  testSegmentTime();
  testTimeModelMatchesProfiles();
  testSpeedLoop();
  testSteerForward();
  testSteerPurePursuit();
  testSteerReverse();
  testSteerPurePursuitReverse();
  testSteerStanleyReverse();
  testReverseDispatch();
  testMiscalibratedServo();
  testAimTrim();
  testSimWheel();
  testDirSplit();
  testUtils();
  printf("\n%s (%d failures)\n", fails ? "TESTS FAILED" : "ALL MATH TESTS PASS", fails);
  return fails ? 1 : 0;
}
