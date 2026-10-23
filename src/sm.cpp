#include "sm.h"
#include "cal.h"
#include "config.h"
#include "drive.h"
#include "imu.h"
#include "odo.h"
#include "plan.h"
#include "pose.h"
#include "speed.h"
#include "steer.h"
#include "steer_law.h"
#include "ui.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

// The estimator (poseUpdate) runs on core1 — see main.cpp. Here on core0 we
// sequence the run and issue drive/steer commands.
//
// NOTHING on the motion path prints or writes the LCD. stdio over USB CDC can
// block for up to 500 ms when the host isn't reading, and an LCD write is ~30 ms
// of blocking I2C — either one in the control loop would delay a stop. Events go
// into a RAM log instead and are printed after the car has stopped.
//
// Nothing on the motion path waits on core1 either. Distance and speed come from
// the encoder on THIS core; only the steering uses the pose, and if the IMU (and
// with it core1) stalls, the run degrades to encoder-only rather than stopping.

// Why a run ended early. Any abort brakes and holds; it never skips the brake.
// Names fit "ABORT " + name on the 16-character LCD.
enum class Abort { None, Operator, Timeout, NoEncoder, Stall, WrongWay };

static const char *abortName(Abort a) {
  switch (a) {
    case Abort::Operator:  return "by SET";
    case Abort::Timeout:   return "timeout";
    case Abort::NoEncoder: return "no motion";   // dead encoder, or car stuck
    case Abort::Stall:     return "stall";
    case Abort::WrongWay:  return "wrong way";
    default:               return "none";
  }
}

static const char *stateName(EvState s) {
  switch (s) {
    case EvState::CRUISE:     return "CRUISE";
    case EvState::CAPTURE:    return "CAPTURE";
    case EvState::PUSH:       return "PUSH";
    case EvState::TURNAROUND: return "TURN";
    case EvState::REVERSE:    return "REVERSE";
    case EvState::STOP:       return "STOP";
  }
  return "?";
}

static uint32_t nowMs() { return to_ms_since_boot(get_absolute_time()); }

// ---- RAM event log ---------------------------------------------------------------
// One entry per state change (plus IMU events). Printed after the run so tuning
// has a timeline without any I/O in the control loop.
struct LogEntry { uint32_t ms; const char *what; float x; float v; float pwm; };
static LogEntry s_log[RUN_LOG_CAPACITY];
static int      s_logN = 0;

static void logEvent(uint32_t ms, const char *what, float x, float v, float pwm) {
  if (s_logN < RUN_LOG_CAPACITY) s_log[s_logN++] = LogEntry{ms, what, x, v, pwm};
}

struct RunResult {
  bool  cancelled;     // backed out of ARMED: no run happened
  bool  drove;         // the motion phase began (the motor was driven)
  bool  moved;         // encoder saw at least MOVED_MIN_M of travel
  Abort abort;
  bool  imuLost;       // steering ran without a usable heading (centred) for part of the motion
  float xMax;          // furthest forward MP position = actual forward stop
  float xEnd;          // final MP position after the hold
  float runTime;       // START -> stopped, s (what the timers measure)
  float motionTime;    // first motion -> stopped, s
};

// Everything in config.h §1 that must be MEASURED before the car may move. A zero
// here means a distance that reads 0, a plan built on a guess, or steering on an
// unknown servo scale — refuse rather than run. Checked BEFORE target entry, so
// you find out without typing in D, B and T first. (planRun() checks L_P_M again
// for its own callers; the message is the same.)
static const char *preflightError() { return calibrationMissing(); }

// Waits for a fresh SET press. Returns true for a long press (>= CAL_ENTRY_HOLD_MS).
static bool waitSetPress() {
  while (uiSetPressed()) sleep_ms(5);      // ignore a press still held
  while (!uiSetPressed()) sleep_ms(2);     // wait for a fresh press
  const uint32_t tDown = nowMs();
  bool longPress = false;
  while (uiSetPressed()) {
    if (nowMs() - tDown >= CAL_ENTRY_HOLD_MS) {
      longPress = true;
      uiShow("Bench menu", "release SET");    // so you know to let go
      break;
    }
    sleep_ms(5);
  }
  uiWaitSetRelease();                      // either way, and let the bounce settle
  return longPress;
}

// Target entry. `in` and `level` carry over between runs, so run 2 starts from
// what run 1 used. The bottle position is always derived from the level.
static void enterSettings(RunInputs &in, int &level) {
#if USE_DIAL_ENTRY
  in.D     = uiEnterDistanceM(in.D);
  in.B     = uiEnterBottleLineM(in.B);
  in.T     = uiEnterTimeS(in.T);
  level    = uiSelectLevel(level);
  in.mpOfs  = uiEnterOffsetM("MPofs:", in.mpOfs);
  in.btlOfs = uiEnterOffsetM("BtlOfs:", in.btlOfs);
#else
  // No dial: the targets are compile-time constants (config.h §5b), clamped so a
  // bad edit can't plan an illegal run.
  in.D      = std::clamp(DEFAULT_TARGET_D_M,    TARGET_DIST_MIN_M, TARGET_DIST_MAX_M);
  in.B      = std::clamp(DEFAULT_BOTTLE_LINE_M, BOTTLE_LINE_MIN_M, BOTTLE_LINE_MAX_M);
  in.T      = std::clamp(DEFAULT_TARGET_T_S,    TARGET_TIME_MIN_S, TARGET_TIME_MAX_S);
  level     = std::clamp(DEFAULT_LEVEL, 0, 2);
  in.mpOfs  = std::clamp(DEFAULT_MP_OFFSET_M,  STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M);
  in.btlOfs = std::clamp(DEFAULT_BTL_OFFSET_M, STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M);
#endif
  in.xBottle = bottleXForLevel(level);
}

static void printPlan(const RunPlan &p) {
  printf("[SM] PLAN D=%.2f B=%.2f T=%.1f bottle@%.1f  mpOfs=%+.3f btlOfs=%+.3f\n",
         p.in.D, p.in.B, p.in.T, p.in.xBottle, p.in.mpOfs, p.in.btlOfs);
  if (!p.valid) { printf("[SM] PLAN INVALID: %s\n", p.error); return; }
  printf("[SM]   contact@%.3f  slow@%.3f  push@%.3f  xF=%.3f  xFinal=%.3f  rev=%.3f  path=%.2f m\n",
         p.xContact, p.captureStart, p.pushStart, p.xF, p.xFinal, p.rev, p.path);
  printf("[SM]   v cruise=%.2f push=%.2f rev=%.2f m/s  motion~%.2f s  delay=%.2f s%s\n",
         p.v.vCruise, p.v.vPush, p.v.vRev, p.tMotion, p.delayS,
         p.timeFits ? "" : "   ** T TOO SHORT: will finish late **");
}

//==============================================================================
// One run, ARMED -> stopped.
//==============================================================================
static RunResult executeRun(const RunPlan &p, bool imuOk) {
  RunResult r{};
  r.abort = Abort::None;

  // ---- ARMED: MP over the Start dot (§4.d.vii.1); nothing moves until START ---
  // The estimator runs from here so the IMU stays polled and fresh; the pose is
  // zeroed at the pencil press, so placing or nudging the car while ARMED is fine.
  driveCoast();
  steerCenter();
  poseSetActive(true);
  char armL1[17];
  snprintf(armL1, sizeof armL1, "D%.2fB%.2fT%.1f", p.in.D, p.in.B, p.in.T);
  uiShow("ARMED  SET=back", armL1);
  const uint32_t tArm = nowMs();
  bool imuChecked = false;
  // START only counts after ARMED_SETTLE_MS: the IMU sat unpolled through target
  // entry and the BNO085 queues reports, so core1 needs a moment to work through
  // the backlog. A press sooner would zero the heading on a queued, stale sample.
  // A press during the settle time is REMEMBERED, and Run Time is taken from that
  // press (the ES timers start at actuation); the zero just waits for the settle.
  // Both buttons act on a PRESS EDGE, never on a level: a START that is already
  // down when ARMED appears (held, or a stuck/shorted switch) must not launch the
  // car while it is being placed, and a SET still held from the plan screen must
  // not cancel the arm.
  uint32_t tPress = 0, tCand = 0;   // tCand: a START press not yet qualified
  bool lastStart = uiStartPressed();
  HeldButton setCancel;
  bool setReleased = !uiSetPressed();   // a SET still held from the last screen must be released first
  while (true) {
    const bool startNow = uiStartPressed(), setNow = uiSetPressed();
    const bool startEdge = startNow && !lastStart;
    if (!setNow) setReleased = true;
    lastStart = startNow;
    if (tPress == 0) {
      // Qualify the press: it must stay down BUTTON_QUALIFY_MS (a pencil press
      // lasts far longer), so a one-sample glitch can't start the run before the
      // supervisor is ready (§8.r.ii). Run Time still counts from the press.
      if (startEdge) tCand = nowMs();
      if (tCand != 0 && !startNow) tCand = 0;
      if (tCand != 0 && nowMs() - tCand >= BUTTON_QUALIFY_MS) {
        tPress = tCand;
        driveHold();                // from actuation on, the car must not roll
      }
    }
    if (tPress != 0 && nowMs() - tArm >= ARMED_SETTLE_MS) break;
    // Cancel only on a qualified SET hold, like every other SET action: one noise
    // sample must not throw away six entry screens' worth of targets.
    if (setReleased && setCancel.update(setNow, nowMs()) && tPress == 0) {   // back out: no run happened
      uiWaitSetRelease();
      poseSetActive(false);
      r.cancelled = true;
      return r;
    }
    // Flag a missing heading BEFORE the run instead of discovering it after: ONE
    // check, IMU_STALE_MS in (time for core1 to fetch fresh samples). Only once,
    // because an LCD write blocks ~30 ms and a later one could land on the pencil
    // press (timestamping it late, or missing a quick tap). No printf either.
    if (!imuChecked && nowMs() - tArm > IMU_STALE_MS) {
      imuChecked = true;
      if (!(imuOk && imuHealthy())) uiShowLive("ARMED  NO IMU!", armL1);
    }
    sleep_ms(1);
  }

  // ---- START: Run Time begins at actuation (§8.o) ----------------------------
  const uint32_t t0 = tPress;
  HeldButton setHeld;               // SET aborts only on a real press, not a glitch
  // Hold the car still through the start delay. Coasting let the pencil press or a
  // slight slope roll it off the Start dot after the zero — or back 3 cm, which
  // trips the wrong-way abort at launch (a Failed Run, §8.r.i).
  driveHold();
  s_logN = 0;
  logEvent(0, "START", 0.0f, 0.0f, 0.0f);

  // Heading health, two kinds:
  //   headingDead  — permanent for this run: no IMU, the zero failed, or the chip
  //                  RESET after the zero (it restarts in a new frame, so the zero
  //                  is wrong from then on even once it is healthy again). The
  //                  reset count catches a reset that recovers between samples.
  //   stale        — samples briefly late (an I2C retry). Steer straight while it
  //                  lasts, then carry on: the zero is still valid.
  const uint32_t imuResets = imuResetCount();
  bool headingDead = !imuOk;
  // Whether the heading may be used right now. Latches headingDead on a reset.
  // `moving`: a long gap only matters once the car moves — the estimator then
  // integrates position along a heading it can't see. In the stationary delay
  // nothing is integrated and the zero is still valid, so only a reset counts.
  auto headingUsable = [&](uint32_t msNow, float x, float v, bool moving) {
    if (!headingDead && imuResetCount() != imuResets) {
      headingDead = true;
      logEvent(msNow - t0, "IMU RESET", x, v, 0.0f);
    }
    if (moving && !headingDead && imuSampleAgeMs() > IMU_TRUST_GAP_MS) {
      headingDead = true;                      // blind too long to trust again
      logEvent(msNow - t0, "IMU GAP", x, v, 0.0f);
    }
    return !headingDead && imuHealthy();
  };
  // Zeroes the encoder here and now; the heading zero comes from core1. If core1
  // can't do it (IMU stuck), steer encoder-only rather than on a stale heading.
  if (!poseZeroAtArm()) {
    headingDead = true;
    logEvent(nowMs() - t0, "ZERO FAIL", 0.0f, 0.0f, 0.0f);
  } else if (!headingDead && (imuRecovering() || imuSampleAgeMs() > IMU_TRUST_GAP_MS)) {
    // A heading gap longer than IMU_TRUST_GAP_MS: the newest yaw may be from before
    // the car was placed (e.g. a loose wire since target entry).
    // The chip reset while ARMED (already counted in imuResets) and hasn't sent a
    // sample since, so the zero used an OLD-frame or placeholder yaw. Nothing later
    // would show it: once healthy, the car would steer on a wrong heading. (A
    // merely LATE sample is fine — the car is standing still, so the last good
    // yaw is still its heading; steering just pauses until samples resume.)
    headingDead = true;
    logEvent(nowMs() - t0, "ZERO STALE", 0.0f, 0.0f, 0.0f);
  }

  // ---- DELAY: stand still. Counts toward Run Time and may exceed 3 s (§8.o.i).
  const uint32_t delayMs = (uint32_t)(p.delayS * 1000.0f);
  uint32_t lastLcd = 0;
  bool     lcdOk   = true;
  while (nowMs() - t0 < delayMs) {
    if (setHeld.update(uiSetPressed(), nowMs())) { r.abort = Abort::Operator; break; }
    (void)headingUsable(nowMs(), 0.0f, 0.0f, false);   // a reset here must still be caught
    const uint32_t el = nowMs() - t0;
    // LCD only while stationary, and never within LCD_LAUNCH_GUARD_MS of the
    // launch (a write would make it late). Written as `el + guard < delayMs`:
    // `delayMs - el` is unsigned and wraps if el has just passed delayMs.
    if (LCD_RUN_UPDATE_MS && lcdOk && el - lastLcd >= LCD_RUN_UPDATE_MS &&
        el + LCD_LAUNCH_GUARD_MS < delayMs) {
      lastLcd = el;
      char a[17], b[17];
      snprintf(a, sizeof a, "Delay %5.1fs", (delayMs - el) / 1000.0f);
      snprintf(b, sizeof b, "t %5.1f / %4.1f", el / 1000.0f, p.in.T);
      const uint32_t tw = nowMs();
      uiShowLive(a, b);
      // A healthy write is ~30 ms. Much longer means the LCD bus is failing (each
      // byte running into LCD_I2C_TIMEOUT_US): stop updating, so neither the
      // launch nor the SET check waits on a dead display.
      if (nowMs() - tw > LCD_SLOW_WRITE_MS) lcdOk = false;
    }
    sleep_ms(2);
  }
  if (r.abort != Abort::None) {     // aborted before any motion: no run occurred
    logEvent(nowMs() - t0, abortName(r.abort), 0.0f, 0.0f, 0.0f);
    // Stay braked (driveHold, set at START): the car is on the Start dot and must
    // not roll. Released at the next ARMED, like after a run.
    poseSetActive(false);
    r.runTime = (nowMs() - t0) / 1000.0f;
    return r;
  }

  // ---- Motion -----------------------------------------------------------------
  const uint32_t tMove = nowMs();
  r.drove = true;                   // the motor drives from here on
  odoSpeedReset();                  // speed estimate: this core, from first motion
  const float timeoutS = p.delayS + RUN_TIMEOUT_FACTOR * p.tMotion + RUN_TIMEOUT_PAD_S;
  EvState  st  = EvState::CRUISE;
  int      dir = +1;                // leg direction: +1 forward, -1 reverse
  SpeedLoop loop;
  loop.reset();
  float    vRef = 0.0f, pwm = 0.0f, delta = 0.0f, stallBoost = 0.0f;
  float    xMax = 0.0f;
  float    revAnchor = 0.0f;        // xMax frozen at REVERSE entry (wrong-way reference)
  uint32_t legStartMs = tMove, turnStartMs = 0;
  float    legStartX = 0.0f;
  uint32_t progMs = tMove, encMs = tMove, stuckMs = tMove;
  float    progX = 0.0f, encX = 0.0f, stuckX = 0.0f;
  uint64_t lastUs = to_us_since_boot(get_absolute_time());
  logEvent(tMove - t0, stateName(st), 0.0f, 0.0f, 0.0f);

  auto enter = [&](EvState next, uint32_t ms, float x, float v) {
    st = next;
    logEvent(ms - t0, stateName(next), x, v, pwm);
  };

  while (true) {
    const uint64_t us = to_us_since_boot(get_absolute_time());
    const float    dt = (float)(us - lastUs) * 1e-6f;
    lastUs = us;
    const uint32_t ms = (uint32_t)(us / 1000);
    const float    x  = odoDistanceM();       // rear MP position (net travel)
    odoSpeedTick(x);
    const float    v  = odoSpeedMps();        // signed: + forward
    xMax = std::max(xMax, x);
    // Moved = a few mm of real travel, not one count of jitter from the pencil.
    if (std::fabs(x) >= MOVED_MIN_M) r.moved = true;
    const bool steerOk = headingUsable(ms, x, v, true);
    // Report heading trouble only if it actually took the steering away; a blip
    // during the stationary delay that recovered before launch did not.
    if (!steerOk && !r.imuLost) {
      r.imuLost = true;
      if (!headingDead) logEvent(ms - t0, "IMU STALE", x, v, pwm);
    }

    // ---- guards --------------------------------------------------------------
    if (setHeld.update(uiSetPressed(), ms))      { r.abort = Abort::Operator; break; }
    if ((ms - t0) / 1000.0f > timeoutS)          { r.abort = Abort::Timeout;  break; }
    if (st != EvState::TURNAROUND) {
      // Dead encoder: still no count at all after the stall re-kick (below) has
      // had its chance. Without feedback the speed loop winds up, so this has to
      // be much faster than the stall check.
      if (x != encX) { encX = x; encMs = ms; }
      else if (ms - encMs > ENC_DEAD_MS)          { r.abort = Abort::NoEncoder; break; }
      // Stall: counts, but not getting anywhere.
      if (std::fabs(x - progX) >= PROGRESS_MIN_M) { progX = x; progMs = ms; }
      else if (ms - progMs > STALL_ABORT_MS)      { r.abort = Abort::Stall; break; }
    }
    // Wrong way: backward from the Start is a Failed Run (§8.r.i), and forward on
    // the reverse leg would push the bottle. Also catches a flipped ENCODER_SIGN.
    if (dir > 0 && x < -WRONG_WAY_M)               { r.abort = Abort::WrongWay; break; }
    // Reverse: measured against the forward peak FROZEN at the turnaround. (Not
    // the live xMax — it is updated above every tick, so x could never exceed it.)
    if (dir < 0 && x > revAnchor + WRONG_WAY_M)    { r.abort = Abort::WrongWay; break; }

    // ---- transitions -----------------------------------------------------------
    switch (st) {
      case EvState::CRUISE:
        if (x >= p.captureStart) enter(EvState::CAPTURE, ms, x, v);
        break;
      case EvState::CAPTURE:
        if (x >= p.pushStart) enter(EvState::PUSH, ms, x, v);
        break;
      case EvState::PUSH:
        if (x >= p.xF) {
          driveHold();                // dynamic brake from V_PUSH_END: gentle
          turnStartMs = ms;
          enter(EvState::TURNAROUND, ms, x, v);
        }
        break;
      case EvState::TURNAROUND:
        if (ms - turnStartMs >= TURN_MS) {
          dir = -1;
          revAnchor = xMax;         // includes any roll during the turnaround
          // Start the reverse with the wheels STRAIGHT, not at the forward leg's
          // last angle: in reverse that angle steers the other way, and on a leg
          // shorter than REV_STEER_HOLD_M AimAtTarget would hold it the whole way.
          delta = 0.0f;
          vRef = 0.0f;
          loop.reset();
          legStartMs = ms; legStartX = x;
          progMs = ms;     progX = x;
          stallBoost = 0.0f;
          encMs = ms;      encX = x;
          stuckMs = ms;    stuckX = x;
          enter(EvState::REVERSE, ms, x, v);
        }
        break;
      case EvState::REVERSE:
        if (x <= p.xFinal) enter(EvState::STOP, ms, x, v);
        break;
      default:
        break;
    }
    if (st == EvState::STOP) break;

    // ---- longitudinal ----------------------------------------------------------
    if (st == EvState::TURNAROUND) {
      pwm = 0.0f;
      driveHold();
    } else {
      float vTarget, aUp;
      if (dir > 0) {
        vTarget = speedProfileForward(x, p);
        aUp = (st == EvState::PUSH) ? A_PUSH : A_ACCEL;   // gentle with the bottle
      } else {
        vTarget = speedProfileReverse(x, p);
        aUp = A_ACCEL;
      }
      // Kick through stiction at each standstill start (launch, start of reverse),
      // and again whenever the car sticks partway: at slow targets the PI's error
      // is tiny and it winds up too slowly to break free on its own. While stuck,
      // the integrator is frozen: the kick breaks it free, and a wound-up
      // integrator would then surge the car past its stop.
      const bool launching = ms - legStartMs < KICK_MS && std::fabs(x - legStartX) < KICK_DONE_M;
      // Stuck = no PROGRESS in the leg's direction, not "no count at all": a
      // stationary roller sitting on a quadrature edge jitters +/-1 count, which
      // would otherwise keep a stuck car looking alive. Only forward progress
      // resets it (backward motion is the wrong-way check's job).
      if (dir * (x - stuckX) >= STUCK_MIN_M) { stuckX = x; stuckMs = ms; }
      const bool stuck     = ms - stuckMs > STALL_KICK_MS;
      vRef = speedRateLimit(vTarget, vRef, aUp, dt);
      pwm  = loop.update(vRef, dir * v, dt, /*freezeIntegrator=*/stuck);
      // The launch kick is a step (nothing on board yet). A mid-leg stall gets a
      // RAMP instead: it may be mid-push with the bottle seated, where a sudden
      // jump to KICK_PWM could tip it or kick it ahead, and near xFinal a step
      // would lurch the car past the Target. The boost drops back to 0 the
      // moment counts resume.
      // One breakaway mechanism: the launch kick seeds the boost at KICK_PWM, so a
      // car still stuck when the kick window ends keeps that drive instead of
      // dipping back to the PI output and ramping up again.
      if (launching)  stallBoost = KICK_PWM;
      else if (stuck) stallBoost = std::min(stallBoost + STALL_BOOST_PCT_PER_S * dt, KICK_PWM);
      // Once moving, let the boost DECAY rather than drop to 0: a car whose PI
      // output is still below the rolling-friction duty would otherwise stop
      // again at once and hop forward in 2 mm steps, jolting the bottle. The PI
      // (integrator unfrozen now) takes over as the boost fades.
      // But it is only a floor for a car that is still too SLOW: once the
      // measured speed reaches the target, drop it, or a short leg (the reverse
      // after the kick) would be carried past its slowdown ramp.
      else if (dir * v >= vRef) stallBoost = 0.0f;
      else            stallBoost = std::max(stallBoost - STALL_BOOST_PCT_PER_S * dt, 0.0f);
      pwm = std::max(pwm, stallBoost);
      // PWM 0 on this driver is NOT coasting: with the bridges enabled and both
      // inputs low, both low-side FETs short the motor — a hard dynamic brake.
      // When the speed loop asks for zero (an overshoot), coast instead, or the
      // car could stop faster than mu*g mid-push and the bottle slide off.
      if (pwm <= 0.0f)  driveCoast();
      else if (dir > 0) driveForward(pwm);
      else              driveReverse(pwm);
    }

    // ---- lateral ---------------------------------------------------------------
    if (st == EvState::TURNAROUND) {
      // Hold the wheels exactly where they are. Steering a stopped car loads the
      // servo for nothing, and swinging the beam can roll the encoder wheel.
    } else if (!steerOk) {
      delta = 0.0f;                              // no usable heading: wheels straight
    } else if (dir > 0) {
      const Position ro = poseGet();             // roller = front axle (Stanley)
      delta = steerLawForward((float)ro.y, (float)ro.theta, std::fabs(v));
    } else {
      const Position ro = poseGet();             // the reverse law finds the rear itself
      delta = steerLawReverse((float)ro.x, (float)ro.y, (float)ro.theta, std::fabs(v),
                              p.xFinal, delta);
    }
    steerSetAngle(delta);

    sleep_us(CONTROL_TICK_US);
  }

  // ---- STOP -------------------------------------------------------------------
  if (r.abort == Abort::None) {
    driveBrakeToStop(FINAL_BRAKE_PWM, FINAL_BRAKE_MAX_MS);
  } else {
    // A wrong-way abort gets the hold only: it usually means the encoder or the
    // motor leads are reversed, and then "against the measured motion" is the way
    // the car is already going — a counter-pulse would push it further.
    const float vNow = odoSpeedMps();
    driveBrakeToStop(r.abort == Abort::WrongWay ? 0.0f : ABORT_BRAKE_PWM, ABORT_BRAKE_MAX_MS);
    logEvent(nowMs() - t0, abortName(r.abort), odoDistanceM(), vNow, pwm);
  }
  const uint32_t tStop = nowMs();
  r.runTime    = (tStop - t0) / 1000.0f;
  r.motionTime = (tStop - tMove) / 1000.0f;
  r.xMax       = xMax;

  // Keep the dynamic-brake hold AND the wheel angle: the car may still be on the
  // track waiting to be measured, and freeing the drive or re-centring the
  // steering could move the MP. Both are released at the next ARMED.
  sleep_ms(STOP_SETTLE_MS);
  r.xEnd = odoDistanceM();
  poseSetActive(false);
  // Stop driving the servo. The wheels may be at full lock — against the linkage
  // stop — and holding that for minutes through the result screen is stall
  // current. With no pulses the servo stops pushing, and its gear train still
  // holds the wheels where they are, so the measured car doesn't move.
  steerRelease();
  return r;
}

static void report(const RunPlan &p, const RunResult &r) {
  printf("\n[SM] ===== RUN DONE  abort=%s  imu=%s =====\n", abortName(r.abort),
         r.imuLost ? "LOST (encoder-only)" : "ok");
  printf("[SM] event log (t ms, state, x m, v m/s, pwm %%):\n");
  for (int i = 0; i < s_logN; i++) {
    printf("[SM]   %6lu  %-9s x=%7.3f  v=%+5.2f  pwm=%4.1f\n",
           (unsigned long)s_log[i].ms, s_log[i].what, s_log[i].x, s_log[i].v, s_log[i].pwm);
  }

  if (!r.moved && r.drove) {
    // The motor drove but the encoder saw (almost) nothing — a dead encoder, or an
    // abort right at launch. The firmware can't tell whether the car moved, so it
    // must not claim "no run": the event supervisor decides.
    printf("[SM] The motor drove but the encoder saw under %.0f mm of travel. The car\n"
           "[SM] may have moved, so this may count as a run - ask the event supervisor.\n",
           1000.0 * MOVED_MIN_M);
    printf("[SM] SET = next run, hold SET = bench menu\n");
    uiShow("NO COUNTS", "ask ES: moved?");
    return;
  }
  if (!r.moved) {
    // §4.d.vii.4.a: a vehicle that doesn't move when triggered has not had a run.
    printf("[SM] The car never moved: under the rules no run occurred. Fix and re-run.\n");
    printf("[SM] SET = next run, hold SET = bench menu\n");
    char l2[17];
    snprintf(l2, sizeof l2, "ABORT %s", abortName(r.abort));
    uiShow("NO RUN (no move)", l2);
    return;
  }

  const float mpErrCm  = (r.xEnd - p.xFinal) * 100.0f;   // + = stopped beyond Target
  const float fwdErrCm = (r.xMax - p.xF) * 100.0f;       // + = pushed further than planned
  printf("[SM] forward stop: planned %.3f  reached %.3f  (%+.1f cm)\n", p.xF, r.xMax, fwdErrCm);
  printf("[SM] final MP:     planned %.3f  encoder %.3f  (%+.1f cm)\n", p.xFinal, r.xEnd, mpErrCm);
  if (r.abort != Abort::None) {
    // An aborted run stopped early: its time and stop positions say nothing about
    // T_BIAS_S or the offsets. Don't offer numbers someone might average in.
    printf("[SM] Aborted (%s): no T_BIAS_S or offset suggestions from this run.\n",
           abortName(r.abort));
  } else {
    printf("[SM] time: run %.2f s (T %.1f)  motion %.2f s (est %.2f -> T_BIAS_S would be %+.2f)\n",
           r.runTime, p.in.T, r.motionTime, p.tMotion,
           r.motionTime - (p.tMotion - T_BIAS_S));
    printf("[SM] The encoder numbers only show what the car THINKS it did. Measure with\n"
           "[SM] a tape and enter the corrections for run 2 (config.h §5 conventions):\n"
           "[SM]   new MPofs  = %+.3f - (MP position - Target), in m\n"
           "[SM]                e.g. MP 3 cm beyond the Target -> %+.3f\n"
           "[SM]   new BtlOfs = %+.3f + (how much further the bottle should go)\n",
           p.in.mpOfs, p.in.mpOfs - 0.030, p.in.btlOfs);
  }
  printf("[SM] SET = next run, hold SET = bench menu\n");

  char l1[17], l2[17];
  snprintf(l1, sizeof l1, "MP%+5.1fcm t%5.2f", mpErrCm, r.runTime);
  if (r.abort != Abort::None) snprintf(l2, sizeof l2, "ABORT %s", abortName(r.abort));
  else snprintf(l2, sizeof l2, "F%+5.1fcm %s", fwdErrCm, r.imuLost ? "IMU!" : "ok");
  uiShow(l1, l2);
}

//==============================================================================
void smRun(bool imuOk) {
  RunInputs in{DEFAULT_TARGET_D_M, DEFAULT_BOTTLE_LINE_M, DEFAULT_TARGET_T_S,
               bottleXForLevel(DEFAULT_LEVEL), DEFAULT_MP_OFFSET_M, DEFAULT_BTL_OFFSET_M};
  int  level = DEFAULT_LEVEL;
  bool showReady = true;

  while (true) {
    // ---- SETUP: short SET starts entry, long SET opens the bench menu ----------
    // After a run the result screen stays up and doubles as this prompt.
    if (showReady) uiShow("EV ready", "SET=run hold=cal");
    showReady = true;
    if (waitSetPress()) {
      printf("[SM] SET held -> bench menu\n");
      calMenu();
      continue;
    }

    if (const char *e = preflightError()) {
      printf("[SM] NOT CALIBRATED: %s — see config.h §1. Refusing to arm.\n", e);
      uiShow("NOT CALIBRATED", e);
      sleep_ms(UI_MSG_MS);
      continue;
    }

    enterSettings(in, level);
    const RunPlan p = planRun(in);
    printPlan(p);
    if (!p.valid) {
      uiShow("PLAN ERROR", p.error);
      sleep_ms(UI_MSG_MS);
      continue;
    }
    {
      char l1[17], l2[17];
      snprintf(l1, sizeof l1, "xF%5.2f rev%4.2f", p.xF, p.rev);
      snprintf(l2, sizeof l2, "dly%4.1fs %s", p.delayS, p.timeFits ? "ok" : "T SHORT");
      uiShow(l1, l2);
      sleep_ms(UI_PLAN_MS);
    }
    if (!imuOk) {
      // Steering off is still a scored run; no run at all is a Failed Run (2500).
      uiShow("NO IMU: steer", "off. SET=ok");
      if (waitSetPress()) {         // long press: the bench menu, as the LCD said
        printf("[SM] SET held -> bench menu\n");
        calMenu();
        continue;
      }
    }

    const RunResult r = executeRun(p, imuOk);
    if (r.cancelled) continue;
    report(p, r);
    showReady = false;
  }
}
