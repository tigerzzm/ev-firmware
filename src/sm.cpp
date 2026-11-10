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
enum class Abort { None, Operator, Timeout, NoEncoder, Stall, WrongWay, RealMotion };

static const char *abortName(Abort a) {
  switch (a) {
    case Abort::Operator:  return "by SET";
    case Abort::Timeout:   return "timeout";
    case Abort::NoEncoder: return "no motion";   // dead encoder, or car stuck
    case Abort::Stall:     return "stall";
    case Abort::WrongWay:  return "wrong way";
    case Abort::RealMotion: return "on floor?";  // dry run, but the real roller turned
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
    case EvState::DELAY:      return "DELAY";
  }
  return "?";
}

static uint32_t nowMs() { return to_ms_since_boot(get_absolute_time()); }

// Target entries, kept across runs (and dry runs) so run 2 starts from run 1's.
static RunInputs s_in{DEFAULT_TARGET_D_M, DEFAULT_BOTTLE_LINE_M, DEFAULT_TARGET_T_S,
                      bottleXForLevel(DEFAULT_LEVEL), DEFAULT_MP_OFFSET_M, DEFAULT_BTL_OFFSET_M,
                      A_PAST_M, AIM_TRIM_DEG};
static int  s_level = DEFAULT_LEVEL;
// True for the length of a bench dry run (smDryRun): the encoder is simulated.
static bool s_sim = false;

// ---- RAM event log ---------------------------------------------------------------
// One entry per state change (plus IMU events). Printed after the run so tuning
// has a timeline without any I/O in the control loop.
struct LogEntry { uint32_t ms; const char *what; float x; float v; float pwm; };

// ---- Odometry + steering record -------------------------------------------------
// One sample every TELEM_PERIOD_MS of motion, printed as CSV after the run
// (telemPrint). Like the event log: RAM only, no I/O on the motion path.
struct TelemSample {
  uint32_t ms;                 // since START (the pencil press)
  float    x, v, vRef, pwm;    // encoder MP position (m), speed and target (m/s), duty %
  float    rx, ry, th;         // pose: roller x, y (m) and heading (rad)
  float    delta;              // steering command, rad (+ = wheels left)
  uint8_t  st;                 // EvState
  uint8_t  flags;              // TF_*
};
enum : uint8_t { TF_STEER_OK = 1, TF_STUCK = 2, TF_LAUNCH = 4 };

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

// ---- Stored runs -----------------------------------------------------------------
// The last RUN_HISTORY runs stay in RAM (a ring, oldest overwritten), so several
// runs can be made unplugged and read in one go: START at the result screen
// prints them all, oldest first, and tools/capture_run.py saves each one once.
// RAM only: power-off loses them.
struct RunRecord {
  uint32_t    runNo;          // runs since power-on, from 1
  uint32_t    startMs;        // power-on -> START press; with runNo, the run's id
  bool        sim;            // a bench dry run (simulated encoder)
  RunPlan     plan;           // the settings it ran with
  RunResult   result;
  LogEntry    log[RUN_LOG_CAPACITY];
  int         logN;
  TelemSample tel[TELEM_CAPACITY];
  int         telN;
  bool        telFull;
  // The second heading zero in the start delay (HEADING_ZERO_DELAY_MS): when it
  // happened (0 = not done) and how far the car had turned since the press zero.
  uint32_t    rezeroMs;
  float       rezeroTurn;     // rad, + = left
};
static RunRecord  s_runs[RUN_HISTORY];
static int        s_runsHead  = 0;   // the slot the next run records into
static int        s_runsN     = 0;   // finished runs stored
static uint32_t   s_runCount  = 0;   // runs since power-on
static RunRecord *s_rec       = &s_runs[0];   // being recorded, or being printed

static void logEvent(uint32_t ms, const char *what, float x, float v, float pwm) {
  if (s_rec->logN < RUN_LOG_CAPACITY) s_rec->log[s_rec->logN++] = LogEntry{ms, what, x, v, pwm};
}

static void telemAdd(const TelemSample &t) {
  if (s_rec->telN < TELEM_CAPACITY) s_rec->tel[s_rec->telN++] = t;
  else s_rec->telFull = true;
}

// The id line tools/capture_run.py names files by and de-duplicates on.
static void printRunId(const RunRecord &rec) {
  printf("[SM] run #%lu (power-on +%.1f s)\n", (unsigned long)rec.runNo, rec.startMs / 1000.0f);
}

// Everything in config.h §1 that must be MEASURED before the car may move. A zero
// here means a distance that reads 0, a plan built on a guess, or steering on an
// unknown servo scale — refuse rather than run. Checked BEFORE target entry, so
// you find out without typing in D, B and T first. (planRun() checks L_P_M again
// for its own callers; the message is the same.)
static const char *preflightError() { return calibrationMissing(); }

static void printStoredRuns();

// Waits for a fresh SET press. Returns true for a long press (>= CAL_ENTRY_HOLD_MS).
// With `startReprints`, a START press meanwhile prints every stored run again
// (settings, record, event log, result), so runs made without the USB cable can
// still be read: plug in, open the serial monitor, press START.
static bool waitSetPress(bool startReprints = false) {
  while (uiSetPressed()) sleep_ms(5);      // ignore a press still held
  bool lastStart = uiStartPressed();
  while (!uiSetPressed()) {                // wait for a fresh press
    if (startReprints) {
      const bool sp = uiStartPressed();
      if (sp && !lastStart) printStoredRuns();
      lastStart = sp;
    }
    sleep_ms(2);
  }
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
  in.margin = uiEnterBottleMarginM(in.margin);
  in.aimDeg = uiEnterAimDeg(in.aimDeg);
#else
  // No dial: the targets are compile-time constants (config.h §5b), clamped so a
  // bad edit can't plan an illegal run.
  in.D      = std::clamp(DEFAULT_TARGET_D_M,    TARGET_DIST_MIN_M, TARGET_DIST_MAX_M);
  in.B      = std::clamp(DEFAULT_BOTTLE_LINE_M, BOTTLE_LINE_MIN_M, BOTTLE_LINE_MAX_M);
  in.T      = std::clamp(DEFAULT_TARGET_T_S,    TARGET_TIME_MIN_S, TARGET_TIME_MAX_S);
  level     = std::clamp(DEFAULT_LEVEL, 0, 2);
  in.mpOfs  = std::clamp(DEFAULT_MP_OFFSET_M,  STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M);
  in.btlOfs = std::clamp(DEFAULT_BTL_OFFSET_M, STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M);
  in.margin = std::clamp(A_PAST_M, A_PAST_MIN_M, A_PAST_MAX_M);
  in.aimDeg = std::clamp(AIM_TRIM_DEG, AIM_TRIM_MIN_DEG, AIM_TRIM_MAX_DEG);
#endif
  in.xBottle = bottleXForLevel(level);
}

static void printPlan(const RunPlan &p) {
  printf("[SM] PLAN D=%.2f B=%.2f T=%.1f bottle@%.1f  mpOfs=%+.3f btlOfs=%+.3f btlPast=%.1fcm aim=%+.2fdeg\n",
         p.in.D, p.in.B, p.in.T, p.in.xBottle, p.in.mpOfs, p.in.btlOfs, 100.0f * p.in.margin, p.in.aimDeg);
  if (!p.valid) { printf("[SM] PLAN INVALID: %s\n", p.error); return; }
  printf("[SM]   contact@%.3f  slow@%.3f  push@%.3f  xF=%.3f  xFinal=%.3f  rev=%.3f  path=%.2f m\n",
         p.xContact, p.captureStart, p.pushStart, p.xF, p.xFinal, p.rev, p.path);
  printf("[SM]   v cruise=%.2f catch=%.2f push=%.2f rev=%.2f m/s  motion~%.2f s  delay=%.2f s%s\n",
         p.v.vCruise, p.v.vCatch, p.v.vPush, p.v.vRev, p.tMotion, p.delayS,
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
  uiShow(s_sim ? "SIM ARMED SET=bk" : "ARMED  SET=back", armL1);
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
  s_rec = &s_runs[s_runsHead];      // records into the oldest slot once the ring is full
  if (s_runsN == RUN_HISTORY) s_runsN--;
  s_rec->runNo      = ++s_runCount;
  s_rec->startMs    = t0;
  s_rec->logN       = 0;
  s_rec->telN       = 0;
  s_rec->telFull    = false;
  s_rec->rezeroMs   = 0;
  s_rec->rezeroTurn = 0.0f;
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
  const float aimRad = p.in.aimDeg * (float)M_PI / 180.0f;
  poseSetAimTrim(aimRad);           // before the zeros: every zero starts at -aim
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
  uint32_t lastTel = 0;
  bool     lcdOk   = true;
  bool     firstTel = true;
  // Second heading zero once the press has settled (START_SETTLE_S); the plan keeps
  // the delay at least that long. It must happen before the motor starts.
  const uint32_t zeroAt = std::min(HEADING_ZERO_DELAY_MS, delayMs);
  bool reZeroPending = !headingDead && HEADING_ZERO_DELAY_MS > 0;   // 0 = settle off
  auto reZero = [&](uint32_t el) {
    reZeroPending = false;
    const float before = (float)poseGet().theta + aimRad;   // turned since the press (trim removed)
    if (poseZeroHeading()) {
      s_rec->rezeroMs   = el;
      s_rec->rezeroTurn = before;
      logEvent(el, "HDG ZERO", odoDistanceM(), 0.0f, 0.0f);
    } else {
      headingDead = true;
      logEvent(nowMs() - t0, "ZERO FAIL", 0.0f, 0.0f, 0.0f);
    }
  };
  while (nowMs() - t0 < delayMs) {
    if (setHeld.update(uiSetPressed(), nowMs())) { r.abort = Abort::Operator; break; }
    const bool hdgOk = headingUsable(nowMs(), 0.0f, 0.0f, false);   // a reset here must still be caught
    const uint32_t el = nowMs() - t0;
    if (reZeroPending && el >= zeroAt) {
      reZero(el);
      firstTel = true;                               // a sample right after the new zero
    }
    // Record the heading while the car stands still: a step just after the zero is
    // the START press nudging the car; a steady slope is IMU drift (config.h §7).
    if (firstTel || el - lastTel >= TELEM_PERIOD_MS) {
      firstTel = false;
      lastTel  = el;
      const Position ro = poseGet();
      telemAdd({el, odoDistanceM(), 0.0f, 0.0f, 0.0f, (float)ro.x, (float)ro.y, (float)ro.theta, 0.0f,
                (uint8_t)EvState::DELAY, (uint8_t)(hdgOk ? TF_STEER_OK : 0)});
    }
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
  // The loop can end a tick before zeroAt when the delay IS the settle: zero now,
  // still before the motor starts.
  if (r.abort == Abort::None && reZeroPending) reZero(nowMs() - t0);
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
  uint32_t lastTelMs = tMove - TELEM_PERIOD_MS;   // so the first tick is recorded
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
    // Dry run with the car really moving: the simulated distance says nothing
    // about where it is, and the encoder watchdogs above are watching the model.
    if (s_sim && odoSimRealMotionM() > SIM_REAL_MOTION_M) { r.abort = Abort::RealMotion; break; }

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
    bool launching = false, stuck = false;   // also recorded below
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
      launching = ms - legStartMs < KICK_MS && std::fabs(x - legStartX) < KICK_DONE_M;
      // Stuck = no PROGRESS in the leg's direction, not "no count at all": a
      // stationary roller sitting on a quadrature edge jitters +/-1 count, which
      // would otherwise keep a stuck car looking alive. Only forward progress
      // resets it (backward motion is the wrong-way check's job).
      if (dir * (x - stuckX) >= STUCK_MIN_M) { stuckX = x; stuckMs = ms; }
      stuck     = ms - stuckMs > STALL_KICK_MS;
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
      if (st == EvState::PUSH && vRef > 0.0f) pwm = std::max(pwm, PUSH_PWM_FLOOR_PCT);
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

    if (ms - lastTelMs >= TELEM_PERIOD_MS) {
      lastTelMs = ms;
      const Position ro = poseGet();
      telemAdd({ms - t0, x, v, vRef, pwm, (float)ro.x, (float)ro.y, (float)ro.theta, delta,
                (uint8_t)st,
                (uint8_t)((steerOk ? TF_STEER_OK : 0) | (stuck ? TF_STUCK : 0) |
                          (launching ? TF_LAUNCH : 0))});
    }

    sleep_us(CONTROL_TICK_US);
  }

  // ---- STOP -------------------------------------------------------------------
  if (r.abort == Abort::None) {
    driveBrakeToStop(FINAL_BRAKE_PWM, FINAL_BRAKE_MAX_MS);
  } else {
    // A wrong-way abort gets the hold only: it usually means the encoder or the
    // motor leads are reversed, and then "against the measured motion" is the way
    // the car is already going — a counter-pulse would push it further. Same for
    // a dry run caught on the floor: the measured motion is the simulated one.
    const float vNow = odoSpeedMps();
    const bool holdOnly = r.abort == Abort::WrongWay || r.abort == Abort::RealMotion;
    driveBrakeToStop(holdOnly ? 0.0f : ABORT_BRAKE_PWM, ABORT_BRAKE_MAX_MS);
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
  {
    // One last sample, settled: where the car (and the pose) ended up.
    const Position ro = poseGet();
    telemAdd({nowMs() - t0, r.xEnd, 0.0f, 0.0f, 0.0f, (float)ro.x, (float)ro.y, (float)ro.theta,
              delta, (uint8_t)EvState::STOP, 0});
  }
  poseSetActive(false);
  // Stop driving the servo. The wheels may be at full lock — against the linkage
  // stop — and holding that for minutes through the result screen is stall
  // current. With no pulses the servo stops pushing, and its gear train still
  // holds the wheels where they are, so the measured car doesn't move.
  steerRelease();
  return r;
}

// The odometry + steering record: CSV between markers (paste into a spreadsheet
// or plot it), then a summary of what usually needs checking after a run — did
// the steering run, how far off the line the pose says it went, and whether the
// servo centre needs a trim. Printed before report(), so the result stays last.
static void telemPrint() {
  if (s_rec->telN == 0) return;
  printf("\n[TEL] ===== odometry + steering record%s: %d samples, every %lu ms =====\n",
         s_sim ? " (SIMULATED distance)" : "", s_rec->telN, (unsigned long)TELEM_PERIOD_MS);
  printf("[TEL] signs: + = LEFT for y, heading and steer (heading + = nose turned left)\n");
  printf("[TEL] geometry: ENC_FROM_MP_M=%.3f L_P_M=%.3f\n", ENC_FROM_MP_M, L_P_M);
  printf("[TEL] BEGIN CSV\n");
  printf("t_ms,state,x_mp_m,v_mps,vref_mps,pwm_pct,roller_x_m,y_m,heading_deg,steer_deg,steer_ok,stuck,launch\n");
  for (int i = 0; i < s_rec->telN; i++) {
    const TelemSample &t = s_rec->tel[i];
    printf("%lu,%s,%.4f,%.3f,%.3f,%.1f,%.4f,%.4f,%.2f,%.2f,%d,%d,%d\n",
           (unsigned long)t.ms, stateName((EvState)t.st), t.x, t.v, t.vRef, t.pwm, t.rx, t.ry,
           t.th * 180.0f / (float)M_PI, t.delta * 180.0f / (float)M_PI,
           (t.flags & TF_STEER_OK) ? 1 : 0, (t.flags & TF_STUCK) ? 1 : 0, (t.flags & TF_LAUNCH) ? 1 : 0);
  }
  printf("[TEL] END CSV\n");
  if (s_rec->telFull) printf("[TEL] record FULL at %d samples: the end of the run is missing.\n", TELEM_CAPACITY);

  int   moving = 0, off = 0, nTrim = 0;
  float maxYf = 0.0f, maxYr = 0.0f, yFwdEnd = 0.0f, sumTrim = 0.0f;
  for (int i = 0; i < s_rec->telN; i++) {
    const TelemSample &t = s_rec->tel[i];
    const EvState st = (EvState)t.st;
    const bool fwd = st == EvState::CRUISE || st == EvState::CAPTURE || st == EvState::PUSH;
    const bool ok  = t.flags & TF_STEER_OK;
    if (fwd || st == EvState::REVERSE) { moving++; if (!ok) off++; }
    if (fwd) { maxYf = std::max(maxYf, std::fabs(t.ry)); yFwdEnd = t.ry; }
    if (st == EvState::REVERSE) maxYr = std::max(maxYr, std::fabs(t.ry));
    if (st == EvState::CRUISE && ok && !(t.flags & TF_LAUNCH) && t.x >= TELEM_TRIM_FROM_M) {
      sumTrim += t.delta;
      nTrim++;
    }
  }
  auto side = [](float v) { return v >= 0.0f ? "LEFT" : "RIGHT"; };
  const TelemSample &last = s_rec->tel[s_rec->telN - 1];
  // Start delay, car standing still: the heading should stay at its zero.
  {
    const float d = 180.0f / (float)M_PI;
    if (s_rec->rezeroMs)
      printf("[TEL] heading re-zeroed %.1f s after START: the car had turned %.2f deg %s since the press\n"
             "[TEL]   (the press zero would have locked that in; a big value = the press twists the car)\n",
             s_rec->rezeroMs / 1000.0f, std::fabs(s_rec->rezeroTurn * d), side(s_rec->rezeroTurn));
    // Drift: the slope while still, after the last zero plus TELEM_NUDGE_WINDOW_MS.
    const uint32_t from = (s_rec->rezeroMs ? s_rec->rezeroMs : 0) + TELEM_NUDGE_WINDOW_MS;
    int n = 0;
    float hA = 0.0f, hB = 0.0f;
    uint32_t tA = 0, tB = 0;
    for (int i = 0; i < s_rec->telN && (EvState)s_rec->tel[i].st == EvState::DELAY; i++) {
      const TelemSample &t = s_rec->tel[i];
      if (t.ms < from) continue;
      if (n == 0) { hA = t.th; tA = t.ms; }
      hB = t.th; tB = t.ms; n++;
    }
    if (n >= 2 && tB > tA + 1000)
      printf("[TEL] start delay, car still: heading drift %.2f deg/min %s (%.1f s measured)\n",
             std::fabs((hB - hA) * d) * 60000.0f / (tB - tA), side(hB - hA), (tB - tA) / 1000.0f);
  }
  if (off == 0) printf("[TEL] steering: active for the whole run\n");
  else          printf("[TEL] steering: OFF (wheels centred, no usable heading) for %d of %d moving samples\n",
                       off, moving);
  // The rear MP is what gets scored; the pose tracks the roller ENC_FROM_MP_M ahead.
  const float mpY = last.ry - ENC_FROM_MP_M * std::sin(last.th);
  const float thDeg = last.th * 180.0f / (float)M_PI;
  printf("[TEL] sideways per the pose (front roller): forward max %.1f cm, %.1f cm %s at the\n"
         "[TEL]   forward stop; reverse max %.1f cm; final %.1f cm %s\n",
         100.0f * maxYf, 100.0f * std::fabs(yFwdEnd), side(yFwdEnd), 100.0f * maxYr,
         100.0f * std::fabs(last.ry), side(last.ry));
  printf("[TEL] end: rear MP %.1f cm %s of the car's line, heading %.1f deg %s (nose)\n",
         100.0f * std::fabs(mpY), side(mpY), std::fabs(thDeg), side(thDeg));
  printf("[TEL]   The pose is what the car THOUGHT: tape-measure the real sideways error. A\n"
         "[TEL]   car that drifts while the pose stays near 0 is skidding or its heading is off.\n");
  if (s_sim) {
    printf("[TEL] (dry run: no servo trim from a simulated run)\n");
  } else if (nTrim >= TELEM_TRIM_MIN_SAMPLES) {
    // In steady cruise the command cancels whatever offset the wheels have at
    // SERVO_CENTER_US, so the average command IS the trim. Re-centre on the pulse
    // it took to drive straight.
    const float meanRad = sumTrim / nTrim, meanDeg = meanRad * 180.0f / (float)M_PI;
    const long  newCentre = std::lround(SERVO_CENTER_US + STEER_DIR_SIGN * meanRad * SERVO_US_PER_RAD);
    if (std::fabs(meanDeg) >= TELEM_TRIM_MIN_DEG)
      printf("[TEL] servo trim: the cruise steered %.2f deg %s on average to drive straight.\n"
             "[TEL]   Try SERVO_CENTER_US = %ld (now %d).\n",
             std::fabs(meanDeg), side(meanDeg), newCentre, SERVO_CENTER_US);
    else
      printf("[TEL] servo trim: %.2f deg %s average on the cruise - centre is fine.\n",
             std::fabs(meanDeg), side(meanDeg));
  } else {
    printf("[TEL] servo trim: too little steady cruise with steering on to judge.\n");
  }
}

static void report(const RunPlan &p, const RunResult &r);

// A finished run: keep its settings and result with its records, and move the
// ring on so the next run records into the next slot.
static void remember(const RunPlan &p, const RunResult &r) {
  s_rec->plan   = p;
  s_rec->result = r;
  s_rec->sim    = s_sim;
  s_runsHead = (s_runsHead + 1) % RUN_HISTORY;
  s_runsN    = std::min(s_runsN + 1, RUN_HISTORY);
}

// Print every stored run, oldest first: settings, record, event log and result,
// each ending with the report's last line (tools/capture_run.py saves on it).
static void printStoredRuns() {
  if (s_runsN == 0) {
    printf("[SM] No run to print yet.\n");
    return;
  }
  RunRecord *const rec = s_rec;
  const bool sim = s_sim;
  for (int k = 0; k < s_runsN; k++) {
    s_rec = &s_runs[(s_runsHead - s_runsN + k + RUN_HISTORY) % RUN_HISTORY];
    s_sim = s_rec->sim;             // the labels follow the run, not now
    printf("\n[SM] ===== stored %s %d of %d =====\n", s_sim ? "dry run" : "run", k + 1, s_runsN);
    printRunId(*s_rec);
    printPlan(s_rec->plan);         // D, B, bottle: the path plot (tools/plot_run.py) needs them
    telemPrint();
    report(s_rec->plan, s_rec->result);
  }
  s_rec = rec;
  s_sim = sim;
}

static void report(const RunPlan &p, const RunResult &r) {
  const char *next = s_sim ? "[SM] SET = back to the bench menu, START = print the stored runs\n"
                           : "[SM] SET = next run, hold SET = bench menu, START = print the stored runs\n";
  printf("\n[SM] ===== %s DONE  abort=%s  imu=%s =====\n", s_sim ? "SIM DRY RUN" : "RUN",
         abortName(r.abort), r.imuLost ? "LOST (encoder-only)" : "ok");
  if (s_sim)
    printf("[SM] *** SIMULATED: distance and speed came from a model of the motor, not\n"
           "[SM] *** the roller. This checks the sequence, not the car's accuracy.\n");
  printf("[SM] event log (t ms, state, x m, v m/s, pwm %%):\n");
  for (int i = 0; i < s_rec->logN; i++) {
    printf("[SM]   %6lu  %-9s x=%7.3f  v=%+5.2f  pwm=%4.1f\n",
           (unsigned long)s_rec->log[i].ms, s_rec->log[i].what, s_rec->log[i].x, s_rec->log[i].v, s_rec->log[i].pwm);
  }

  if (!r.moved && r.drove) {
    // The motor drove but the encoder saw (almost) nothing — a dead encoder, or an
    // abort right at launch. The firmware can't tell whether the car moved, so it
    // must not claim "no run": the event supervisor decides.
    printf("[SM] The motor drove but the encoder saw under %.0f mm of travel. The car\n"
           "[SM] may have moved, so this may count as a run - ask the event supervisor.\n",
           1000.0 * MOVED_MIN_M);
    printf("%s", next);
    uiShow("NO COUNTS", "ask ES: moved?");
    return;
  }
  if (!r.moved) {
    // §4.d.vii.4.a: a vehicle that doesn't move when triggered has not had a run.
    printf("[SM] The car never moved: under the rules no run occurred. Fix and re-run.\n");
    printf("%s", next);
    char l2[17];
    snprintf(l2, sizeof l2, "ABORT %s", abortName(r.abort));
    uiShow("NO RUN (no move)", l2);
    return;
  }

  const float mpErrCm  = (r.xEnd - p.xFinal) * 100.0f;   // + = stopped beyond Target
  const float fwdErrCm = (r.xMax - p.xF) * 100.0f;       // + = pushed further than planned
  printf("[SM] forward stop: planned %.3f  reached %.3f  (%+.1f cm)\n", p.xF, r.xMax, fwdErrCm);
  // Where the car thinks the bottle's near side ended: the pusher face at the furthest
  // forward point, plus the slide the plan allows for. Scored from the Bottle Line.
  const float btlPastCm = (r.xMax + L_P_M + S_SLIDE_M - (p.in.D + p.in.B)) * 100.0f;
  printf("[SM] bottle (est.): near side %.1f cm %s the Bottle Line (aimed %.1f cm past)\n",
         std::fabs(btlPastCm), btlPastCm >= 0.0f ? "PAST" : "SHORT of",
         100.0f * (p.in.margin + p.in.btlOfs));
  printf("[SM] final MP:     planned %.3f  encoder %.3f  (%+.1f cm)\n", p.xFinal, r.xEnd, mpErrCm);
  if (r.abort != Abort::None) {
    // An aborted run stopped early: its time and stop positions say nothing about
    // T_BIAS_S or the offsets. Don't offer numbers someone might average in.
    printf("[SM] Aborted (%s): no T_BIAS_S or offset suggestions from this run.\n",
           abortName(r.abort));
  } else if (s_sim) {
    // A model, not the car: its times and stops must never be averaged into
    // T_BIAS_S or the offsets.
    printf("[SM] time: run %.2f s (T %.1f)  motion %.2f s (est %.2f)  - simulated, not for T_BIAS_S\n",
           r.runTime, p.in.T, r.motionTime, p.tMotion);
  } else {
    printf("[SM] time: run %.2f s (T %.1f)  motion %.2f s (est %.2f -> T_BIAS_S would be %+.2f)\n",
           r.runTime, p.in.T, r.motionTime, p.tMotion,
           r.motionTime - (p.tMotion - T_BIAS_S));
    // 1 cm sideways at the Target = atan(0.01 / D), in degrees.
    const float degPerCm = 0.01f / p.in.D * 180.0f / (float)M_PI;
    printf("[SM] The encoder numbers only show what the car THINKS it did. Measure with\n"
           "[SM] a tape and enter the corrections for run 2 (config.h §5 conventions):\n"
           "[SM]   new MPofs  = %+.3f - (MP position - Target), in m\n"
           "[SM]                e.g. MP 3 cm beyond the Target -> %+.3f\n"
           "[SM]   new BtlOfs = %+.3f + (BtlPast %.3f - measured bottle past the line), in m\n"
           "[SM]                (BtlPast %.1f cm is where you WANT it; BtlOfs fixes the miss)\n"
           "[SM]   new Aim    = %+.2f + %.3f per cm the MP ended RIGHT of the Target (- per cm LEFT), deg\n"
           "[SM]                e.g. 3 cm right -> %+.2f\n",
           p.in.mpOfs, p.in.mpOfs - 0.030, p.in.btlOfs, p.in.margin, 100.0f * p.in.margin,
           p.in.aimDeg, degPerCm, p.in.aimDeg + 3.0f * degPerCm);
  }
  printf("%s", next);

  char l1[17], l2[17];
  if (s_sim) snprintf(l1, sizeof l1, "SIM%+5.1fcm t%4.1f", mpErrCm, r.runTime);
  else       snprintf(l1, sizeof l1, "MP%+5.1fcm t%5.2f", mpErrCm, r.runTime);
  if (r.abort != Abort::None) snprintf(l2, sizeof l2, "ABORT %s", abortName(r.abort));
  else snprintf(l2, sizeof l2, "F%+5.1fcm %s", fwdErrCm, r.imuLost ? "IMU!" : "ok");
  uiShow(l1, l2);
}

//==============================================================================
// Plan from the entered targets, print it, and show it on the LCD. Returns false
// (after showing why) if no legal plan exists.
static bool planAndShow(RunPlan &p) {
  p = planRun(s_in);
  printPlan(p);
  if (!p.valid) {
    uiShow("PLAN ERROR", p.error);
    sleep_ms(UI_MSG_MS);
    return false;
  }
  char l1[17], l2[17];
  snprintf(l1, sizeof l1, "xF%5.2f rev%4.2f", p.xF, p.rev);
  // Estimated Run Time (start delay + motion): T when it fits, later when it can't.
  snprintf(l2, sizeof l2, "est%5.1fs %s", p.delayS + p.tMotion, p.timeFits ? "ok" : "LATE");
  uiShow(l1, l2);
  sleep_ms(UI_PLAN_MS);
  return true;
}

//==============================================================================
void smDryRun() {
  if (const char *e = preflightError()) {
    printf("[SM] NOT CALIBRATED: %s — a dry run needs it too.\n", e);
    uiShow("NOT CALIBRATED", e);
    sleep_ms(UI_MSG_MS);
    return;
  }
  enterSettings(s_in, s_level);
  RunPlan p{};
  if (!planAndShow(p)) return;
  // The motor really runs. With the wheels on the floor the car would drive off
  // on a distance it cannot measure (it aborts once the real roller sees
  // SIM_REAL_MOTION_M, but by then it has moved).
  printf("[SM] SIM DRY RUN: wheels must be OFF the ground. SET = go on, hold SET = back.\n");
  uiShow("SIM: WHEELS UP?", "SET=ok hold=back");
  if (waitSetPress()) return;       // long press: back out (it waits for the release)

  odoSimSet(true);
  s_sim = true;
  const RunResult r = executeRun(p, imuInitOk());
  s_sim = false;
  odoSimSet(false);
  if (!r.cancelled) {
    printRunId(*s_rec);
    telemPrint();
    report(p, r);
    remember(p, r);
    (void)waitSetPress(true);       // keep the result up until it has been read
  }
  driveCoast();                     // back to the bench tools: free the wheels
}

//==============================================================================
void smRun(bool imuOk) {
  RunInputs &in = s_in;
  int &level = s_level;
  bool showReady = true;

  while (true) {
    // ---- SETUP: short SET starts entry, long SET opens the bench menu ----------
    // After a run the result screen stays up and doubles as this prompt.
    if (showReady) uiShow("EV ready", "SET=run hold=cal");
    showReady = true;
    if (waitSetPress(true)) {
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
    RunPlan p{};
    if (!planAndShow(p)) continue;
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
    printRunId(*s_rec);
    telemPrint();
    report(p, r);
    remember(p, r);
    showReady = false;
  }
}
