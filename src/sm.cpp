#include "sm.h"
#include "cal.h"
#include "config.h"
#include "drive.h"
#include "imu.h"
#include "odo.h"
#include "path.h"
#include "pose.h"
#include "speed.h"
#include "steer.h"
#include "ui.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

// The estimator (poseUpdate) runs continuously on core1 — see main.cpp.
// Here on core0 we sequence the run and issue drive/steer commands.

// Why a run ended. Everything except None skips APPROACH; BRAKE always runs.
enum class Abort { None, Imu, Operator, Timeout };

static const char *abortName(Abort a) {
  switch (a) {
    case Abort::Imu:      return "IMU lost";
    case Abort::Operator: return "SET pressed";
    case Abort::Timeout:  return "run timeout";
    default:              return "none";
  }
}

static inline float elapsedS(uint64_t t0) {
  return (to_ms_since_boot(get_absolute_time()) - t0) / 1000.0f;
}

void smRun() {
  ArcPlan plan{};
  float targetD = 8.0f, targetT = 15.0f, bulgeH = 0.92f;
  // Carried BETWEEN runs: run 2 starts at whatever run 1 used, so the operator
  // adjusts rather than re-entering from zero inside the 8-minute window.
  float stopOffset = 0.0f;

  while (true) {
    // ---- SETUP: enter D, T, gate bulge; plan the arc --------------------------
    // Gate before target entry. A short SET starts a run; holding SET drops into
    // the bench menu. Previously "SET to enter" was shown and then immediately
    // overwritten by the first entry screen — the prompt did not actually wait.
    uiShow("EV ready", "SET=run hold=cal");
    {
      while (uiSetPressed()) sleep_ms(5);      // ignore a press still held
      while (!uiSetPressed()) sleep_ms(2);     // wait for a fresh press
      const uint64_t tDown = to_ms_since_boot(get_absolute_time());
      bool longPress = false;
      while (uiSetPressed()) {
        if (to_ms_since_boot(get_absolute_time()) - tDown >= CAL_ENTRY_HOLD_MS) {
          longPress = true;
          break;
        }
        sleep_ms(5);
      }
      while (uiSetPressed()) sleep_ms(5);      // wait for release either way
      if (longPress) {
        printf("[SM] SET held -> bench menu\n");
        uiShow("Bench menu", "release SET");
        calMenu();                             // returns via "5 Exit to run"
      }
    }

#if USE_DIAL_ENTRY
    targetD = uiEnterDistanceM();     // dial entry (ui.cpp enterValue)
    targetT = uiEnterTimeS();
    bulgeH  = uiEnterBulgeH();
    stopOffset = uiEnterStopOffsetM(stopOffset);
#else
    // No dial: the targets are compile-time constants. Edit config.h §5b and
    // reflash to change them. Clamped here so a bad edit can't plan an illegal
    // arc. Shown on the LCD before ARMED so the operator can still verify.
    targetD    = std::clamp(DEFAULT_TARGET_D_M, TARGET_DIST_MIN_M, TARGET_DIST_MAX_M);
    targetT    = std::clamp(DEFAULT_TARGET_T_S, TARGET_TIME_MIN_S, TARGET_TIME_MAX_S);
    bulgeH     = GATE_BULGE_H_M;
    stopOffset = std::clamp(DEFAULT_STOP_OFFSET_M, STOP_OFFSET_MIN_M, STOP_OFFSET_MAX_M);
    {
      char d1[17], d2[17];
      snprintf(d1, sizeof d1, "D%.2f T%.1f", targetD, targetT);
      snprintf(d2, sizeof d2, "Ofs %+.3f m", stopOffset);
      uiShow(d1, d2);
      sleep_ms(2000);
    }
#endif
    plan = pathPlan(targetD, bulgeH);
    // Stop target = planned arc length + the operator's empirical correction.
    // The offset is applied HERE and not inside pathPlan() on purpose: feeding a
    // fudged D into the planner would change R and the curvature, moving the line
    // the car takes through the can gate.
    const float stopTarget  = plan.L + stopOffset;
    const float runTimeoutS = targetT * RUN_TIMEOUT_FACTOR + RUN_TIMEOUT_PAD_S;
    printf("[SM] SETUP D=%.2f m T=%.1f s h=%.2f m -> R=%.2f L=%.3f k=%.4f\n",
           targetD, targetT, bulgeH, plan.R, plan.L, plan.k);
    printf("[SM] stopOffset=%+.3f m -> stopTarget=%.3f m; timeout %.1f s\n",
           stopOffset, stopTarget, runTimeoutS);
    // Centre the wheels. NOT steerInit() — main() has already configured the PWM
    // slice, and re-running pwm_init here would reset the slice counter and glitch
    // the servo frame on every pass through SETUP.
    steerSetAngle(0.0f);
    driveCoast();

    // ---- ARMED: impound-safe; zero pose; wait for the pencil trigger ----------
    uiShow("ARMED", "waiting START");
    poseZeroAtArm();                  // pose=(0,0,0); IMU heading ref = now
    steerSetAngle(std::atan(WHEELBASE_L_M * plan.k));  // pre-aim to arc tangent
    while (!uiStartPressed()) { sleep_ms(2); }

    // Start the estimator. The IMU (I2C0, core1) and the LCD (I2C1, core0) are on
    // SEPARATE controllers, so both may be used concurrently — uiShowRun() below
    // writes the display mid-run. poseSetActive is now purely the estimator gate.
    poseSetActive(true);

    // ---- LAUNCH: brief open-loop kick to get rolling --------------------------
    uint64_t t0 = to_ms_since_boot(get_absolute_time());
    driveForward(LAUNCH_PWM);
    sleep_ms(LAUNCH_MS);

    // ---- CRUISE: follow the arc + hold the time schedule ----------------------
    Abort aborted = Abort::None;
    float creepStart = stopTarget - CREEP_ZONE_M;
    // The LCD is on its own bus (I2C1) so writing it mid-run cannot collide with
    // the IMU on I2C0. Throttled because an LCD write is slow, not because it is
    // unsafe — see LCD_RUN_UPDATE_MS.
    uint32_t lastLcdMs = 0;
    while (odoDistanceM() < creepStart) {
      // Lost the heading source: we can't steer safely, so stop rather than
      // coast blind. (Any abort skips APPROACH; BRAKE still runs below.)
      if (!imuHealthy())            { aborted = Abort::Imu;      break; }
      // Operator kill switch. GPIO read only — touches no I2C, so this is safe
      // while the estimator owns the bus.
      if (uiSetPressed())           { aborted = Abort::Operator; break; }
      // Runaway guard: an uncalibrated M_PER_COUNT reads 0 distance forever, so
      // the loop condition above would never go false.
      if (elapsedS(t0) > runTimeoutS) { aborted = Abort::Timeout; break; }
      Position p = poseGet();
      float s = odoDistanceM();
      float measuredMps = odoSpeedMps();   // filtered free-roller ground speed
      steerControl(plan, p, measuredMps);
      // stopTarget, not plan.L: the car stops at stopTarget, so the trapezoid must
      // cover THAT distance in targetT for the arrival time to be right. With a
      // large stop offset (range is +/-0.50 m, up to 6% of an 8 m run) using plan.L
      // would put the decel ramp in the wrong place.
      driveForward(speedControl(s, stopTarget, targetT, measuredMps));
      if (LCD_RUN_UPDATE_MS) {
        const uint32_t nowMs = to_ms_since_boot(get_absolute_time());
        if (nowMs - lastLcdMs >= LCD_RUN_UPDATE_MS) {
          uiShowRun(s, stopTarget, measuredMps, elapsedS(t0));
          lastLcdMs = nowMs;
        }
      }
      // NOTE: nothing here compares elapsedS(t0) against a time schedule — the
      // longitudinal loop is still pure feedforward. See speed.cpp.
      sleep_ms(CONTROL_TICK_US / 1000);   // ~control tick
    }

    // ---- APPROACH: closed-loop precise stop (the #1 accuracy win) -------------
    // Creep toward the target while polling the encoder, with a stall guard: if the
    // car isn't making progress each window, step the creep PWM up until it moves.
    if (aborted == Abort::None) {
      printf("[SM] APPROACH: creeping to target\n");  // serial only (no LCD mid-run)
      float creepPwm = CREEP_PWM;
      int   creepBumps = 0;            // reported after the loop, not inside it
      float lastProgressDist = odoDistanceM();
      uint64_t lastProgressT = to_ms_since_boot(get_absolute_time());
      while (odoDistanceM() < stopTarget) {
        if (uiSetPressed())             { aborted = Abort::Operator; break; }
        if (elapsedS(t0) > runTimeoutS) { aborted = Abort::Timeout;  break; }
        Position p = poseGet();
        steerControl(plan, p, odoSpeedMps());   // keep holding the arc
        driveForward(creepPwm);         // low, guaranteed-to-move
        if (LCD_RUN_UPDATE_MS) {
          const uint32_t nowMs = to_ms_since_boot(get_absolute_time());
          if (nowMs - lastLcdMs >= LCD_RUN_UPDATE_MS) {
            uiShowRun(odoDistanceM(), stopTarget, odoSpeedMps(), elapsedS(t0));
            lastLcdMs = nowMs;
          }
        }
        uint64_t now = to_ms_since_boot(get_absolute_time());
        if (now - lastProgressT >= CREEP_STALL_WINDOW_MS) {
          float movedCm = (odoDistanceM() - lastProgressDist) * 100.0f;
          if (movedCm < CREEP_STALL_MIN_CM && creepPwm < CREEP_PWM_MAX) {
            creepPwm = std::min(creepPwm + CREEP_PWM_STEP, CREEP_PWM_MAX);
            creepBumps++;
            // Deliberately NOT printed here: stdio over USB CDC can block for up
            // to PICO_STDIO_USB_STDOUT_TIMEOUT_US (500 ms by default) when the
            // host is not draining the pipe. Stalling the loop for half a second
            // in the middle of the precise stop is far worse than losing the log
            // line. Reported once after the loop instead.
          }
          lastProgressDist = odoDistanceM();
          lastProgressT = now;
        }
        sleep_ms(CONTROL_TICK_US / 1000);
      }
      if (creepBumps) {
        printf("[SM] creep stall guard fired %d time(s); final creep PWM %.1f%%\n",
               creepBumps, creepPwm);
      }
    }

    // ---- BRAKE ---------------------------------------------------------------
    if (aborted != Abort::None) printf("[SM] ABORT: %s\n", abortName(aborted));
    poseSetActive(false);   // estimator off — core0 may use the LCD again
    driveBrake();
    sleep_ms(BRAKE_PULSE_MS);
    driveCoast();
    // Stamp the run time HERE — the brake pulse is part of the run, but the
    // settle delay below is not. Timing after that sleep over-reported by ~600 ms,
    // which would have skewed the run-2 time correction.
    const float runTime = elapsedS(t0);

    // ---- DONE: report for run-2 offset ---------------------------------------
    sleep_ms(500);          // let it settle before reading the final distance
    float finalDist = odoDistanceM();
    printf("[SM] DONE  finalArc=%.3f m (target %.3f)  time=%.2f s (target %.1f)  abort=%s\n",
           finalDist, stopTarget, runTime, targetT, abortName(aborted));
    // NOTE: finalArc is what the ENCODER read, so it always lands near stopTarget
    // and cannot tell you your score. Measure the real error with a tape
    // (Measurement Point -> Target Point) and enter it negated as the offset next
    // run: stopped 3 cm past the target -> -0.030.
    printf("[SM] next run: measure MP->Target, enter negated as Ofs (now %+.3f m)\n",
           stopOffset);
    char l1[17], l2[17];
    snprintf(l1, sizeof l1, "Fin %.2fm", finalDist);
    if (aborted != Abort::None) snprintf(l2, sizeof l2, "ABORT %s", abortName(aborted));
    else                        snprintf(l2, sizeof l2, "t %.1fs", runTime);
    uiShow(l1, l2);

    // loop back to SETUP for the second run
  }
}
