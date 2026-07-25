#include "sm.h"
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

void smRun() {
  ArcPlan plan{};
  float targetD = 8.0f, targetT = 15.0f, bulgeH = 0.92f;

  while (true) {
    // ---- SETUP: enter D, T, gate bulge; plan the arc --------------------------
    uiShow("EV ready", "SET to enter");
    targetD = uiEnterDistanceM();     // TODO: real dial entry (ui.cpp)
    targetT = uiEnterTimeS();
    bulgeH  = uiEnterBulgeH();
    plan = pathPlan(targetD, bulgeH);
    printf("[SM] SETUP D=%.2f m T=%.1f s h=%.2f m -> R=%.2f L=%.3f k=%.4f\n",
           targetD, targetT, bulgeH, plan.R, plan.L, plan.k);
    steerInit();                      // centre wheels
    driveCoast();

    // ---- ARMED: impound-safe; zero pose; wait for the pencil trigger ----------
    uiShow("ARMED", "waiting START");
    poseZeroAtArm();                  // pose=(0,0,0); IMU heading ref = now
    steerSetAngle(std::atan(WHEELBASE_L_M * plan.k));  // pre-aim to arc tangent
    while (!uiStartPressed()) { sleep_ms(2); }

    // Start the estimator now: from here until BRAKE, core1 owns the I2C bus
    // (IMU) and core0 issues no LCD writes — so the shared bus is never contended.
    poseSetActive(true);

    // ---- LAUNCH: brief open-loop kick to get rolling --------------------------
    uint64_t t0 = to_ms_since_boot(get_absolute_time());
    driveForward(45.0f);              // TODO: pwm_launch, ~150 ms
    sleep_ms(150);

    // ---- CRUISE: follow the arc + hold the time schedule ----------------------
    bool aborted = false;
    float creepStart = plan.L - CREEP_ZONE_M;
    while (odoDistanceM() < creepStart) {
      if (!imuHealthy()) {
        // Lost the heading source: we can't steer safely. Abort to a controlled
        // stop rather than coast blind. (Skips APPROACH; BRAKE still runs below.)
        printf("[SM] IMU unhealthy during CRUISE — aborting run to a safe stop\n");
        aborted = true;
        break;
      }
      Position p = poseGet();
      float s = odoDistanceM();
      float measuredMps = 0.0f;       // TODO: derive from odo delta / dt (filtered)
      steerControl(plan, p, measuredMps);
      float t = (to_ms_since_boot(get_absolute_time()) - t0) / 1000.0f;
      driveForward(speedControl(s, plan.L, targetT, measuredMps));
      (void)t;
      sleep_ms(CONTROL_TICK_US / 1000);   // ~control tick
    }

    // ---- APPROACH: closed-loop precise stop (the #1 accuracy win) -------------
    // Creep toward the target while polling the encoder, with a stall guard: if the
    // car isn't making progress each window, step the creep PWM up until it moves.
    if (!aborted) {
      printf("[SM] APPROACH: creeping to target\n");  // serial only (no LCD mid-run)
      float creepPwm = CREEP_PWM;
      float lastProgressDist = odoDistanceM();
      uint64_t lastProgressT = to_ms_since_boot(get_absolute_time());
      while (odoDistanceM() < plan.L) {
        Position p = poseGet();
        steerControl(plan, p, 0.0f);    // keep holding the arc
        driveForward(creepPwm);         // low, guaranteed-to-move
        uint64_t now = to_ms_since_boot(get_absolute_time());
        if (now - lastProgressT >= CREEP_STALL_WINDOW_MS) {
          float movedCm = (odoDistanceM() - lastProgressDist) * 100.0f;
          if (movedCm < CREEP_STALL_MIN_CM && creepPwm < CREEP_PWM_MAX) {
            creepPwm = std::min(creepPwm + CREEP_PWM_STEP, CREEP_PWM_MAX);
            printf("[SM] creep stall (%.2f cm/window) — bumping PWM to %.1f%%\n", movedCm, creepPwm);
          }
          lastProgressDist = odoDistanceM();
          lastProgressT = now;
        }
        sleep_ms(CONTROL_TICK_US / 1000);
      }
    }

    // ---- BRAKE ---------------------------------------------------------------
    poseSetActive(false);   // estimator off — core0 may use the LCD again
    driveBrake();
    sleep_ms(BRAKE_PULSE_MS);
    driveCoast();

    // ---- DONE: report for run-2 offset ---------------------------------------
    sleep_ms(500);
    float finalDist = odoDistanceM();
    float runTime   = (to_ms_since_boot(get_absolute_time()) - t0) / 1000.0f;
    printf("[SM] DONE  finalArc=%.3f m (target %.3f)  time=%.2f s (target %.1f)\n",
           finalDist, plan.L, runTime, targetT);
    char l1[17], l2[17];
    snprintf(l1, sizeof l1, "Fin %.2fm", finalDist);
    snprintf(l2, sizeof l2, "t %.1fs", runTime);
    uiShow(l1, l2);

    // loop back to SETUP for the second run
  }
}
