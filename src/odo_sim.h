#pragma once
//==============================================================================
// odo_sim — the simulated free roller for "8 Sim dry run" (config.h §7). Pure
// logic, header-only, tested in host_check.
//
// With the wheels up the real roller never turns, so a run would abort on the
// dead-encoder watchdog half a second after launch. In a dry run odo.cpp feeds
// the direction split from this model instead: a speed that follows the motor
// command through the feedforward table (run backwards) with a first-order lag,
// and decelerates when the drive coasts or brakes. Everything downstream — the
// speed loop, the watchdogs, the pose, the stop — runs unchanged on its counts.
//==============================================================================
#include "config.h"
#include "drive.h"
#include "speed.h"
#include <algorithm>
#include <cmath>

struct SimWheel {
  float  v      = 0.0f;   // simulated ground speed, m/s, signed (+ forward)
  double counts = 0.0;    // simulated roller position, fractional counts

  // Steady speed the drive gives at signed duty `pct`: the feedforward table
  // inverted, scaled. At or below its first PWM the car doesn't move.
  static float speedFor(float pct) {
    const float mag = SIM_SPEED_SCALE * ffSpeedForPwm(std::fabs(pct));
    return pct >= 0.0f ? mag : -mag;
  }

  // Advance by dt seconds with the drive doing `mode` at signed duty `pct`.
  // mFwd / mRev turn metres into counts, as the real roller's scales do.
  void step(float dt, DriveMode mode, float pct, float mFwd, float mRev) {
    if (dt <= 0.0f || mFwd <= 0.0f || mRev <= 0.0f) return;
    dt = std::min(dt, SIM_MAX_STEP_S);
    if (mode == DriveMode::Drive) {
      v += (speedFor(pct) - v) * std::min(1.0f, dt / SIM_TAU_S);
    } else {
      const float dv = (mode == DriveMode::Hold ? SIM_BRAKE_DECEL_MPS2 : SIM_COAST_DECEL_MPS2) * dt;
      v = std::fabs(v) <= dv ? 0.0f : v - std::copysign(dv, v);
    }
    const double dx = (double)v * dt;
    counts += dx / (dx >= 0.0 ? mFwd : mRev);
  }
};
