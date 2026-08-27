#include "pose.h"
#include "config.h"
#include "imu.h"
#include "odo.h"
#include "utils.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include <atomic>
#include <cmath>

static mutex_t s_lock;
static Position s_state{0, 0, 0};
// IMU yaw (rad) captured at ARM. Written by core0 (poseZeroAtArm), read by core1
// (poseUpdate) outside the mutex, so it must be atomic. float rather than double
// because a 4-byte atomic is lock-free on 32-bit ARM while an 8-byte one is not —
// and ~7 significant digits is ample for a heading offset.
static std::atomic<float> s_headingOffsetRad{0.0f};
// Written by core0 in poseSetActive(true) BEFORE s_active goes true, and only by
// core1 thereafter. The seq_cst store of s_active publishes it (release/acquire),
// so no separate synchronisation is needed.
static float    s_lastDist = 0.0f;          // last odo distance (m) for ds
static std::atomic<bool> s_active{false};   // estimator gate (see pose.h)

// HEADING_SIGN lives in config.h §3 with the other three bench signs.

void poseInit() {
  mutex_init(&s_lock);
}

void poseZeroAtArm() {
  mutex_enter_blocking(&s_lock);
  odoZero();
  odoSpeedReset();
  s_lastDist = 0.0f;
  s_headingOffsetRad.store((float)utils::degToRad(imuYawDeg()));  // current yaw becomes 0
  s_state = Position{0, 0, 0};
  mutex_exit(&s_lock);
}

void poseSetActive(bool on) {
  if (on) {
    s_lastDist = odoDistanceM();   // avoid a bogus first-tick ds
    odoSpeedReset();               // ...and a bogus first-tick speed
  }
  s_active.store(on);
}

void poseUpdate() {
  if (!s_active.load()) return;          // inactive: don't touch the shared I2C bus
  odoSpeedTick();                        // ground-speed estimate (self-rate-limited)
  // heading relative to ARM, wrapped to [0, 2pi) then to a continuous value
  double rawRad = utils::degToRad(imuYawDeg());
  double theta  = HEADING_SIGN * (rawRad - (double)s_headingOffsetRad.load());
  theta = utils::angleSquish(theta, true);   // [0, 2pi)

  float dist = odoDistanceM();
  float ds   = dist - s_lastDist;
  s_lastDist = dist;

  mutex_enter_blocking(&s_lock);
  s_state.x    += ds * std::cos(theta);
  s_state.y    += ds * std::sin(theta);
  s_state.theta = theta;
  mutex_exit(&s_lock);
}

Position poseGet() {
  mutex_enter_blocking(&s_lock);
  Position p = s_state;
  mutex_exit(&s_lock);
  return p;
}

float poseHeadingRad() {
  return poseGet().theta;
}
