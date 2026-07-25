#include "pose.h"
#include "imu.h"
#include "odo.h"
#include "utils.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include <atomic>
#include <cmath>

static mutex_t s_lock;
static Position s_state{0, 0, 0};
static double   s_headingOffsetRad = 0.0;   // IMU yaw (rad) captured at ARM
static float    s_lastDist = 0.0f;          // last odo distance (m) for ds
static std::atomic<bool> s_active{false};   // estimator gate (see pose.h)

// Sign of the heading. robotour flipped IMU sense to match its frame; the EV's
// sign is confirmed on the bench (turn the car left by hand, heading should
// increase). Flip this if the bench test disagrees.
static constexpr double HEADING_SIGN = +1.0;

void poseInit() {
  mutex_init(&s_lock);
}

void poseZeroAtArm() {
  mutex_enter_blocking(&s_lock);
  odoZero();
  s_lastDist = 0.0f;
  s_headingOffsetRad = utils::degToRad(imuYawDeg());  // current yaw becomes 0
  s_state = Position{0, 0, 0};
  mutex_exit(&s_lock);
}

void poseSetActive(bool on) {
  if (on) s_lastDist = odoDistanceM();   // avoid a bogus first-tick ds
  s_active.store(on);
}

void poseUpdate() {
  if (!s_active.load()) return;          // inactive: don't touch the shared I2C bus
  // heading relative to ARM, wrapped to [0, 2pi) then to a continuous value
  double rawRad = utils::degToRad(imuYawDeg());
  double theta  = HEADING_SIGN * (rawRad - s_headingOffsetRad);
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
