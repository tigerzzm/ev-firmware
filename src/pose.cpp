#include "pose.h"
#include "config.h"
#include "imu.h"
#include "odo.h"
#include "steer.h"
#include "utils.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include <atomic>
#include <cmath>

static mutex_t s_lock;
static Position s_state{ENC_FROM_MP_M, 0, 0};
// IMU yaw (rad) at the heading zero. Written by whichever core performs the zero,
// read by core1 outside the mutex, so it is atomic.
static std::atomic<float> s_headingOffsetRad{0.0f};
// Last odo distance (m), for ds. Owned by core1 while the estimator is active;
// core0 writes it only while inactive (the seq_cst store of s_active publishes it).
static float    s_lastDist = 0.0f;
static std::atomic<bool> s_active{false};

// Heading-zero handshake. core0 raises s_zeroReq; core1 CLAIMS it with
// exchange(false) and raises s_zeroDone when finished. Because claiming is atomic,
// core0 can withdraw a request with the same exchange: whichever side gets `true`
// owns it, so a request core0 gave up on can never be served late, mid-run.
static std::atomic<bool> s_zeroReq{false};
static std::atomic<bool> s_zeroDone{false};
// True while core1 is inside an update. poseSetActive(false) waits for it to drop,
// so when that returns, core1 is out of imuYawDeg() and core0 owns the IMU.
static std::atomic<bool> s_busy{false};
static void updateActive();

void poseInit() {
  mutex_init(&s_lock);
}

// Roller back to (ENC_FROM_MP_M, 0, 0) with `yawDeg` as the new heading zero.
// Runs on whichever core owns the IMU right now. Never touches the I2C bus.
static void zeroPose(float yawDeg) {
  mutex_enter_blocking(&s_lock);
  s_lastDist = odoDistanceM();
  s_headingOffsetRad.store((float)utils::degToRad(yawDeg));
  s_state = Position{ENC_FROM_MP_M, 0, 0};
  mutex_exit(&s_lock);
}

bool poseZeroAtArm() {
  // The encoder is core-safe, so zero it here and now: the run's distance must
  // never depend on the IMU answering.
  odoZero();

  if (!s_active.load()) {
    // Estimator idle: this core may use the IMU. Nobody has been polling it, and
    // the BNO085 queues reports, so drain it before trusting the heading.
    float yaw = 0.0f;
    for (int i = 0; i < IMU_DRAIN_READS; i++) { yaw = imuYawDeg(); sleep_ms(IMU_DRAIN_SPACING_MS); }
    zeroPose(yaw);
    return true;
  }

  s_zeroDone.store(false);
  s_zeroReq.store(true);
  const uint32_t t0 = to_ms_since_boot(get_absolute_time());
  while (!s_zeroDone.load()) {
    if (to_ms_since_boot(get_absolute_time()) - t0 >= POSE_ZERO_TIMEOUT_MS) {
      if (s_zeroReq.exchange(false)) return false;   // withdrawn: core1 never claimed it
      // core1 claimed it at the last moment. It already holds its yaw sample, so
      // only a mutex stands between it and done — wait it out.
      while (!s_zeroDone.load()) sleep_us(50);
      return true;
    }
    sleep_ms(1);
  }
  return true;
}

void poseSetActive(bool on) {
  if (on) {
    s_lastDist = odoDistanceM();         // avoid a bogus first-tick ds
    imuSetQuiet(true);                   // core1 must not block on serial mid-run
    s_active.store(true);
    return;
  }
  s_active.store(false);
  // Core1 may already be past its gate check, mid I2C read. Wait for it to finish
  // that update before handing the IMU (and serial) back to this core. Bounded:
  // a core1 stuck on a hung bus must not hang the caller too.
  const uint32_t t0 = to_ms_since_boot(get_absolute_time());
  while (s_busy.load() && to_ms_since_boot(get_absolute_time()) - t0 < POSE_ZERO_TIMEOUT_MS)
    sleep_us(50);
  imuSetQuiet(false);
}

void poseUpdate() {
  // Announce FIRST, then check the gate. poseSetActive(false) does the mirror
  // image (clear the gate, then check busy), so with these sequentially consistent
  // atomics one side always sees the other: core1 can never start an update that
  // core0 thinks has not begun.
  s_busy.store(true);
  if (s_active.load()) updateActive();
  s_busy.store(false);
}

static void updateActive() {
  // Only poll a chip whose init succeeded: after a failed init imuYawDeg() would
  // just keep core1 busy on a dead bus. A chip recovering from a reset IS polled,
  // because polling is what retries its re-enable.
  const float yawDeg = imuInitOk() ? imuYawDeg() : 0.0f;

  // A heading zero requested by core0 (START pressed). Read the yaw FIRST, then
  // claim: once claimed, the zero completes without touching the bus.
  if (s_zeroReq.load() && s_zeroReq.exchange(false)) {
    zeroPose(yawDeg);
    s_zeroDone.store(true);
    return;
  }

  const double rawRad = utils::degToRad(yawDeg);
  const double theta  = utils::angleError(
      HEADING_SIGN * (rawRad - (double)s_headingOffsetRad.load()), 0.0, true);

  const float dist = odoDistanceM();
  const float ds   = dist - s_lastDist;
  s_lastDist = dist;
  const double dir = theta + (double)steerAngleRad();   // roller rolls along this

  mutex_enter_blocking(&s_lock);
  s_state.x    += ds * std::cos(dir);
  s_state.y    += ds * std::sin(dir);
  s_state.theta = theta;
  mutex_exit(&s_lock);
}

Position poseGet() {
  mutex_enter_blocking(&s_lock);
  Position p = s_state;
  mutex_exit(&s_lock);
  return p;
}
