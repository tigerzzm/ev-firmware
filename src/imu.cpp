//==============================================================================
// IMU driver implementation — BNO085 Game Rotation Vector, gyro+accel (mag-free)
// Adapted from robotour-pico src/imu.cpp. See imu.h for the change list.
//==============================================================================
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "BNO08x/Adafruit_BNO08x.h"
#include "BNO08x/sh2.h"            // SH2_GAME_ROTATION_VECTOR
#include "BNO08x/sh2_SensorValue.h"
#include "config.h"
#include "imu.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <atomic>

static euler_t s_ypr;
static Adafruit_BNO08x *s_imu = new Adafruit_BNO08x(BNO085_RST_PIN);

// Report we use as the SINGLE heading source.
static constexpr uint8_t GAME_RV = SH2_GAME_ROTATION_VECTOR;  // 0x08

// --- quaternion -> euler (unchanged from robotour) ---------------------------
static void quaternionToEuler(float qr, float qi, float qj, float qk,
                              euler_t *ypr, bool degrees) {
  float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;
  ypr->yaw   = std::atan2(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = std::asin(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll  = std::atan2(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  if (degrees) {
    ypr->yaw   *= 180.0f / (float)M_PI;
    ypr->pitch *= 180.0f / (float)M_PI;
    ypr->roll  *= 180.0f / (float)M_PI;
  }
}

// --- health/state tracking -----------------------------------------------------
// imuYawDeg() runs on core1 during a run; imuHealthy() and imuResetCount() are
// read on core0. The fields both cores touch are atomic. (The M0+ has no atomic
// instructions; the SDK's pico_atomic supplies them.)
static std::atomic<bool>     s_initialized{false};
static std::atomic<bool>     s_has_valid{false};
static std::atomic<uint32_t> s_last_valid_ms{0};
static std::atomic<uint32_t> s_reset_count{0};   // chip resets seen since boot
static bool     s_init_ok = false;        // imuInit() completed successfully
static bool     s_begun = false;          // begin_I2C succeeded: safe to poll the chip
static uint32_t s_fail_count = 0;

static bool enableHeadingReport() {
  // 10 ms report interval (100 Hz).
  return s_imu->enableReport(GAME_RV, 10'000);
}

// Serial output from here can block for up to ~500 ms (USB CDC with nobody
// reading), and during a run this runs on core1 inside the estimator. While the
// estimator is active (imuSetQuiet), messages are dropped instead.
static std::atomic<bool> s_quiet{false};
static void imuLog(const char *fmt, ...) {
  if (s_quiet.load()) return;
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}
void imuSetQuiet(bool quiet) { s_quiet.store(quiet); }

static uint32_t s_last_retry_ms = 0;
// True once any valid sample has arrived. Every "no new data" path returns the
// last good yaw from then on — never 0, which would be a heading the car never had.
// Atomic: written by core1, read by core0 (imuSampleAgeMs).
static std::atomic<bool> s_ever_valid{false};
static uint32_t s_valid_samples = 0;   // valid samples since boot (imuInit validation)
// Set by a reset, cleared by the first valid sample after it. enableReport() only
// confirms the command was SENT — the chip doesn't acknowledge it — so "enable
// returned true" is not proof of recovery; a fresh sample is.
static std::atomic<bool> s_awaiting_data{false};   // read by core0 (imuRecovering)
static bool s_enable_sent   = true;   // the enable command went out since the last reset
static bool s_initing       = false;  // inside imuInit(), after begin_I2C succeeded

float imuYawDeg() {
  if (!s_begun) return 0.0f;       // chip never answered: don't touch the bus
  // Recover if the chip reset itself. After a reset the Game Rotation Vector
  // restarts from a NEW reference, so any heading zero taken before it is wrong;
  // sm.cpp watches imuResetCount() for that. Returning the last good yaw (not 0)
  // keeps the estimator from seeing a sudden jump meanwhile.
  if (s_imu->wasReset()) {
    imuLog("IMU was reset - re-enabling Game Rotation Vector...\n");
    s_initialized = false;
    s_has_valid   = false;
    s_reset_count.fetch_add(1);
    s_fail_count++;
    s_last_retry_ms = 0;           // re-enable on this call, below
    s_awaiting_data = true;
    s_enable_sent   = false;
  }
  // Re-enable the report after a reset, and keep retrying every IMU_RETRY_MS until
  // a sample actually arrives (an enable sent while the hub is still booting is
  // dropped silently). This also runs DURING imuInit(): the power-on reset is
  // usually first seen in its validation reads, and without a re-enable there the
  // chip never streams and init fails. Only a chip whose init fully SUCCEEDED is
  // marked ready again, so one that fails validation stays not-ready.
  if ((s_init_ok || s_initing) && s_awaiting_data) {
    const uint32_t t = to_ms_since_boot(get_absolute_time());
    if (s_last_retry_ms == 0 || t - s_last_retry_ms >= IMU_RETRY_MS) {
      s_last_retry_ms = t ? t : 1;
      if (enableHeadingReport()) {
        s_enable_sent = true;
        // imuHealthy() still reads false until a fresh sample arrives
        // (s_has_valid), and the next START re-zeroes the heading against the
        // chip's new reference.
        if (s_init_ok) s_initialized = true;
      } else {
        imuLog("ERROR: re-enabling Game Rotation Vector failed (%lu); will retry\n",
               (unsigned long)s_fail_count);
      }
    }
    if (!s_enable_sent) return s_ypr.yaw;   // enable not sent yet: nothing to read
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  if (s_has_valid && (now - s_last_valid_ms) > IMU_WARN_MS) {
    imuLog("WARNING: IMU timeout - no data for %lu ms\n", (unsigned long)IMU_WARN_MS);
    s_has_valid = false;
    s_fail_count++;
  }

  sh2_SensorValue_t event;
  if (!s_imu->getSensorEvent(&event)) {
    // no new event: hold last good, or 0 if we never had data
    return s_ever_valid ? s_ypr.yaw : 0.0f;
  }

  if (event.sensorId == GAME_RV) {
    float qr = event.un.gameRotationVector.real;
    float qi = event.un.gameRotationVector.i;
    float qj = event.un.gameRotationVector.j;
    float qk = event.un.gameRotationVector.k;

    if (qr == 0.0f && qi == 0.0f && qj == 0.0f && qk == 0.0f) {
      imuLog("WARNING: IMU zero quaternion\n");
      return s_ever_valid ? s_ypr.yaw : 0.0f;
    }

    // Convert into a temporary and keep it only if it is finite. Writing s_ypr
    // first meant one bad packet replaced the last good yaw with NaN, which every
    // "keep last good" path then returned — and NaN in the heading poisons the
    // pose (x, y) for the rest of the run.
    euler_t ypr;
    quaternionToEuler(qr, qi, qj, qk, &ypr, true);
    if (!std::isfinite(ypr.yaw)) {
      imuLog("WARNING: IMU non-finite yaw\n");
      return s_ever_valid ? s_ypr.yaw : 0.0f;
    }
    s_ypr = ypr;

    s_has_valid     = true;
    s_last_valid_ms = now;
    s_fail_count    = 0;
    s_awaiting_data = false;         // recovered for real
    s_ever_valid    = true;
    s_valid_samples++;
    return s_ypr.yaw;
  }

  // Any other report we didn't ask for: ignore, keep last good.
  return s_ever_valid ? s_ypr.yaw : 0.0f;
}

bool imuInit() {
  printf("Initializing BNO085 (Game Rotation Vector, mag-free)...\n");
  s_fail_count = 0;
  s_has_valid  = false;
  s_init_ok    = false;

  // I2C bring-up with retries.
  const int MAX_INIT_ATTEMPTS = 10;
  int attempts = 0;
  for (; attempts < MAX_INIT_ATTEMPTS; attempts++) {
    if (s_imu->begin_I2C(BNO08x_I2CADDR_DEFAULT, i2c0, I2C_SDA_PIN, I2C_SCL_PIN)) {
      printf("IMU I2C connection established (SDA=GP%d, SCL=GP%d).\n",
             I2C_SDA_PIN, I2C_SCL_PIN);
      s_begun = true;
      break;
    }
    printf("IMU I2C init failed, retry %d/%d...\n", attempts + 1, MAX_INIT_ATTEMPTS);
    sleep_ms(200);
  }
  if (attempts >= MAX_INIT_ATTEMPTS) {
    printf("ERROR: IMU I2C init failed. Check wiring / 3.3V / 4.7k pull-ups.\n");
    return false;
  }

  s_initing = true;                 // resets from here on get re-enabled (imuYawDeg)

  // Enable the single heading report, with retries.
  const int MAX_REPORT_ATTEMPTS = 5;
  bool enabled = false;
  for (int i = 0; i < MAX_REPORT_ATTEMPTS; i++) {
    if (enableHeadingReport()) { enabled = true; break; }
    printf("Failed to enable Game Rotation Vector, retry %d/%d...\n", i + 1, MAX_REPORT_ATTEMPTS);
    sleep_ms(100);
  }
  if (!enabled) {
    s_initing = false;
    printf("ERROR: Could not enable Game Rotation Vector.\n");
    return false;
  }

  // Let it start streaming, then validate a few reads.
  sleep_ms(1000);
  int valid = 0;
  const int REQUIRED = 3;
  for (int i = 0; i < 20 && valid < REQUIRED; i++) {
    // Count only reads that brought a NEW sample: s_has_valid stays true after one
    // real sample, so counting it would pass on one sample plus two stale repeats.
    const uint32_t before = s_valid_samples;
    (void)imuYawDeg();
    if (s_valid_samples != before) valid++;
    sleep_ms(50);
  }
  s_initing = false;
  if (valid < REQUIRED) {
    printf("ERROR: IMU validation failed (%d/%d valid reads).\n", valid, REQUIRED);
    return false;
  }

  s_init_ok     = true;
  s_initialized = true;
  printf("IMU ready.\n");
  return true;
}

bool imuInitOk() {
  return s_init_ok;
}

bool imuRecovering() {
  return s_awaiting_data.load();
}

uint32_t imuSampleAgeMs() {
  if (!s_ever_valid) return UINT32_MAX;
  const uint32_t last = s_last_valid_ms.load();      // timestamp first (see imuHealthy)
  return to_ms_since_boot(get_absolute_time()) - last;
}

uint32_t imuResetCount() {
  return s_reset_count.load();
}

bool imuHealthy() {
  if (!s_initialized.load() || !s_has_valid.load()) return false;
  // Timestamp FIRST, then the clock. The other order lets core1 store a newer
  // timestamp in between, and `now - last` then wraps to ~4e9: a healthy IMU
  // reads as stale.
  const uint32_t last = s_last_valid_ms.load();
  const uint32_t now  = to_ms_since_boot(get_absolute_time());
  return (now - last) < IMU_STALE_MS;
}
