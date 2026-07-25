//==============================================================================
// IMU driver implementation — BNO085 Game Rotation Vector, gyro+accel (mag-free)
// Adapted from robotour-pico src/imu.cpp. See imu.h for the change list.
//==============================================================================
#include <cmath>
#include <cstdio>

#include "BNO08x/Adafruit_BNO08x.h"
#include "BNO08x/sh2.h"            // SH2_GAME_ROTATION_VECTOR
#include "BNO08x/sh2_SensorValue.h"
#include "config.h"
#include "imu.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

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

// --- health/state tracking (unchanged logic from robotour) -------------------
static bool     s_initialized = false;
static bool     s_has_valid   = false;
static uint32_t s_last_valid_ms = 0;
static uint32_t s_fail_count = 0;
static const uint32_t IMU_TIMEOUT_MS = 5000;
static const uint32_t MAX_IMU_FAILURES = 3;

static bool enableHeadingReport() {
  // 10 ms report interval (100 Hz).
  return s_imu->enableReport(GAME_RV, 10'000);
}

float imuYawDeg() {
  // Recover if the chip reset itself.
  if (s_imu->wasReset()) {
    printf("IMU was reset - re-enabling Game Rotation Vector...\n");
    s_initialized = false;
    s_has_valid   = false;
    s_fail_count++;
    if (s_fail_count > MAX_IMU_FAILURES) {
      printf("ERROR: IMU has failed too many times (%lu). Check hardware!\n", (unsigned long)s_fail_count);
      return 0.0f;
    }
    if (!enableHeadingReport()) {
      printf("ERROR: Failed to re-enable Game Rotation Vector after reset!\n");
      return 0.0f;
    }
    sleep_ms(300);
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  if (s_has_valid && (now - s_last_valid_ms) > IMU_TIMEOUT_MS) {
    printf("WARNING: IMU timeout - no data for %lu ms\n", (unsigned long)IMU_TIMEOUT_MS);
    s_has_valid = false;
    s_fail_count++;
  }

  sh2_SensorValue_t event;
  if (!s_imu->getSensorEvent(&event)) {
    // no new event: hold last good, or 0 if we never had data
    return s_has_valid ? s_ypr.yaw : 0.0f;
  }

  if (event.sensorId == GAME_RV) {
    float qr = event.un.gameRotationVector.real;
    float qi = event.un.gameRotationVector.i;
    float qj = event.un.gameRotationVector.j;
    float qk = event.un.gameRotationVector.k;

    if (qr == 0.0f && qi == 0.0f && qj == 0.0f && qk == 0.0f) {
      printf("WARNING: IMU zero quaternion\n");
      return s_has_valid ? s_ypr.yaw : 0.0f;
    }

    quaternionToEuler(qr, qi, qj, qk, &s_ypr, true);

    if (!std::isfinite(s_ypr.yaw)) {
      printf("WARNING: IMU non-finite yaw\n");
      return s_has_valid ? s_ypr.yaw : 0.0f;
    }

    s_has_valid     = true;
    s_last_valid_ms = now;
    s_fail_count    = 0;
    return s_ypr.yaw;
  }

  // Any other report we didn't ask for: ignore, keep last good.
  return s_has_valid ? s_ypr.yaw : 0.0f;
}

bool imuInit() {
  printf("Initializing BNO085 (Game Rotation Vector, mag-free)...\n");
  s_fail_count = 0;
  s_has_valid  = false;

  // I2C bring-up with retries.
  const int MAX_INIT_ATTEMPTS = 10;
  int attempts = 0;
  for (; attempts < MAX_INIT_ATTEMPTS; attempts++) {
    if (s_imu->begin_I2C(BNO08x_I2CADDR_DEFAULT, i2c0, I2C_SDA_PIN, I2C_SCL_PIN)) {
      printf("IMU I2C connection established (SDA=GP%d, SCL=GP%d).\n",
             I2C_SDA_PIN, I2C_SCL_PIN);
      break;
    }
    printf("IMU I2C init failed, retry %d/%d...\n", attempts + 1, MAX_INIT_ATTEMPTS);
    sleep_ms(200);
  }
  if (attempts >= MAX_INIT_ATTEMPTS) {
    printf("ERROR: IMU I2C init failed. Check wiring / 3.3V / 4.7k pull-ups.\n");
    return false;
  }

  // Enable the single heading report, with retries.
  const int MAX_REPORT_ATTEMPTS = 5;
  bool enabled = false;
  for (int i = 0; i < MAX_REPORT_ATTEMPTS; i++) {
    if (enableHeadingReport()) { enabled = true; break; }
    printf("Failed to enable Game Rotation Vector, retry %d/%d...\n", i + 1, MAX_REPORT_ATTEMPTS);
    sleep_ms(100);
  }
  if (!enabled) {
    printf("ERROR: Could not enable Game Rotation Vector.\n");
    return false;
  }

  // Let it start streaming, then validate a few reads.
  sleep_ms(1000);
  int valid = 0;
  const int REQUIRED = 3;
  for (int i = 0; i < 20 && valid < REQUIRED; i++) {
    if (std::isfinite(imuYawDeg()) && s_has_valid) valid++;
    sleep_ms(50);
  }
  if (valid < REQUIRED) {
    printf("ERROR: IMU validation failed (%d/%d valid reads).\n", valid, REQUIRED);
    return false;
  }

  s_initialized = true;
  printf("IMU ready.\n");
  return true;
}

bool imuHealthy() {
  if (!s_initialized) return false;
  uint32_t now = to_ms_since_boot(get_absolute_time());
  if (s_has_valid && (now - s_last_valid_ms) < IMU_TIMEOUT_MS) return true;
  return false;
}
