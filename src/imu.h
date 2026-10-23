#pragma once
//==============================================================================
// IMU driver — Adafruit BNO085, Game Rotation Vector (accel+gyro, NO magnetometer)
//
// Harvested & adapted from robotour-pico (src/imu.h + src/imu.cpp).
// CHANGES vs robotour:
//   * Report switched from GYRO_INTEGRATED_RV (0x2A) to GAME_ROTATION_VECTOR
//     (0x08) — the on-chip fused, mag-free heading your EV design specifies.
//   * ARVR report no longer enabled (it was only needed to make the gyro report
//     stream on robotour); Game Rotation Vector streams on its own.
//   * I2C on i2c0, pins from config.h §6 (GP20/GP21); RST = GP7.
// KEPT: the robust init-with-retries, reset-recovery, and health-check logic —
//       that scaffolding is the whole reason to reuse this file.
//
// heading is returned in DEGREES, raw (relative to power-on). Zero it at ARM
// time in the pose/estimator layer (see pose.h), exactly as robotour did with
// its resetValues.theta offset.
//==============================================================================

#include <cstdint>

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

// Bring up the IMU (retries + validation). Returns false if it never streams.
bool imuInit();

// Latest fused yaw in DEGREES (Game Rotation Vector). Holds last-good on a miss.
float imuYawDeg();

// True if imuInit() ever fully succeeded. The estimator keeps polling such a chip
// even while it isn't ready, because polling is what retries the re-enable.
bool imuInitOk();

// Drop serial messages (they can block ~500 ms on USB). pose.cpp turns this on
// while the estimator runs, i.e. during a run.
void imuSetQuiet(bool quiet);

// Chip resets seen since boot. A reset restarts the Game Rotation Vector from a
// NEW reference, so any heading zero taken before it is wrong — even after the
// chip is healthy again. Record this at the heading zero and compare later.
uint32_t imuResetCount();

// True from a chip reset until the first valid sample after it. While true, the
// newest yaw is from the OLD frame or a placeholder — never zero a heading on it.
// (Different from !imuHealthy(), which is also true for a merely late sample.)
bool imuRecovering();

// Milliseconds since the newest valid sample (UINT32_MAX if there never was one).
uint32_t imuSampleAgeMs();

// True if the newest valid sample is younger than IMU_STALE_MS (150 ms).
// Safe to call from either core.
//
// NOTE: this is a FRESHNESS check, not a chip-status check. The timestamp behind
// it is only updated inside imuYawDeg(), so it decays to false whenever nothing
// is polling. During a run core1 polls at 500 Hz; anything else (bench tools)
// must prime it with a few imuYawDeg() calls before trusting the answer.
bool imuHealthy();
