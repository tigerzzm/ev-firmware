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
//   * I2C pins moved to the EV bus: i2c0 on GP4/GP5 (was GP20/GP21); RST = GP7.
// KEPT: the robust init-with-retries, reset-recovery, and health-check logic —
//       that scaffolding is the whole reason to reuse this file.
//
// heading is returned in DEGREES, raw (relative to power-on). Zero it at ARM
// time in the pose/estimator layer (see pose.h), exactly as robotour did with
// its resetValues.theta offset.
//==============================================================================

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

// Bring up the IMU (retries + validation). Returns false if it never streams.
bool imuInit();

// Latest fused yaw in DEGREES (Game Rotation Vector). Holds last-good on a miss.
float imuYawDeg();

// True if the IMU has produced recent valid data.
bool imuHealthy();
