#pragma once
//==============================================================================
// pose — dead-reckoned position of the FREE-ROLLER contact point, in track
// coordinates (x forward along the centerline from Start, y left), plus heading.
//   theta = BNO085 yaw, zeroed at START               [radians, (-pi, pi]]
//   ds    = free-roller step (signed: < 0 in reverse) [metres]
//   x += ds*cos(theta + delta);  y += ds*sin(theta + delta)
// The roller sits on the steered axle, so it rolls along theta + delta (the
// steering angle the servo was actually sent, from steer.cpp), not along the body
// heading.
//
// At START the rear MP is over the Start dot, so the roller starts at
// (ENC_FROM_MP_M, 0). The steering laws work out the rear axle / rear MP from it
// (steer_law.cpp).
//==============================================================================
#include "position.h"

void     poseInit();          // init mutex

// Zero everything: encoder distance (always, immediately, on the calling core),
// roller pose to (ENC_FROM_MP_M, 0, 0), and the heading reference to the car's
// heading now. Call from core0.
//
// While the estimator runs, core1 owns the IMU's I2C bus, so it captures the
// heading zero with its freshest sample and this waits for it. Returns false if
// core1 did not do so within POSE_ZERO_TIMEOUT_MS (IMU stuck): the heading is then
// NOT zeroed, and the request is withdrawn so it can never fire later, mid-run.
// The caller must treat the heading as unusable (steer encoder-only).
bool     poseZeroAtArm();

void     poseUpdate();        // integrate one tick (core1)
Position poseGet();           // roller (x, y, theta) — mutex-guarded snapshot

// Estimator gate: poseUpdate() is a no-op until poseSetActive(true), so core1
// only polls the IMU while something needs heading. While active, ONLY core1 may
// call imuYawDeg() — the IMU is not safe to read from two cores at once.
void     poseSetActive(bool on);
