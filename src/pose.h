#pragma once
//==============================================================================
// pose — dead-reckoned (x, y, theta) estimate.
//   theta  = BNO085 yaw (zeroed at ARM)         [radians]
//   ds     = free-roller ground step            [metres]
//   x += ds*cos(theta);  y += ds*sin(theta)
//
// Mirrors robotour-pico's odom pattern (mutex-guarded shared state, heading
// offset captured at zero time via resetValues.theta), reduced to a bicycle/
// single-track model — no differential-drive wheel mixing.
//==============================================================================
#include "position.h"

void     poseInit();          // init mutex
void     poseZeroAtArm();     // x=y=0, distance=0, capture IMU heading offset
void     poseUpdate();        // integrate one tick (call on the control core)
Position poseGet();           // (x, y, theta[rad]) — mutex-guarded snapshot
float    poseHeadingRad();    // heading only, zeroed at ARM

// Estimator gate. The estimator (core1) reads the IMU over I2C0, which is SHARED
// with the LCD (written from core0). To avoid concurrent cross-core bus access,
// the estimator only runs between poseSetActive(true) and poseSetActive(false);
// the LCD is only written while it is inactive (car stopped). poseUpdate() is a
// no-op while inactive.
void     poseSetActive(bool on);
