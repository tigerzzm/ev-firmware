#pragma once
//==============================================================================
// steer — steering servo output. The control laws live in steer_law.* (pure math,
// tested); this file maps a steering angle onto the servo and remembers it.
//==============================================================================

void  steerInit();                       // servo PWM @ 50 Hz, centre the wheels
void  steerWriteUs(int us);              // raw servo command (clamped)
void  steerSetAngle(float deltaRad);     // beam angle (+ = wheels LEFT) -> servo
void  steerCenter();                     // steerSetAngle(0)
void  steerRelease();                    // stop the pulses: servo goes limp, holds by friction

// Beam angle the servo was last actually sent (after the travel and electrical
// clamps), radians. Read by the estimator on core1: the free roller sits on the
// steered axle, so it rolls along theta + delta, not theta.
float steerAngleRad();
