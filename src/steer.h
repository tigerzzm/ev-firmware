#pragma once
//==============================================================================
// steer — steering servo output. The control laws live in steer_law.* (pure math,
// tested); this file maps a steering angle onto the servo.
//==============================================================================

void  steerInit();                       // servo PWM @ 50 Hz, centre the wheels
void  steerWriteUs(int us);              // raw servo command (clamped)
void  steerSetAngle(float deltaRad);     // beam angle (+ = wheels LEFT) -> servo
void  steerCenter();                     // steerSetAngle(0)
void  steerRelease();                    // stop the pulses: servo goes limp, holds by friction
