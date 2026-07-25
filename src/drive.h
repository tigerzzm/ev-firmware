#pragma once
//==============================================================================
// drive — BTS7960 single-motor driver helpers.
// NEW for the EV (robotour used a dual L298N; that driver's PWM-setup pattern
// is the structural reference here — see l298n.cpp in the robotour repo).
//
// BTS7960 wiring (config.h): RPWM=forward PWM, LPWM=reverse/brake PWM,
// R_EN+L_EN tied to one enable GPIO.
//==============================================================================

void driveInit();
void driveForward(float pwmPercent);   // 0..100 forward
void driveBrake();                      // active brake (drive against motion briefly)
void driveCoast();                      // disable outputs, let it roll
