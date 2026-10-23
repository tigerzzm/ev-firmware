#pragma once
//==============================================================================
// drive — BTS7960 single-motor driver helpers.
// NEW for the EV (robotour used a dual L298N; that driver's PWM-setup pattern
// is the structural reference here — see l298n.cpp in the robotour repo).
//
// BTS7960 wiring (config.h): RPWM=forward PWM, LPWM=reverse/brake PWM,
// R_EN and L_EN on two separate GPIOs (config.h §6), always driven together.
//==============================================================================
#include <cstdint>

void driveInit();
void driveForward(float pwmPercent);   // 0..100 forward
void driveReverse(float pwmPercent);   // 0..100 reverse
void driveCoast();                     // bridges disabled: motor open, car rolls freely

// Dynamic brake: bridges ENABLED with both PWMs at 0, so both low-side FETs are on
// and the motor terminals are shorted together. Back-EMF then brakes the motor and
// it resists being rolled — a stop that holds, unlike coast.
void driveHold();

// Stop the car: dynamic-brake hold at once, then watch the encoder for one
// BRAKE_POLL_MS poll. If it is still moving, drive the OPPOSITE way at pwmPercent
// until a poll shows no further progress (or maxMs passes), then hold again. The
// direction is measured here, so a car that is already stopped never gets a
// counter-pulse. pwmPercent = 0 gives a pure dynamic-brake stop. Blocks for at
// most maxMs + one poll.
void driveBrakeToStop(float pwmPercent, uint32_t maxMs);
