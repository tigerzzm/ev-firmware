#pragma once
//==============================================================================
// steer — steering servo output + lateral control law.
// NEW for the EV (robotour steered differentially; there is no servo code to
// harvest). Servo I/O below is real; the control GAINS and SIGNS are bench-tuned
// (see EV_Detailed_Design §6.2 and §7). robotour's utils::getCurvature /
// findLookaheadCurvature can be a reference if you prefer pure-pursuit over the
// Stanley-style law sketched here.
//==============================================================================
#include "position.h"
#include "path.h"

void  steerInit();                       // servo PWM @ 50 Hz, centre the wheels
void  steerWriteUs(int us);              // raw servo command (clamped)
void  steerSetAngle(float deltaRad);     // steering angle -> servo us

// Full lateral control: feedforward arc + cross-track/heading feedback.
// Returns the commanded steering angle (rad) and also writes it to the servo.
float steerControl(const ArcPlan &plan, const Position &pose, float speedEst);
