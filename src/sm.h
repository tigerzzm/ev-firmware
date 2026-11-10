#pragma once
//==============================================================================
// sm — top-level state machine for the 2027 bottle run
// (docs/design/EV_Detailed_Design.md §3). Runs on core0; the pose estimator runs
// on core1 (see main.cpp).
//
//   SETUP      enter D, B, T, level, offsets; plan; refuse if uncalibrated
//   ARMED      servo centred, estimator polling the IMU; START (pencil) zeroes
//              the pose and begins Run Time
//   DELAY      stand still for the planned start delay (legal, §8.o.i)
//   CRUISE     forward on y = 0 toward the bottle
//   CAPTURE    slow through first contact so the bottle seats
//   PUSH       push the bottle, then a gentle decel to stop at xF
//   TURNAROUND dynamic-brake hold for TURN_MS (well under the 3 s stop rule)
//   REVERSE    back up, aiming the rear MP at the Target, decel into xFinal
//   STOP       brake, hold through the 3 s Complete Stop, report
//
// SETUP, ARMED and DELAY are simple waits before the car moves; EvState lists the
// states of the motion loop that follows them.
//==============================================================================

// DELAY: the stationary start delay, recorded in the telemetry only (heading watch).
enum class EvState { CRUISE, CAPTURE, PUSH, TURNAROUND, REVERSE, STOP, DELAY };

// Never returns. imuOk = false runs every leg with the servo centred (encoder
// only) after an on-screen warning, rather than refusing to run at all.
void smRun(bool imuOk);

// "8 Sim dry run" (bench menu): target entry, plan, then a full run with the
// wheels UP on a simulated encoder (odo.h, odoSimSet), and the usual report
// marked SIMULATED. Returns to the bench menu.
void smDryRun();
