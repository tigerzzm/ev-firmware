#pragma once
//==============================================================================
// cal — bench calibration routines.
//
// Entered by HOLDING THE SET BUTTON at power-on (see main.cpp). Runs instead of
// the normal state machine, and never returns — power-cycle to get back.
//
// Deliberately does NOT need the IMU: this runs before imuInit(), so you can
// calibrate the drivetrain before the heading sensor is even wired.
//==============================================================================

// Top-level bench menu. Never returns — power-cycle to leave.
void calMenu();

// Motor + driver check. Short bursts at a dialled PWM, forward or reverse, with
// the encoder delta reported — so this verifies motor direction, driver wiring,
// encoder direction, and the lowest duty that actually moves the car, all at once.
void calMotorTest();

// Servo jog. The dial moves the servo LIVE so you can centre it, find the real
// mechanical limits, and measure beam angle vs pulse width for SERVO_US_PER_RAD.
void calServoTest();

// Live IMU heading readout. The only way to verify HEADING_SIGN (config.h §3) —
// turn the car LEFT by hand and the relative heading must INCREASE. Also shows
// imuHealthy(), so a chip that answered at boot but has since stopped streaming
// is visible rather than silent.
// Works from either entry route: main() runs imuInit() before the calibration
// menu, and defers the fatal halt until after it.
void calImuTest();

// Live steering-response test. Plans a nominal arc, activates the estimator, and
// runs steerControl() continuously so you can rotate/push the car by hand and
// watch the servo react. This is the only way to verify STEER_FB_SIGN, and it
// exposes e_head and cross-track so the arc geometry can be sanity-checked too.
void calSteerTest();

// Live input test: shows the raw logic level of every button and both dial
// phases. The firmware cannot tell "button not wired" from "button not pressed",
// so this is the only way to see whether a press actually reaches the Pico.
void calInputTest();

// Live encoder readout. The ONLY way to calibrate M_PER_COUNT: zero it, push the
// car a measured distance by hand, read the count. Prints the resulting
// M_PER_COUNT for you. Needs no calibration itself, so it is the first thing to
// run on new hardware.
void calEncoderReadout();

// PWM -> ground-speed probe. One data point per run: dial a duty %, press START,
// the car drives ~3.5 m and reports the average speed measured over the middle
// CAL_MEASURE_M. Repeat across the range to fill in pwmForSpeed() (speed.cpp).
void calPwmProbe();
