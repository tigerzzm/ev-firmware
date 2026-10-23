#pragma once
//==============================================================================
// cal — bench calibration routines.
//
// Two ways in: hold SET at power-on (main.cpp), or hold SET for
// CAL_ENTRY_HOLD_MS at the "EV ready" screen (sm.cpp). "8 Exit to run" returns
// to the run sequence.
//
// Only the IMU-heading and steer-response tests need the IMU. A failed imuInit()
// does not lock you out: everything else works with the heading sensor unwired.
//==============================================================================

// Top-level bench menu. Returns when "8 Exit to run" is chosen.
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
// menu. (A failed init no longer halts anything: runs go encoder-only.)
void calImuTest();

// Live steering-response test. Activates the estimator and runs the real forward
// or reverse steering law (DialBtn toggles) so you can rotate/push the car by hand
// and watch the servo react. This is how STEER_DIR_SIGN is verified.
void calSteerTest();

// Live input test: shows the raw logic level of every button and both dial
// phases. The firmware cannot tell "button not wired" from "button not pressed",
// so this is the only way to see whether a press actually reaches the Pico.
void calInputTest();

// Live encoder readout, forward and reverse counts separately. The ONLY way to
// calibrate M_PER_COUNT and M_PER_COUNT_REV: zero it, push the car 5.000 m forward
// by hand, then pull it 5.000 m back; it prints both constants for you. Needs no
// calibration itself, so it is the first thing to run on new hardware.
void calEncoderReadout();

// PWM -> ground-speed probe, forward or reverse. One data point per run: dial a
// duty %, press START, the car drives ~3.5 m and reports the average speed over
// the middle CAL_MEASURE_M. Fit a line through the points for the feedforward
// constants SPEED_FF_OFFSET_PCT / SPEED_FF_PCT_PER_MPS (config.h §2).
void calPwmProbe();
