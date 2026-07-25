#pragma once
//==============================================================================
// speed — longitudinal control (hit the target TIME), DECOUPLED from the stop.
//
// Design intent (EV_Design_Review + EV_Detailed_Design §6.3): pick a cruise
// speed from target time, follow a trapezoid, and OPTIONALLY close a speed loop
// on the free-roller's ground speed with the harvested MotorController.
//
// Deliberately does NOT copy robotour's continuous "RPM = revs_left/time_left"
// scheme — robotour couples distance and time; the EV wants them separate.
//
// The PWM<->speed table is empty until you calibrate it (config/bench).
//==============================================================================

void  speedInit();

// Desired ground speed (m/s) right now, on a trapezoid covering arc length L in
// time T given current along-path distance s.
float speedTrapezoid(float s, float L, float T);

// Feedforward-only mapping speed(m/s) -> PWM% from the calibrated table.
// TODO(bench): fill pwm_for_speed() from measured PWM 30/40/.../100% -> cm/s.
float pwmForSpeed(float mps);

// Optional closed loop: correct PWM using measured ground speed (free roller).
// Uses the harvested MotorController internally when enabled.
float speedControl(float s, float L, float T, float measuredMps);
