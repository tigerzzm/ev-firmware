#pragma once
//==============================================================================
// odo — distance from the free-rolling FRONT encoder, decoded in PIO.
// This is the along-path progress AND the closed-loop stop trigger.
// Encoder-read pattern harvested from robotour-pico (src/odom.cpp readSensorData
// + quadrature_encoder.pio); simplified to a single undriven wheel.
//==============================================================================
#include <cstdint>

void  odoInit();                 // set up PIO SM for the front encoder
void  odoZero();                 // zero the distance reference (call at ARM)
long  odoCountRaw();             // raw quadrature count, sign as wired
long  odoCountSigned();          // ENCODER_SIGN applied — what the firmware uses
float odoDistanceM();            // ground distance since odoZero(), metres
float odoDistanceCm();           // convenience

// ---- Ground-speed estimate ---------------------------------------------------
// The free roller is also the speed source (there is no drive-motor encoder), so
// speed is differentiated from distance and low-passed. odoSpeedTick() is cheap
// and safe to call every control tick — it re-samples only once per
// SPEED_SAMPLE_MIN_S so encoder quantisation doesn't dominate the derivative.
// It is called from poseUpdate() on core1; odoSpeedMps() is read from core0.
void  odoSpeedReset();           // clear filter state (call at ARM)
void  odoSpeedTick();            // sample/update the estimate (core1, every tick)
float odoSpeedMps();             // filtered ground speed, m/s (any core)
