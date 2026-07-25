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
long  odoCountRaw();             // raw quadrature count
float odoDistanceM();            // ground distance since odoZero(), metres
float odoDistanceCm();           // convenience
