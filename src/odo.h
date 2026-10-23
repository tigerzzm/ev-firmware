#pragma once
//==============================================================================
// odo — distance from the free-rolling FRONT encoder, decoded in PIO.
// Counts in BOTH directions. The run's MP position is the NET distance since
// odoZero(): forward travel adds, reverse travel subtracts.
//
// Forward and reverse counts are accumulated separately (odo_split.h) so each
// direction gets its own scale (M_PER_COUNT / M_PER_COUNT_REV, config.h §1). Both
// set the final MP; the forward one matters more, because it multiplies the long
// forward leg (see config.h §1).
//==============================================================================
#include <cstdint>
#include "odo_split.h"

void  odoInit();                 // set up PIO SM for the front encoder
void  odoZero();                 // zero distance AND the speed estimate (core0 only)
long  odoCountRaw();             // raw quadrature count, sign as wired
long  odoCountSigned();          // ENCODER_SIGN applied: + = forward (net, since boot)
float odoDistanceM();            // net MP travel since odoZero(), metres (signed)

// One consistent sample of the counts since odoZero(): .fwd (>= 0) and .rev (<= 0)
// attributed to each direction, .net() exact. For calibrating the two scales:
// push 5.000 m forward, then 5.000 m back, read each. (net is not always
// fwd + rev: up to ODO_DIR_HYST_COUNTS - 1 counts can be pending, odo_split.h.)
DirSplit odoSnapshot();

// ---- Ground-speed estimate ---------------------------------------------------
// Differentiated from distance and low-passed; signed (+ forward).
// SINGLE-CORE: call all three from core0 only (the control loop / bench tools).
// It used to run inside the estimator on core1, after a blocking IMU read — so
// any IMU stall froze the speed feedback the drive loop depends on.
// odoSpeedTick() is cheap and safe to call every tick: it re-samples only once
// per SPEED_SAMPLE_MIN_S, and takes the distance the caller already read so the
// encoder isn't sampled twice per tick.
void  odoSpeedReset();           // clear filter state (call when motion starts)
void  odoSpeedTick(float distM); // update from odoDistanceM() the caller just read
float odoSpeedMps();             // filtered ground speed, m/s, signed
