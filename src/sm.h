#pragma once
//==============================================================================
// sm — top-level state machine (EV_Detailed_Design §5).
// NEW for the EV. Runs on core0; the pose estimator runs on core1 (see main.cpp).
// This is a working skeleton: the flow and module calls are real, the numbers
// and a few branches are marked TODO for bench tuning.
//==============================================================================

enum class EvState { BOOT, SETUP, ARMED, LAUNCH, CRUISE, APPROACH, BRAKE, DONE };

void smRun();   // never returns; loops SETUP..DONE for each of the 2 runs
