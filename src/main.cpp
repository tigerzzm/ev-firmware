//==============================================================================
// EV firmware entry point (Raspberry Pi Pico 2 / RP2350).
//
// Two-core split (mirrors robotour-pico's structure):
//   core1 = pose estimator, run at the control tick (poseUpdate).
//   core0 = the state machine (sm.cpp), which issues drive/steer commands.
//==============================================================================
#include "config.h"
#include "drive.h"
#include "imu.h"
#include "odo.h"
#include "pose.h"
#include "sm.h"
#include "speed.h"
#include "steer.h"
#include "ui.h"

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <cstdio>

// ---- core1: the estimator loop ----------------------------------------------
static void estimatorCore1() {
  while (true) {
    poseUpdate();
    sleep_us(CONTROL_TICK_US);
  }
}

int main() {
  stdio_init_all();
  sleep_ms(1500);   // let USB CDC enumerate before the first prints
  printf("=== EV firmware boot (Pico 2) ===\n");

  driveInit();
  driveCoast();
  steerInit();
  speedInit();
  uiInit();
  odoInit();
  poseInit();

  if (!imuInit()) {
    printf("FATAL: IMU init failed — halting. Check I2C wiring / pull-ups.\n");
    while (true) { uiShow("IMU FAIL", "check wiring"); sleep_ms(1000); }
  }

  // Estimator on the second core.
  multicore_launch_core1(estimatorCore1);

  // State machine owns the run sequence forever.
  smRun();
  return 0;
}
