//==============================================================================
// EV firmware entry point (Raspberry Pi Pico 2 / RP2350).
//
// Two-core split (mirrors robotour-pico's structure):
//   core1 = pose estimator, run at the control tick (poseUpdate).
//   core0 = the state machine (sm.cpp), which issues drive/steer commands.
//==============================================================================
#include "cal.h"
#include "config.h"
#include "drive.h"
#include "imu.h"
#include "odo.h"
#include "pose.h"
#include "sm.h"
#include "speed.h"
#include "steer.h"
#include "ui.h"

#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <cstdio>

// ---- core1: the estimator loop ----------------------------------------------
// Also carries the heartbeat: a blinking LED means core1 is alive and ticking.
// Dark or solid means the estimator has stopped, which is exactly the failure you
// cannot otherwise see without a serial monitor attached.
static void estimatorCore1() {
#ifdef PICO_DEFAULT_LED_PIN
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  uint32_t lastBlinkMs = 0;
  bool led = false;
#endif
  while (true) {
    poseUpdate();
#ifdef PICO_DEFAULT_LED_PIN
    if (HEARTBEAT_MS) {
      const uint32_t nowMs = to_ms_since_boot(get_absolute_time());
      if (nowMs - lastBlinkMs >= HEARTBEAT_MS) {
        led = !led;
        gpio_put(PICO_DEFAULT_LED_PIN, led);
        lastBlinkMs = nowMs;
      }
    }
#endif
    sleep_us(CONTROL_TICK_US);
  }
}

int main() {
  // Actuators FIRST, before anything that blocks. Until these run, the BTS7960
  // enable pins and the servo signal pin are undriven inputs (weak internal
  // pull-down only) and both peripherals are in an undefined state. Doing this
  // after the USB delay below left that window open for 1.5 s on every power-on —
  // long enough for coupled noise on the servo line to read as stray pulses.
  driveInit();
  driveCoast();
  steerInit();      // valid 50 Hz frame at SERVO_CENTER_US from here on

  stdio_init_all();
  sleep_ms(1500);   // let USB CDC enumerate before the first prints
  printf("=== EV firmware boot (%s) ===\n", PICO_BOARD);

  speedInit();
  uiInit();
  odoInit();
  poseInit();

  // Launch core1 EARLY. poseUpdate() returns immediately while the estimator is
  // inactive, so until ARM this thread does nothing but blink the heartbeat —
  // and that is precisely when you need it. Launching after imuInit() meant no
  // heartbeat during calibration mode, and none during the IMU-failure halt,
  // i.e. a dark LED in both situations where "is the board alive?" is the
  // question being asked.
  multicore_launch_core1(estimatorCore1);

  // Sample the button BEFORE imuInit(), which takes a few seconds with its retries
  // — nobody should have to hold SET through all of that.
  const bool wantCal = uiSetPressed();
  if (wantCal) printf("[MAIN] SET held at boot -> calibration mode after IMU init\n");

  // Always attempt the IMU, even when heading is not needed. Doing it before the
  // calibration menu is what makes the IMU-heading screen usable from BOTH entry
  // routes (SET at power-on, and the long-press from "EV ready"). Failure is not
  // fatal here — only a real run needs heading, so the halt is deferred.
  const bool imuOk = imuInit();
  if (!imuOk) {
    printf("WARNING: IMU init failed. Bench calibration still available;\n");
    printf("         a RUN is not — check I2C wiring / pull-ups.\n");
  }

  if (wantCal) {
    calMenu();                      // returns via "Exit to run"
  }

  if (!imuOk) {
    printf("FATAL: cannot run without heading — halting.\n");
    while (true) { uiShow("IMU FAIL", "hold SET=cal"); sleep_ms(1000); }
  }

  // State machine owns the run sequence forever.
  smRun();
  return 0;
}
