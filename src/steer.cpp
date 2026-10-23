#include "steer.h"
#include "config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <atomic>
#include <cmath>

// ---- Servo I/O ----------------------------------------------------------------
// 50 Hz (20 ms) standard servo frame, with the PWM counter running at exactly
// 1 MHz so one tick == one microsecond and steerWriteUs() can write pulse widths
// directly.
//
// The divider is DERIVED from the live system clock, not hardcoded. RP2040 boots
// at 125 MHz and RP2350 at 150 MHz; a fixed divider would silently mis-scale every
// servo command by ~20% on the other chip. This way the same source is correct on
// both.
static uint s_slice, s_chan;

// Written on core0 (the state machine), read on core1 (poseUpdate), so it is
// atomic. (The M0+ has no atomic instructions; the SDK's pico_atomic supplies
// them, as it does for the other cross-core atomics in odo/pose/imu.)
static std::atomic<float> s_deltaRad{0.0f};

void steerInit() {
  gpio_set_function(SERVO_SIG_PIN, GPIO_FUNC_PWM);
  s_slice = pwm_gpio_to_slice_num(SERVO_SIG_PIN);
  s_chan  = pwm_gpio_to_channel(SERVO_SIG_PIN);
  pwm_config cfg = pwm_get_default_config();
  const float div = (float)clock_get_hz(clk_sys) / 1000000.0f;  // -> 1 MHz counter
  pwm_config_set_clkdiv(&cfg, div);     // 125.0 on RP2040, 150.0 on RP2350
  pwm_config_set_wrap(&cfg, 20000);     // 20000 us = 20 ms = 50 Hz
  pwm_init(s_slice, &cfg, true);
  steerCenter();
}

void steerWriteUs(int us) {
  // Travel limit first (config.h §2), then the servo's electrical range. The
  // linkage runs out of travel well before the servo does, and holding against a
  // mechanical stop means stall current until something is commanded otherwise.
  if (SERVO_TRAVEL_US > 0) {
    us = std::clamp(us, SERVO_CENTER_US - SERVO_TRAVEL_US,
                        SERVO_CENTER_US + SERVO_TRAVEL_US);
  }
  us = std::clamp(us, SERVO_MIN_US, SERVO_MAX_US);
  pwm_set_chan_level(s_slice, s_chan, (uint16_t)us);   // 1 tick == 1 us
  // Record the angle this pulse ACTUALLY gives, after both clamps — that is what
  // the estimator integrates along. Recording the requested angle overstated it
  // whenever SERVO_TRAVEL_US is tighter than DELTA_MAX_RAD (0.12 rad needs
  // ~428 us at ~3565 us/rad, but travel stops at 400 us).
  // Until SERVO_US_PER_RAD is measured (it ships at 0) the angle is unknown; the
  // car won't arm then, and the raw servo jog must not divide by zero.
  s_deltaRad.store(SERVO_US_PER_RAD > 0.0f
                       ? (float)(us - SERVO_CENTER_US) / (STEER_DIR_SIGN * SERVO_US_PER_RAD)
                       : 0.0f);
}

void steerSetAngle(float deltaRad) {
  deltaRad = std::clamp(deltaRad, -DELTA_LIMIT_RAD, DELTA_LIMIT_RAD);
  // STEER_DIR_SIGN (config.h §3) makes "+ = wheels LEFT" true on this linkage, so
  // the control laws never carry a sign of their own.
  const int us = SERVO_CENTER_US + (int)std::lround(STEER_DIR_SIGN * deltaRad * SERVO_US_PER_RAD);
  steerWriteUs(us);
}

void steerCenter() { steerSetAngle(0.0f); }

// A pulse width of 0 means no pulse at all: an analog servo stops driving its
// motor (no holding current) and the gear train holds the horn by friction. The
// next steerWriteUs()/steerSetAngle() resumes normal pulses.
void steerRelease() { pwm_set_chan_level(s_slice, s_chan, 0); }

float steerAngleRad() { return s_deltaRad.load(); }
