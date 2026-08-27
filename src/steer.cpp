#include "steer.h"
#include "config.h"
#include "utils.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <cmath>

// ---- Servo I/O (real) -------------------------------------------------------
// 50 Hz (20 ms) standard servo frame, with the PWM counter running at exactly
// 1 MHz so one tick == one microsecond and steerWriteUs() can write pulse widths
// directly.
//
// The divider is DERIVED from the live system clock, not hardcoded. RP2040 boots
// at 125 MHz and RP2350 at 150 MHz; a fixed divider would silently mis-scale every
// servo command by ~20% on the other chip and stretch the frame to 24 ms. This
// way the same source is correct on both.
static uint s_slice, s_chan;

void steerInit() {
  gpio_set_function(SERVO_SIG_PIN, GPIO_FUNC_PWM);
  s_slice = pwm_gpio_to_slice_num(SERVO_SIG_PIN);
  s_chan  = pwm_gpio_to_channel(SERVO_SIG_PIN);
  pwm_config cfg = pwm_get_default_config();
  const float div = (float)clock_get_hz(clk_sys) / 1000000.0f;  // -> 1 MHz counter
  pwm_config_set_clkdiv(&cfg, div);     // 125.0 on RP2040, 150.0 on RP2350
  pwm_config_set_wrap(&cfg, 20000);     // 20000 us = 20 ms = 50 Hz
  pwm_init(s_slice, &cfg, true);
  steerWriteUs(SERVO_CENTER_US);
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
}

void steerSetAngle(float deltaRad) {
  int us = SERVO_CENTER_US + (int)(deltaRad * SERVO_US_PER_RAD);
  steerWriteUs(us);
}

// ---- Lateral control law ----------------------------------------------------
// Gains and signs live in config.h §3/§4: STEER_KH, STEER_KC, STEER_SPEED_EPS,
// STEER_FB_SIGN. Confirm STEER_FB_SIGN by pushing the car off the arc by hand
// and checking the servo steers it BACK.

float steerControl(const ArcPlan &plan, const Position &pose, float speedEst) {
  // Feedforward: nominal steering to hold the planned curvature (bicycle model).
  float delta_ff = std::atan(WHEELBASE_L_M * plan.k);

  // Cross-track error to the reference circle.
  float e_ct = pathCrosstrack(plan, pose);

  // Heading error vs the arc tangent at the nearest point.
  //
  // The arc centre sits BELOW the path (Cy < 0), so the car traverses it
  // CLOCKWISE: the radius vector swings from up-left at the start, through
  // straight up at the apex, to up-right at the end. For a radius vector
  // (dx, dy), the clockwise tangent is (dy, -dx) -> atan2(-dx, dy).
  //
  // This was atan2(dx, -dy) — the COUNTER-clockwise tangent, which is exactly
  // 180 deg out. Verified against three points: at the apex it returned 180 deg
  // where the car is plainly heading +x (0 deg), and at the start it gave
  // -153.96 deg instead of +26.05 deg (= +phi/2 above the chord, as it must be).
  float tangent;
  if (plan.k == 0.0f) {
    tangent = 0.0f;                            // straight line along +x
  } else {
    float dxc = (float)pose.x - plan.Cx;
    float dyc = (float)pose.y - plan.Cy;
    tangent = std::atan2(-dxc, dyc);
  }
  float e_head = (float)utils::angleError(tangent, pose.theta, true);

  // Stanley-style feedback.
  float delta_fb = STEER_FB_SIGN * (STEER_KH * e_head +
                   std::atan2(STEER_KC * e_ct, speedEst + STEER_SPEED_EPS));

  float delta = std::clamp(delta_ff + delta_fb, -DELTA_MAX_RAD, DELTA_MAX_RAD);
  steerSetAngle(delta);
  return delta;
}
