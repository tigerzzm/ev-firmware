#include "steer.h"
#include "config.h"
#include "utils.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <cmath>

// ---- Servo I/O (real) -------------------------------------------------------
// 50 Hz (20 ms) standard servo frame. Pico PWM: pick clkdiv so wrap maps to 20 ms.
// sys clk 150 MHz / clkdiv 150 = 1 MHz -> 1 tick = 1 us -> wrap 20000 = 20 ms.
static uint s_slice, s_chan;

void steerInit() {
  gpio_set_function(SERVO_SIG_PIN, GPIO_FUNC_PWM);
  s_slice = pwm_gpio_to_slice_num(SERVO_SIG_PIN);
  s_chan  = pwm_gpio_to_channel(SERVO_SIG_PIN);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, 150.f);   // 150 MHz -> 1 MHz (1 us/tick). Adjust if sysclk differs.
  pwm_config_set_wrap(&cfg, 20000);     // 20 ms period
  pwm_init(s_slice, &cfg, true);
  steerWriteUs(SERVO_CENTER_US);
}

void steerWriteUs(int us) {
  us = std::clamp(us, SERVO_MIN_US, SERVO_MAX_US);
  pwm_set_chan_level(s_slice, s_chan, (uint16_t)us);   // 1 tick == 1 us
}

void steerSetAngle(float deltaRad) {
  int us = SERVO_CENTER_US + (int)(deltaRad * SERVO_US_PER_RAD);
  steerWriteUs(us);
}

// ---- Lateral control law (bench-tune gains + SIGNS) -------------------------
// TODO(bench): confirm the signs of e_ct, e_head, and SERVO_US_PER_RAD by
// pushing the car off the arc by hand and checking the servo steers it BACK.
static constexpr float Kh  = 1.0f;   // heading-error gain   (TODO tune)
static constexpr float Kc  = 1.0f;   // cross-track gain      (TODO tune)
static constexpr float EPS = 0.2f;   // m/s, avoids div-by-zero at a standstill

float steerControl(const ArcPlan &plan, const Position &pose, float speedEst) {
  // Feedforward: nominal steering to hold the planned curvature (bicycle model).
  float delta_ff = std::atan(WHEELBASE_L_M * plan.k);

  // Cross-track error to the reference circle.
  float e_ct = pathCrosstrack(plan, pose);

  // Heading error vs the arc tangent at the nearest point.
  float tangent;
  if (plan.k == 0.0f) {
    tangent = 0.0f;                            // straight line along +x
  } else {
    float dxc = (float)pose.x - plan.Cx;
    float dyc = (float)pose.y - plan.Cy;
    tangent = std::atan2(dxc, -dyc);           // sign finalized on bench
  }
  float e_head = (float)utils::angleError(tangent, pose.theta, true);

  // Stanley-style feedback.
  float delta_fb = Kh * e_head + std::atan2(Kc * e_ct, speedEst + EPS);

  float delta = std::clamp(delta_ff + delta_fb, -DELTA_MAX_RAD, DELTA_MAX_RAD);
  steerSetAngle(delta);
  return delta;
}
