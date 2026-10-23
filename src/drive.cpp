#include "drive.h"
#include "config.h"
#include "odo.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <cstdint>
#include <initializer_list>

// PWM setup pattern mirrors robotour's l298n.cpp (16-bit wrap, clkdiv 4).
static uint s_rpwmSlice, s_rpwmChan;
static uint s_lpwmSlice, s_lpwmChan;

static void setupPwmPin(uint pin, uint *slice, uint *chan) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  *slice = pwm_gpio_to_slice_num(pin);
  *chan  = pwm_gpio_to_channel(pin);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, 4.f);
  pwm_config_set_wrap(&cfg, 65535);
  pwm_init(*slice, &cfg, true);
  pwm_set_chan_level(*slice, *chan, 0);
}

// Both half-bridges are inhibited/released together: a single motor has no useful
// mode with one side disabled (that side goes high-Z and opens the motor circuit).
// They are separate GPIOs only so the two chips can be wired independently.
static inline void enableBridges(bool on) {
  gpio_put(BTS7960_R_EN_PIN, on);
  gpio_put(BTS7960_L_EN_PIN, on);
}

static inline uint16_t pctToLevel(float pct) {
  pct = std::clamp(pct, 0.0f, 100.0f);
  return (uint16_t)((pct * 65535.0f) / 100.0f);
}

void driveInit() {
  for (uint p : {(uint)BTS7960_R_EN_PIN, (uint)BTS7960_L_EN_PIN}) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_OUT);
    gpio_put(p, 0);                       // start disabled
  }
  setupPwmPin(BTS7960_RPWM_PIN, &s_rpwmSlice, &s_rpwmChan);
  setupPwmPin(BTS7960_LPWM_PIN, &s_lpwmSlice, &s_lpwmChan);
}

void driveForward(float pwmPercent) {
  enableBridges(true);
  pwm_set_chan_level(s_lpwmSlice, s_lpwmChan, 0);
  pwm_set_chan_level(s_rpwmSlice, s_rpwmChan, pctToLevel(pwmPercent));
}

void driveReverse(float pwmPercent) {
  enableBridges(true);
  pwm_set_chan_level(s_rpwmSlice, s_rpwmChan, 0);
  pwm_set_chan_level(s_lpwmSlice, s_lpwmChan, pctToLevel(pwmPercent));
}

void driveCoast() {
  pwm_set_chan_level(s_rpwmSlice, s_rpwmChan, 0);
  pwm_set_chan_level(s_lpwmSlice, s_lpwmChan, 0);
  enableBridges(false);
}

void driveHold() {
  pwm_set_chan_level(s_rpwmSlice, s_rpwmChan, 0);
  pwm_set_chan_level(s_lpwmSlice, s_lpwmChan, 0);
  enableBridges(true);
}

void driveBrakeToStop(float pwmPercent, uint32_t maxMs) {
  // Brake first (both low-side FETs on), THEN look: one poll of the encoder shows
  // which way the car is still going, if at all. Measuring here rather than in
  // each caller means no caller can aim the counter-pulse the wrong way.
  driveHold();
  if (pwmPercent <= 0.0f) return;
  long last = odoCountSigned();
  sleep_ms(BRAKE_POLL_MS);
  long c = odoCountSigned();
  // Moving = more than a few counts in the poll. One or two counts is edge jitter
  // or the last bit of roll the hold is already stopping; a counter-pulse then
  // would shove a car that has stopped.
  const long moved = c - last;
  const int dir = (moved > BRAKE_MOVING_COUNTS) ? +1 : (moved < -BRAKE_MOVING_COUNTS) ? -1 : 0;
  if (dir == 0) return;                      // stopped (or nearly): the hold is enough

  if (dir > 0) driveReverse(pwmPercent); else driveForward(pwmPercent);
  const uint32_t t0 = to_ms_since_boot(get_absolute_time());
  last = c;
  while (to_ms_since_boot(get_absolute_time()) - t0 < maxMs) {
    sleep_ms(BRAKE_POLL_MS);
    c = odoCountSigned();
    // End on the FIRST poll with no progress in `dir` (or with motion the other
    // way). Waiting longer keeps counter-driving a car that has already stopped.
    if ((c - last) * dir <= 0) break;
    last = c;
  }
  driveHold();
}
