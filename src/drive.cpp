#include "drive.h"
#include "config.h"
#include "hardware/gpio.h"
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

void driveBrake() {
  // Active brake: drive the reverse half against the car's motion. Callers pulse
  // this for BRAKE_PULSE_MS then driveCoast(). (Bench-tune the level for a dead
  // stop — too hard and gearbox inertia can roll the car back off the target.)
  driveReverse(BRAKE_PWM);
}

void driveCoast() {
  pwm_set_chan_level(s_rpwmSlice, s_rpwmChan, 0);
  pwm_set_chan_level(s_lpwmSlice, s_lpwmChan, 0);
  enableBridges(false);
}
