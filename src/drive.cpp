#include "drive.h"
#include "config.h"
#include "odo.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <initializer_list>

// PWM setup pattern mirrors robotour's l298n.cpp; the frequency is DRIVE_PWM_HZ.
static uint s_rpwmSlice, s_rpwmChan;
static uint s_lpwmSlice, s_lpwmChan;

// The last command, for the dry run's simulated roller (driveCommand). Written on
// core0, read under odo's lock from either core; the two are read separately, and
// a tick-old mismatch between them is harmless to the simulation.
static std::atomic<int>   s_cmdMode{(int)DriveMode::Coast};
static std::atomic<float> s_cmdPct{0.0f};
static void record(DriveMode m, float pct) {
  s_cmdPct.store(pct);
  s_cmdMode.store((int)m);
}

// Counter period for DRIVE_PWM_HZ, set in driveInit: the finest resolution that
// fits the 16-bit counter (20 kHz on a 125 MHz clock: divider 1, 6250 steps).
static float    s_div  = 1.0f;
static uint16_t s_wrap = 65535;

static void setupPwmPin(uint pin, uint *slice, uint *chan) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  *slice = pwm_gpio_to_slice_num(pin);
  *chan  = pwm_gpio_to_channel(pin);
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, s_div);
  pwm_config_set_wrap(&cfg, s_wrap);
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
  return (uint16_t)((pct * s_wrap) / 100.0f);
}

void driveInit() {
  for (uint p : {(uint)BTS7960_R_EN_PIN, (uint)BTS7960_L_EN_PIN}) {
    gpio_init(p);
    gpio_set_dir(p, GPIO_OUT);
    gpio_put(p, 0);                       // start disabled
  }
  const float ticks = (float)clock_get_hz(clk_sys) / DRIVE_PWM_HZ;   // at divider 1
  s_div  = std::max(1.0f, ticks / 65536.0f);
  s_wrap = (uint16_t)(ticks / s_div - 1.0f);
  setupPwmPin(BTS7960_RPWM_PIN, &s_rpwmSlice, &s_rpwmChan);
  setupPwmPin(BTS7960_LPWM_PIN, &s_lpwmSlice, &s_lpwmChan);
}

void driveForward(float pwmPercent) {
  // PWM 0 with the bridges on is the dynamic brake, not drive.
  record(pwmPercent > 0.0f ? DriveMode::Drive : DriveMode::Hold, std::max(pwmPercent, 0.0f));
  enableBridges(true);
  pwm_set_chan_level(s_lpwmSlice, s_lpwmChan, 0);
  pwm_set_chan_level(s_rpwmSlice, s_rpwmChan, pctToLevel(pwmPercent));
}

void driveReverse(float pwmPercent) {
  record(pwmPercent > 0.0f ? DriveMode::Drive : DriveMode::Hold, -std::max(pwmPercent, 0.0f));
  enableBridges(true);
  pwm_set_chan_level(s_rpwmSlice, s_rpwmChan, 0);
  pwm_set_chan_level(s_lpwmSlice, s_lpwmChan, pctToLevel(pwmPercent));
}

void driveCoast() {
  record(DriveMode::Coast, 0.0f);
  pwm_set_chan_level(s_rpwmSlice, s_rpwmChan, 0);
  pwm_set_chan_level(s_lpwmSlice, s_lpwmChan, 0);
  enableBridges(false);
}

void driveHold() {
  record(DriveMode::Hold, 0.0f);
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

void driveCommand(DriveMode &mode, float &pct) {
  mode = (DriveMode)s_cmdMode.load();
  pct  = s_cmdPct.load();
}
