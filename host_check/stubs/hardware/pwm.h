#pragma once
#include "pico/stdlib.h"
struct pwm_config {};
static inline pwm_config pwm_get_default_config(){ return {}; }
static inline void pwm_config_set_clkdiv(pwm_config*, float){}
static inline void pwm_config_set_wrap(pwm_config*, uint16_t){}
static inline void pwm_init(uint, pwm_config*, bool){}
static inline uint pwm_gpio_to_slice_num(uint){ return 0; }
static inline uint pwm_gpio_to_channel(uint){ return 0; }
static inline void pwm_set_chan_level(uint, uint, uint16_t){}
static inline void pwm_set_enabled(uint, bool){}
