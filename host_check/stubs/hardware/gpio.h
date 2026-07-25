#pragma once
#include "pico/stdlib.h"
enum gpio_function { GPIO_FUNC_XIP, GPIO_FUNC_SPI, GPIO_FUNC_I2C, GPIO_FUNC_PWM, GPIO_FUNC_SIO };
#define GPIO_OUT 1
#define GPIO_IN 0
static inline void gpio_init(uint){}
static inline void gpio_set_dir(uint,int){}
static inline void gpio_put(uint,int){}
static inline int  gpio_get(uint){ return 1; }
static inline void gpio_pull_up(uint){}
static inline void gpio_set_function(uint, gpio_function){}
