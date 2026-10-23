#pragma once
#include "pico/stdlib.h"
struct i2c_inst; typedef struct i2c_inst i2c_inst_t;
#define i2c0 ((i2c_inst_t*)0)
#define i2c1 ((i2c_inst_t*)1)
static inline int i2c_init(i2c_inst_t*, uint){ return 0; }
static inline int i2c_write_blocking(i2c_inst_t*, uint8_t, const uint8_t*, size_t, bool){ return 0; }
static inline int i2c_write_timeout_us(i2c_inst_t*, uint8_t, const uint8_t*, size_t, bool, uint){ return 0; }
static inline int i2c_read_blocking(i2c_inst_t*, uint8_t, uint8_t*, size_t, bool){ return 1; }
static inline int i2c_read_timeout_us(i2c_inst_t*, uint8_t, uint8_t*, size_t, bool, uint){ return 1; }
