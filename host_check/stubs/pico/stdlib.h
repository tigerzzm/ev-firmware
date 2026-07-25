#pragma once
#include <cstdint>
#include <cstdio>
#include <cstddef>
typedef unsigned int uint;
struct absolute_time_t { uint64_t v; };
static inline void sleep_ms(uint32_t){}
static inline void sleep_us(uint64_t){}
static inline absolute_time_t get_absolute_time(){ return {}; }
static inline uint32_t to_ms_since_boot(absolute_time_t){ return 0; }
static inline void stdio_init_all(){}
