#pragma once
// Host stub. The real SDK header reports the live system clock; the value here
// only needs to be plausible so the servo divider math type-checks.
#include <cstdint>
enum clock_num_t { clk_sys = 5 };
static inline uint32_t clock_get_hz(clock_num_t) { return 125000000u; }
