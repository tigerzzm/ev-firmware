#pragma once
#include "pico/stdlib.h"
struct pio_hw; typedef struct pio_hw* PIO;
#define pio0 ((PIO)0)
#define pio1 ((PIO)1)
struct pio_program_t {};
static inline uint pio_add_program(PIO, const pio_program_t*){ return 0; }
static inline bool pio_sm_is_rx_fifo_empty(PIO, uint){ return true; }
static inline uint32_t pio_sm_get(PIO, uint){ return 0; }
static inline uint pio_sm_get_rx_fifo_level(PIO, uint){ return 0; }
