#pragma once
#include "hardware/pio.h"
static const pio_program_t quadrature_encoder_program {};
static inline void quadrature_encoder_program_init(PIO, uint, uint, int){}
static inline int  quadrature_encoder_get_count(PIO, uint){ return 0; }
