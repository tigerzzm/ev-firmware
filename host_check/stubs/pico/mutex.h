#pragma once
struct mutex_t {};
static inline void mutex_init(mutex_t*){}
static inline void mutex_enter_blocking(mutex_t*){}
static inline void mutex_exit(mutex_t*){}
