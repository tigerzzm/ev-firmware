#pragma once
// Host stub — no concurrency to guard against.
struct critical_section_t { int dummy; };
static inline void critical_section_init(critical_section_t*){}
static inline void critical_section_enter_blocking(critical_section_t*){}
static inline void critical_section_exit(critical_section_t*){}
