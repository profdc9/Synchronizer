#ifndef _STUB_PICO_STDLIB_H
#define _STUB_PICO_STDLIB_H
#include <stdint.h>
#include <stdbool.h>
extern uint64_t stub_now_us;
static inline uint64_t time_us_64(void) { return stub_now_us; }
#endif
