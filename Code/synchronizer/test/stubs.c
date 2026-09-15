/* stubs.c - the least that control.c needs to run on a host.

   A queue of synthetic sense events, a drive coil that only counts pulses,
   a timebase that is always good, and enough of the configuration to work
   out how long a swing is.
*/

/*
   Copyright (c) 2026 Daniel Marks

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "config.h"
#include "sense.h"
#include "drive.h"
#include "timebase.h"

uint64_t stub_now_us = 0;
synchronizer_config cfg;

/* --- the queue of synthetic events ------------------------------------- */
#define QN 4096
static sense_event q[QN];
static int qh, qt;
void stub_push(const sense_event *e) { q[qt] = *e; qt = (qt + 1) % QN; }
bool sense_next_event(sense_event *o)
{
  if (qh == qt) return false;
  *o = q[qh]; qh = (qh + 1) % QN; return true;
}
uint64_t sense_last_event_us(void) { return stub_now_us; }
uint64_t sense_mean_interval_us(uint32_t n) { (void)n; return 0; }

/* --- the drive coil ---------------------------------------------------- */
uint32_t stub_pulses;
uint64_t stub_last_pulse_us;
bool drive_pulse_at(uint64_t when_us, uint32_t w)
{ (void)w; stub_pulses++; stub_last_pulse_us = when_us; return true; }
void drive_all_off(void) {}
uint32_t drive_pulse_count(void) { return stub_pulses; }

bool tb_have_time(void) { return true; }

/* --- just enough configuration ----------------------------------------- */
uint64_t cfg_period_ns(void)
{ return (3600ull * 1000000000ull * (uint64_t)cfg.beats_per_period)
         / (uint64_t)cfg.beats_per_hour; }
uint64_t cfg_event_interval_ns(void)
{ return cfg_period_ns() / (uint64_t)cfg.events_per_period; }
void cfg_event_ratio(uint64_t *num, uint64_t *den)
{
  *num = 3600ull * 1000000000ull * (uint64_t)cfg.beats_per_period;
  *den = (uint64_t)cfg.beats_per_hour * (uint64_t)cfg.events_per_period;
}
bool config_save(void) { return true; }
void config_defaults(void) {}
void config_load(void) {}
bool config_save_network(void) { return true; }
