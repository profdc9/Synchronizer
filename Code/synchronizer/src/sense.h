/* sense.h - tank oscillator drive and pendulum arrival detection */

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

#ifndef _SENSE_H
#define _SENSE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The sense coil and C3 || C4 form a parallel resonant tank.  GPIO_OSCIL
   drives it at resonance through Q1/Q2 and R5.  When the bob swings past,
   it changes the coil's inductance, the tank detunes, its impedance falls
   and the voltage across it drops.  The LM358 amplifies that and the
   envelope detector turns it into a slow bump on ADC0.

   The bob reaches this end of its swing once per FULL period, so one
   detected event is one full swing - 0.857 s, 4200 per hour.

   Arrival time is taken as the midpoint between the two threshold
   crossings rather than the peak: the bob is nearly stationary at the
   extreme, so the bump is broad and its peak is poorly localised, but the
   two flanks are steep and symmetric about the turning point. */

#define SENSE_RING        32
#define SENSE_SAMPLE_HZ   1000u

typedef struct _sense_event
{
  uint32_t seq;
  uint64_t t_us;        /* local timer at the interpolated midpoint     */
  uint64_t utc_ns;      /* disciplined UTC, 0 if the timebase is cold   */
  uint16_t peak;        /* largest excursion from baseline, ADC counts  */
  uint16_t baseline;    /* baseline the event was measured against      */
  uint32_t width_us;    /* time between the two threshold crossings     */
} sense_event;

void sense_init(void);
void sense_set_tank_hz(uint32_t hz);
uint32_t sense_tank_hz(void);
void sense_enable(bool on);
bool sense_enabled(void);

/* Pull the next event, oldest first.  False when the ring is empty. */
bool sense_next_event(sense_event *out);

/* Live state, for the "status" command. */
uint16_t sense_baseline(void);
uint16_t sense_last_sample(void);
uint32_t sense_event_count(void);
uint32_t sense_overrun_count(void);
uint64_t sense_last_event_us(void);

/* Mean of the last n full-swing intervals, microseconds; 0 if not enough
   events have been seen yet. */
uint64_t sense_mean_interval_us(uint32_t n);

/* --- bring-up tools ---------------------------------------------------- */

/* Step the drive frequency across a range, averaging ADC0 at each step, and
   print a table.  This is how the tank's resonance is found. */
void sense_sweep(uint32_t from_hz, uint32_t to_hz, uint32_t step_hz,
                 uint32_t dwell_ms);

/* Capture the raw amplified tank waveform on ADC1 and print it. */
void sense_capture(uint32_t rate_hz, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* _SENSE_H */
