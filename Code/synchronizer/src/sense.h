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
#define SENSE_SAMPLE_HZ   1000u      /* default only; cfg.sample_hz wins */

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

/* Recompute the detector's timing windows from the configuration.  They
   are percentages of the expected interval between sense events, so they
   follow the pendulum rather than assuming a fast one.  Call this after
   anything that changes the clock's geometry or rate. */
void sense_refresh_timing(void);
uint32_t sense_rearm_us(void);
uint32_t sense_min_event_us(void);
uint32_t sense_max_event_us(void);
void sense_set_tank_hz(uint32_t hz);
/* Drive level as the WIDTH of the pulse on GPIO_OSCIL, in nanoseconds.
   0 means no drive at all, which is a diagnostic.  A width rather than a
   duty because it stays constant as the scan moves the frequency, and
   because the switching pair responds to width.  Clamped to half a
   period. */
void     sense_set_drive_ns(uint32_t ns);
uint32_t sense_drive_ns(void);        /* what was asked for */
uint32_t sense_drive_actual_ns(void); /* what the timer will produce */
uint32_t sense_drive_level(void);     /* in PWM counts, 0 if off */
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
   print a table.  Manual inspection; sense_find_resonance does the real
   calibration. */
void sense_sweep(uint32_t from_hz, uint32_t to_hz, uint32_t step_hz,
                 uint32_t dwell_ms);

/* What a resonance scan found. */
typedef struct _sense_resonance
{
  bool     valid;
  bool     saturated;      /* the envelope railed - readings are clipped  */
  bool     edge;           /* the peak sat at the edge of the search span */
  uint32_t f0_hz;          /* peak, parabolically interpolated            */
  uint32_t f_lo_hz;        /* lower half-power point                      */
  uint32_t f_hi_hz;        /* upper half-power point                      */
  uint32_t q_x10;          /* f0 / (f_hi - f_lo), times ten               */
  uint16_t peak_adc;
  uint16_t floor_adc;      /* envelope reading far off resonance          */
} sense_resonance;

/* Find the sense tank's resonance.  Run this with nothing metallic near
   the coil - the bob, a hand, a steel ruler will all pull it.

   Two passes: a coarse scan across the whole span to locate the peak and
   gauge its width, then a fine scan over about three linewidths centred on
   it.  The peak frequency comes from a parabolic fit to the three points
   around the fine maximum, so it is not limited to the step size, and the
   half-power points come from linear interpolation across the fine scan.

   Leaves the drive at the frequency it found and records it in cfg. */
bool sense_find_resonance(uint32_t lo_hz, uint32_t hi_hz, bool plot,
                          sense_resonance *out);

/* The points of the last scan's fine pass, and its result, so the web
   interface can draw the curve instead of only quoting a number. */
uint32_t sense_scan_count(void);
bool     sense_scan_point(uint32_t i, uint32_t *hz, uint16_t *adc);
const sense_resonance *sense_last_resonance(void);

/* Capture the raw amplified tank waveform on ADC1 and print it. */
void sense_capture(uint32_t rate_hz, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* _SENSE_H */
