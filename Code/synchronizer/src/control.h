/* control.h - the discipline loop */

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

#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* What the loop is actually locking
   ---------------------------------
   The clock keeps correct time when each full swing takes exactly
   beats_per_period / beats_per_hour of an hour of real time.  The sense
   coil reports events_per_period times within that swing, so event number
   n should arrive at

       expected(n) = epoch + n * 3600e9 * beats_per_period
                             / (beats_per_hour * events_per_period)

   nanoseconds of UTC.  All three of those come from the configuration -
   nothing here assumes a particular movement or coil placement.

   and the phase error is simply actual - expected.  Negative means the bob
   arrived early, the pendulum is running fast, and the hands are gaining.

   How it corrects
   ---------------
   A PI controller on that error asks for a number of nanoseconds to be
   added to each swing.  That number is tiny - on the development clock,
   cancelling 11 s/day needs 113 microseconds of retard per swing - far too
   small to deliver as a pulse every swing.  So the demand is accumulated in a credit and spent whole:
   when the credit reaches the measured phase authority of one pulse, one
   pulse is fired and the credit is reduced by what it bought.  That makes
   the actuator a first-order sigma-delta, which is exactly the right shape
   for a quantised effector driving a high-Q oscillator.

   Where the pulse goes
   --------------------
   How long after a sense event the bob reaches the drive coil depends on
   where the two coils were placed, so it is configuration rather than an
   assumption: cfg.drive_offset_ppt, in parts per thousand of a full
   period.  Pulling the bob in before it arrives advances the swing;
   pulling back on it after it has turned retards the swing. */

typedef enum
{
  CTRL_IDLE = 0,    /* switched off                                      */
  CTRL_ACQUIRE,     /* waiting for a timebase and a run of clean events   */
  CTRL_TRACK,       /* locked and correcting                              */
  CTRL_HOLD,        /* lost events or lost time; actuator quiet           */
  CTRL_MEASURE      /* measuring the phase authority of one pulse         */
} control_state;

typedef struct _control_stats
{
  control_state state;
  uint64_t events;            /* sense events counted since lock         */
  int64_t  err_ns;            /* latest phase error                      */
  int64_t  filt_err_ns;       /* smoothed, for display                   */
  int64_t  cmd_ns_per_swing;  /* current controller output               */
  int64_t  credit_ns;         /* undelivered correction                  */
  int64_t  drift_ppb;         /* measured pendulum error vs nominal      */
  uint32_t pulses;
  uint32_t missed;            /* events the detector did not report      */
  int64_t  target_offset_ns;  /* deliberate offset of the hands          */
} control_stats;

void control_init(void);
void control_enable(bool on);
void control_poll(void);              /* call from the main loop         */
void control_stats_get(control_stats *out);
void control_reset(void);

/* Shift what the loop considers "on time", to walk the hands into
   agreement without touching them.  Applied through the same rate limit as
   everything else, so a large offset slews rather than jumps. */
void control_set_offset_ns(int64_t offset_ns);

/* Fire one pulse per event for n events and report the phase step, which
   is the loop gain.  Run this with the loop switched off. */
bool control_measure_authority(uint32_t swings, bool retard);

const char *control_state_name(control_state s);

/* Echo every detected swing to the console as it is processed.  Events are
   consumed here, so this is the only place they can be watched from. */
void control_set_echo(bool on);
bool control_echo(void);

#ifdef __cplusplus
}
#endif

#endif /* _CONTROL_H */
