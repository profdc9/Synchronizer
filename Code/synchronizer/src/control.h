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
   KICK is a bang-bang hysteresis scheme, not a PI controller: no gain to
   tune and no pulse authority to measure first.  It waits until the
   schedule error (sched_err_ns in control.c) crosses a threshold, then
   fires a pulse every kick_min_swings events - advancing or retarding,
   whichever the threshold it crossed calls for - until the error is back
   past zero.  See kick_step()'s comment in control.c for the details.

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
  CTRL_HOLD         /* lost events or lost time; actuator quiet           */
} control_state;

typedef struct _control_stats
{
  control_state state;
  uint64_t events;            /* sense events counted since lock         */
  int64_t  err_ns;            /* latest phase error                      */
  int64_t  filt_err_ns;       /* smoothed, for display                   */
  int64_t  cmd_ns_per_swing;  /* current controller output               */
  int64_t  drift_ppb;         /* measured pendulum error vs nominal      */
  int64_t  ff_ns;             /* feedforward part of the command         */
  uint32_t rate_n;            /* events the rate tracker has seen        */
  uint8_t  rate_ready;        /* 1 once feedforward is being applied     */
  uint32_t pulses;
  uint32_t missed;            /* events the detector did not report      */
  uint8_t  kick_active;       /* KICK mode: currently in the correcting
                                  phase, vs idle waiting to cross back    */
  uint8_t  kick_dir_retard;   /* KICK mode: which way THIS episode is
                                  going - only meaningful if kick_active   */
  uint32_t kick_since;        /* KICK mode: swings since the last pulse  */
  int64_t  kick_filt_ns;      /* KICK mode: short EMA of demod_ns - now a
                                  diagnostic only, not what fires anything */
  int64_t  kick_accum_ns;     /* KICK mode: running sum of kick_filt_ns -
                                  also diagnostic only, see sched_err_ns   */
  uint32_t locked_s;          /* seconds of NTP time since the loop last
                                  entered TRACK; 0 when not locked        */
  int64_t  sched_err_ns;      /* fast-smoothed ev->utc_ns - nominal_ns:
                                  what KICK's hysteresis reacts to         */
} control_stats;

void control_init(void);
void control_enable(bool on);
void control_poll(void);              /* call from the main loop         */
void control_stats_get(control_stats *out);
void control_reset(void);

/* The ADC is about to be taken away for a diagnostic, so events will stop
   arriving.  Drop to hold rather than let the silence be read as a phase
   excursion.  The existing hold path re-acquires by itself once events
   resume.  Returns what it interrupted, so the diagnostic can say so. */
control_state control_blind(void);

/* Zero KICK mode's active/idle latch and since-last-kick counter.  Call
   whenever KICK's own parameters change: state built up under the old
   settings should not carry over into the new ones. */
void control_kick_reset(void);

/* Test only: seed sched_err_ns ("uncorrected error") directly, so KICK's
   hysteresis reacts to it on the very next tracked event instead of
   waiting for a real error of that size to occur naturally.  Useful for
   exercising the advance side of KICK on a clock that in practice only
   ever drifts fast enough to need retarding.  Decays back toward whatever
   the real, measured schedule error is at the usual kick_min_swings-event
   pace, same as any other sample fed into that EMA. */
void control_force_sched_err_ns(int64_t ns);

const char *control_state_name(control_state s);

/* Echo every detected swing to the console as it is processed.  Events are
   consumed here, so this is the only place they can be watched from. */
void control_set_echo(bool on);
bool control_echo(void);

/* A periodic one-line summary instead of WATCH's per-event flood: phase
   error and how many corrective pulses KICK fired since the previous
   line, once every `secs` seconds.  0 turns it off. */
void control_set_phaselog(uint32_t secs);
uint32_t control_phaselog(void);

/* CONTROL Y/N is the master switch - N idles the whole loop, including the
   phase/rate tracking.  This is a narrower one: it mutes only KICK's
   actual corrective pulse, leaving tracking, kick_active/kick_since
   bookkeeping, and PHASELOG's numbers running exactly as if it were on.
   Not saved to flash - defaults to on at every boot, same as CONTROL
   defaults to off; the two are independent and this one only does
   anything while CONTROL is on. */
void control_set_actuator(bool on);
bool control_actuator(void);

/* ERRHIST: the web page's error-vs-swing-number graph reads through
   these.  control_errhist_seq() is the current cursor; control_errhist_
   read() fills err_us_out[]/pulse_out[] (pulse: 0 none, 1 advance,
   2 retard) with up to n samples from `from` onward and reports the
   cursor to pass next time, same idiom as cli_log_read().  See the
   errhist comment in control.c for the ring's size and cadence. */
uint32_t control_errhist_seq(void);
uint32_t control_errhist_read(uint32_t from, int32_t *err_us_out, int8_t *pulse_out,
                              uint32_t n, uint32_t *next);

#ifdef __cplusplus
}
#endif

#endif /* _CONTROL_H */
