/* control.c */

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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "board.h"
#include "config.h"
#include "sense.h"
#include "drive.h"
#include "timebase.h"
#include "control.h"

/* An event this far from where it was expected is not the one we think it
   is; give up and re-acquire rather than lock onto nonsense. */
#define MAX_SLIP_EVENTS     10ll

static control_state st = CTRL_IDLE;

/* The schedule is kept in EVENTS, not swings: a sense coil at the centre
   of the swing reports twice per period, one at an extreme reports once,
   and the loop should not care which.  cfg_event_ratio gives the interval
   as an exact rational so the accumulator never drifts. */
static uint64_t ev_whole_ns;            /* num / den, integer part        */
static uint64_t ev_rem;                 /* and remainder, over den        */
static uint64_t ev_den;

static uint64_t exp_ns;                 /* expected UTC of the next event */
static uint64_t exp_frac;
static uint64_t events;
static uint32_t missed;

static int64_t  integ;
static int64_t  cmd_ns;
static int64_t  credit_ns;
static int64_t  target_off;
static int64_t  last_err;
static int64_t  filt_err;
static int64_t  drift_ppb;

static uint32_t acq_run;
static uint64_t acq_prev_utc;

/* MEASURE mode */
static uint32_t meas_left, meas_total;
static bool     meas_retard;
static int64_t  meas_err0, meas_drift0;
static uint32_t meas_fired;

static bool ev_echo;

void control_set_echo(bool on) { ev_echo = on; }
bool control_echo(void) { return ev_echo; }

static uint64_t event_ns_nominal(void)
{
  return cfg_event_interval_ns();
}

static void exp_advance(int64_t k)
{
  if (k <= 0) return;
  exp_ns += (uint64_t)k * ev_whole_ns;
  {
    uint64_t r = exp_frac + (uint64_t)k * ev_rem;
    exp_ns  += r / ev_den;
    exp_frac = r % ev_den;
  }
}

static void recompute_constants(void)
{
  uint64_t num;
  cfg_event_ratio(&num, &ev_den);
  if (ev_den == 0ull) ev_den = 1ull;
  ev_whole_ns = num / ev_den;
  ev_rem      = num % ev_den;
}

void control_init(void)
{
  recompute_constants();
  control_reset();
  st = cfg.control_enabled ? CTRL_ACQUIRE : CTRL_IDLE;
}

void control_reset(void)
{
  exp_ns = 0; exp_frac = 0; events = 0; missed = 0;
  integ = 0; cmd_ns = 0; credit_ns = 0;
  last_err = 0; filt_err = 0; drift_ppb = 0;
  acq_run = 0; acq_prev_utc = 0;
  meas_left = 0; meas_fired = 0;
  if (st == CTRL_TRACK || st == CTRL_MEASURE) st = CTRL_ACQUIRE;
}

void control_enable(bool on)
{
  cfg.control_enabled = on ? 1u : 0u;
  if (on) { if (st == CTRL_IDLE) { control_reset(); st = CTRL_ACQUIRE; } }
  else    { st = CTRL_IDLE; drive_all_off(); }
}

void control_set_offset_ns(int64_t o) { target_off = o; }

const char *control_state_name(control_state s)
{
  switch (s)
  {
    case CTRL_IDLE:    return "idle";
    case CTRL_ACQUIRE: return "acquire";
    case CTRL_TRACK:   return "track";
    case CTRL_HOLD:    return "hold";
    case CTRL_MEASURE: return "measure";
  }
  return "?";
}

bool control_measure_authority(uint32_t n, bool retard)
{
  if (st != CTRL_TRACK) return false;
  if (n == 0u || n > 500u) return false;
  meas_left   = n;
  meas_total  = n;
  meas_retard = retard;
  meas_err0   = last_err;
  meas_drift0 = (drift_ppb * (int64_t)event_ns_nominal()) / 1000000000ll;
  meas_fired  = 0;
  st = CTRL_MEASURE;
  printf("measuring %s authority over %lu events\r\n",
         retard ? "retard" : "advance", (unsigned long)n);
  return true;
}

/* Put one pulse where it belongs relative to this event.  How long after a
   sense event the bob reaches the DRIVE coil depends entirely on where the
   two coils were placed, so it is configuration: parts per thousand of a
   full period.  Opposite extremes is 500, the same extreme is 0, sense at
   the centre with drive at an extreme is 250. */
static void fire_for(const sense_event *ev, bool retard)
{
  uint64_t period_us = cfg_period_ns() / 1000ull;
  uint64_t offset_us = (period_us * (uint64_t)cfg.drive_offset_ppt) / 1000ull;
  uint64_t arrive    = ev->t_us + offset_us;
  uint64_t when;

  if (retard) when = arrive + cfg.pulse_retard_us;
  else
  {
    /* An advance pulse leads the arrival, and with the coils at the same
       place that would be in the past - wait for the next period. */
    while (arrive < ev->t_us + cfg.pulse_advance_us) arrive += period_us;
    when = arrive - cfg.pulse_advance_us;
  }
  drive_pulse_at(when, cfg.pulse_us);
}

static void track_event(const sense_event *ev)
{
  int64_t d, k, err, limit;

  d = (int64_t)ev->utc_ns - (int64_t)exp_ns;
  {
    int64_t iv = (int64_t)event_ns_nominal();
    k = (d >= 0) ? ((d + iv / 2) / iv) : ((d - iv / 2) / iv);
  }

  if (k > MAX_SLIP_EVENTS || k < 0)
  {
    /* Either the detector is producing nonsense or we have drifted more
       than half an interval from where we thought we were.  Either way
       this is not the event we indexed, and the schedule only moves
       forward. */
    st = CTRL_HOLD;
    return;
  }
  if (k > 0) { exp_advance(k); missed += (uint32_t)k; }

  err = (int64_t)ev->utc_ns - (int64_t)exp_ns - target_off;
  last_err = err;
  filt_err += (err - filt_err) / 8;
  events++;
  exp_advance(1);

  /* A running estimate of how far the pendulum itself is off nominal,
     which is what the loop is having to cancel. */
  {
    uint64_t mean_us = sense_mean_interval_us(64);
    if (mean_us)
    {
      int64_t nom = (int64_t)event_ns_nominal();
      int64_t got = (int64_t)mean_us * 1000ll;
      drift_ppb = ((got - nom) * 1000000000ll) / nom;
    }
  }

  if (st == CTRL_MEASURE)
  {
    if (meas_left > 0u)
    {
      fire_for(ev, meas_retard);
      meas_fired++;
      meas_left--;
      if (meas_left == 0u)
      {
        int64_t step   = err - meas_err0;
        int64_t nat    = meas_drift0 * (int64_t)meas_total;
        int64_t corr   = step - nat;
        int64_t per    = meas_fired ? corr / (int64_t)meas_fired : 0;
        int64_t mag = (per < 0) ? -per : per;
        printf("%s authority: %lu pulses, phase moved %lld us, natural drift %lld us,\r\n"
               "           corrected %lld us -> %lld ns per pulse\r\n",
               meas_retard ? "retard" : "advance",
               (unsigned long)meas_fired, (long long)(step / 1000),
               (long long)(nat / 1000), (long long)(corr / 1000), (long long)per);
        /* Offer the exact command, with the other direction left as it is. */
        printf("           AUTH %lld %lld   then SAVE to keep it\r\n",
               (long long)(meas_retard ? cfg.auth_advance_ns : mag),
               (long long)(meas_retard ? mag : cfg.auth_retard_ns));
        st = CTRL_TRACK;
      }
    }
    return;
  }

  /* --- PI on phase ---------------------------------------------------- */
  limit = ((int64_t)cfg.slew_limit_ppm * (int64_t)event_ns_nominal()) / 1000000ll;
  if (limit < 0) limit = -limit;

  {
    int64_t kp = (int64_t)(cfg.kp_swings ? cfg.kp_swings : 4200u);
    int64_t ki = (int64_t)(cfg.ki_swings ? cfg.ki_swings : 12600u);
    int64_t p, i;

    integ += err;
    p = -err / kp;
    i = -integ / (ki * ki);
    cmd_ns = p + i;

    if (cmd_ns > limit)  { cmd_ns = limit;  integ -= err; }   /* anti-windup */
    if (cmd_ns < -limit) { cmd_ns = -limit; integ -= err; }
  }

  /* --- spend the demand in whole pulses -------------------------------- */
  credit_ns += cmd_ns;

  if (cfg.control_enabled)
  {
    /* Positive credit means the hands are ahead and want retarding.  Each
       direction is spent at its own measured price; a direction that has
       not been measured simply cannot be spent, which is what lets a
       retard-only installation still discipline a clock that gains. */
    int64_t ar = (int64_t)cfg.auth_retard_ns;
    int64_t aa = (int64_t)cfg.auth_advance_ns;

    if (ar > 0 && credit_ns >= ar)       { fire_for(ev, true);  credit_ns -= ar; }
    else if (aa > 0 && credit_ns <= -aa) { fire_for(ev, false); credit_ns += aa; }

    /* Never let the credit run away if the actuator cannot or will not
       deliver - including the case where that direction was never measured. */
    if (ar > 0 && credit_ns >  8 * ar) credit_ns =  8 * ar;
    if (aa > 0 && credit_ns < -8 * aa) credit_ns = -8 * aa;
  }
}

static void acquire_event(const sense_event *ev)
{
  int64_t  nom  = (int64_t)event_ns_nominal();
  uint32_t need = cfg.acquire_events ? cfg.acquire_events : 12u;

  if (acq_prev_utc != 0ull)
  {
    int64_t gap = (int64_t)ev->utc_ns - (int64_t)acq_prev_utc;
    int64_t tol = (nom * (int64_t)(cfg.acquire_tol_pct ? cfg.acquire_tol_pct : 4u)) / 100ll;
    if (gap > nom - tol && gap < nom + tol) acq_run++;
    else acq_run = 0;
  }
  acq_prev_utc = ev->utc_ns;

  if (acq_run >= need)
  {
    /* Anchor the schedule on this event, so the loop starts at zero error
       and only has to hold it there. */
    exp_ns   = ev->utc_ns;
    exp_frac = 0;
    events   = 0;
    missed   = 0;
    integ    = 0;
    cmd_ns   = 0;
    credit_ns = 0;
    last_err = 0;
    filt_err = 0;
    exp_advance(1);
    st = CTRL_TRACK;
  }
}

void control_poll(void)
{
  sense_event ev;

  while (sense_next_event(&ev))
  {
    if (ev_echo)
      printf("ev %lu  t %llu us  peak %u  base %u  width %lu us  gap %lld us\r\n",
             (unsigned long)ev.seq, (unsigned long long)ev.t_us,
             (unsigned)ev.peak, (unsigned)ev.baseline,
             (unsigned long)ev.width_us,
             (long long)sense_mean_interval_us(1));

    if (st == CTRL_IDLE) continue;

    if (!tb_have_time() || ev.utc_ns == 0ull)
    {
      if (st != CTRL_HOLD) { st = CTRL_HOLD; }
      continue;
    }

    if (st == CTRL_HOLD) { st = CTRL_ACQUIRE; acq_run = 0; acq_prev_utc = 0; }

    if (st == CTRL_ACQUIRE) acquire_event(&ev);
    else                    track_event(&ev);
  }

  /* No events for several swings means the detector has lost the bob. */
  if ((st == CTRL_TRACK || st == CTRL_MEASURE) && sense_last_event_us() != 0ull)
  {
    uint64_t quiet = time_us_64() - sense_last_event_us();
    if (quiet > (event_ns_nominal() / 1000ull) * 5ull)
    {
      st = CTRL_HOLD;
      drive_all_off();
    }
  }
}

void control_stats_get(control_stats *o)
{
  memset(o, '\0', sizeof(*o));
  o->state            = st;
  o->events           = events;
  o->err_ns           = last_err;
  o->filt_err_ns      = filt_err;
  o->cmd_ns_per_swing = cmd_ns;
  o->credit_ns        = credit_ns;
  o->drift_ppb        = drift_ppb;
  o->pulses           = drive_pulse_count();
  o->missed           = missed;
  o->target_offset_ns = target_off;
}
