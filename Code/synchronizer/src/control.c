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
/* A pulse has to be scheduled far enough ahead that the event has finished
   being processed.  The sense event is timestamped at the midpoint of a dip
   that can be a third of a period wide, so it is already old when it
   arrives. */
#define PULSE_LEAD_US       20000ull

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
/* An authority measurement has to separate two things a pulse does.  The
   phase kick we want is a STEP.  But the pulse also changes the swing
   amplitude, and amplitude changes the rate through circular error, which
   is a RAMP - and over a few dozen pulses the ramp integrates into a phase
   change far larger than the steps.  Subtracting a single drift snapshot
   taken before any of it happened cannot see that coming, which is why
   repeated measurements of the same direction came back +185 us and then
   -41 us per pulse.

   So watch the rate for a window BEFORE pulsing and again AFTER, take the
   mean of the two as the rate that applied during, and what is left over is
   the steps. */
static void pts_finish(void);

typedef enum { MP_NONE = 0, MP_PRE, MP_PULSE, MP_POST } meas_phase;
static meas_phase m_phase;
static uint32_t   m_w;              /* swings in each observation window */
static uint32_t   m_n;              /* swings of pulsing                 */
static uint32_t   m_left;
static int64_t    m_e0, m_e1, m_e2, m_e3;
static uint32_t meas_left, meas_total;
static bool     meas_retard;
static uint32_t meas_fired;

/* --- PTIME sweep: a MEASURE at each of several pulse placements --------- */
#define PTS_MAX      10u
static bool     pts_on;
static bool     pts_retard;
static uint32_t pts_lo, pts_hi, pts_steps, pts_i, pts_swings;
static uint32_t pts_settle;        /* swings still to wait before the next */
static uint32_t pts_us[PTS_MAX];
static int64_t  pts_ns[PTS_MAX];
static uint16_t pts_restore;

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

control_state control_blind(void)
{
  control_state was = st;

  if (st == CTRL_TRACK || st == CTRL_ACQUIRE || st == CTRL_MEASURE)
  {
    m_phase   = MP_NONE;       /* a measurement across a gap is garbage */
    pts_on    = false;         /* and so is a sweep built out of them    */
    st        = CTRL_HOLD;
    drive_all_off();           /* nothing queued should fire while blind */
  }
  return was;
}

/* Rate before, rate after, and what is left when the mean of the two is
   charged against the pulsing window.  Everything in ns; err is ns. */
static void meas_finish(void)
{
  int64_t rb   = (m_e1 - m_e0) / (int64_t)m_w;      /* ns of phase per swing */
  int64_t ra   = (m_e3 - m_e2) / (int64_t)m_w;
  int64_t rm   = (rb + ra) / 2;
  int64_t dur  = m_e2 - m_e1;                        /* phase across pulsing */
  int64_t ramp = rm * (int64_t)m_n;
  int64_t kick = dur - ramp;
  int64_t per  = meas_fired ? kick / (int64_t)meas_fired : 0;
  int64_t mag  = (per < 0) ? -per : per;

  m_phase = MP_NONE;
  st      = CTRL_TRACK;

  printf("%s authority over %lu pulses\r\n",
         meas_retard ? "retard" : "advance", (unsigned long)meas_fired);
  printf("   rate before %lld ns/swing, after %lld  (the pulses moved it %lld)\r\n",
         (long long)rb, (long long)ra, (long long)(ra - rb));
  printf("   phase across pulsing %lld us, of which %lld us was rate\r\n",
         (long long)(dur / 1000), (long long)(ramp / 1000));
  printf("   kick %lld us -> %lld ns per pulse\r\n",
         (long long)(kick / 1000), (long long)per);

  if (pts_on)
  {
    pts_us[pts_i] = pts_retard ? cfg.pulse_retard_us : cfg.pulse_advance_us;
    pts_ns[pts_i] = per;
    pts_i++;
    if (pts_i >= pts_steps) pts_finish();
    else pts_settle = 12u;          /* let the swing recover before the next */
    return;
  }

  printf("   AUTH %lld %lld   then SAVE to keep it\r\n",
         (long long)(meas_retard ? cfg.auth_advance_ns : mag),
         (long long)(meas_retard ? mag : cfg.auth_retard_ns));
}

static uint32_t pts_place(uint32_t i)
{
  return (pts_steps < 2u) ? pts_lo
       : pts_lo + ((pts_hi - pts_lo) * i) / (pts_steps - 1u);
}

static void pts_start_point(void)
{
  uint32_t us = pts_place(pts_i);
  if (pts_retard) cfg.pulse_retard_us  = (uint16_t)us;
  else            cfg.pulse_advance_us = (uint16_t)us;
  printf("  %lu of %lu: placement %lu us\r\n",
         (unsigned long)(pts_i + 1u), (unsigned long)pts_steps,
         (unsigned long)us);
  control_measure_authority(pts_swings, pts_retard);
}

static void pts_finish(void)
{
  uint32_t i, best = 0u;
  int64_t  bestmag = 0;

  pts_on = false;
  if (pts_retard) cfg.pulse_retard_us  = pts_restore;
  else            cfg.pulse_advance_us = pts_restore;

  for (i = 0; i < pts_steps; i++)
  {
    int64_t m = pts_ns[i] < 0 ? -pts_ns[i] : pts_ns[i];
    if (m > bestmag) { bestmag = m; best = i; }
  }

  printf("\r\n%s placement sweep\r\n", pts_retard ? "retard" : "advance");
  printf("%10s %14s\r\n", "us", "ns per pulse");
  for (i = 0; i < pts_steps; i++)
  {
    uint32_t k, bar = bestmag ? (uint32_t)((pts_ns[i] < 0 ? -pts_ns[i]
                                                          : pts_ns[i]) * 30 / bestmag)
                              : 0u;
    printf("%10lu %14lld |", (unsigned long)pts_us[i], (long long)pts_ns[i]);
    for (k = 0; k < bar; k++) putchar('#');
    printf("%s\r\n", (i == best) ? "  <-- best" : "");
  }
  printf("placement restored to %u us.  'PTIME %u %u' then SAVE to keep the best\r\n",
         pts_restore,
         pts_retard ? cfg.pulse_advance_us : pts_place(best),
         pts_retard ? pts_place(best) : cfg.pulse_retard_us);
  printf("note the signs - a placement that changes sign is on the wrong side\r\n");
}

bool control_ptime_scan(bool retard, uint32_t lo, uint32_t hi,
                        uint32_t steps, uint32_t swings)
{
  if (st != CTRL_TRACK) return false;
  if (steps < 2u) steps = 5u;
  if (steps > PTS_MAX) steps = PTS_MAX;
  if (swings < 5u) swings = 40u;
  if (swings > 200u) swings = 200u;
  if (lo == 0u) lo = 10000u;
  if (hi == 0u) hi = 60000u;
  if (hi <= lo || hi > 65000u) return false;

  pts_on      = true;
  pts_retard  = retard;
  pts_lo      = lo;
  pts_hi      = hi;
  pts_steps   = steps;
  pts_swings  = swings;
  pts_i       = 0u;
  pts_settle  = 0u;
  pts_restore = retard ? cfg.pulse_retard_us : cfg.pulse_advance_us;

  printf("sweeping %s placement %lu..%lu us in %lu steps, %lu swings each\r\n",
         retard ? "retard" : "advance", (unsigned long)lo, (unsigned long)hi,
         (unsigned long)steps, (unsigned long)swings);
  printf("this takes about %lu seconds; keep away from the clock\r\n",
         (unsigned long)((steps * (swings + 12u) * (cfg_period_ns() / 1000000ull)) / 1000ull));
  pts_start_point();
  return true;
}

bool control_measure_authority(uint32_t n, bool retard)
{
  if (st != CTRL_TRACK) return false;
  if (n == 0u || n > 500u) return false;

  m_n     = n;
  m_w     = n / 2u;
  if (m_w < 8u)  m_w = 8u;
  if (m_w > 60u) m_w = 60u;
  m_phase = MP_PRE;
  m_left  = m_w;
  m_e0    = last_err;

  meas_total  = n;
  meas_retard = retard;
  meas_fired  = 0;
  st = CTRL_MEASURE;
  printf("measuring %s authority: %lu swings idle, %lu pulsing, %lu idle"
         " (%lu s)\r\n",
         retard ? "retard" : "advance", (unsigned long)m_w,
         (unsigned long)n, (unsigned long)m_w,
         (unsigned long)(((2u * m_w + n) * (cfg_period_ns() / 1000000ull)) / 1000ull));
  return true;
}

/* Put one pulse where it belongs relative to this event.  How long after a
   sense event the bob reaches the DRIVE coil depends entirely on where the
   two coils were placed, so it is configuration: parts per thousand of a
   full period.  Opposite extremes is 500, the same extreme is 0, sense at
   the centre with drive at an extreme is 250.

   Which DIRECTION a pulse moves the clock is not about leading or trailing
   the bob.  An attract-only coil shifts the phase by an amount proportional
   to -x*dv, where x is displacement from the swing's centre: the sign
   follows which side of centre the bob is on, and has no velocity term at
   all.  So pulsing just before the bob arrives and just after it leaves are
   the same thing - both have the bob on the coil's side, and both retard.

   Measured on the development clock, when both used to fire either side of
   the drive coil: retard 14628 ns per pulse, "advance" 101228 ns per pulse,
   and both with the same sign.

   To advance, the coil has to pull the bob toward itself while the bob is
   on the FAR side, which is half a period from the drive coil's turning
   point - the sense coil's own extreme.  The bob is then accelerated toward
   the drive coil and arrives sooner. */
static void fire_for(const sense_event *ev, bool retard)
{
  uint64_t period_us = cfg_period_ns() / 1000ull;
  uint64_t offset_us = (period_us * (uint64_t)cfg.drive_offset_ppt) / 1000ull;
  uint64_t arrive    = ev->t_us + offset_us;   /* bob at the drive coil */
  uint64_t when;

  if (retard)
    when = arrive + cfg.pulse_retard_us;
  else
    when = arrive + period_us / 2ull - cfg.pulse_advance_us;

  /* Whatever that worked out to, it has to be far enough ahead to schedule. */
  while (when < ev->t_us + PULSE_LEAD_US) when += period_us;

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

  if (pts_on && pts_settle > 0u && st == CTRL_TRACK)
  {
    if (--pts_settle == 0u) pts_start_point();
    return;
  }

  if (st == CTRL_MEASURE)
  {
    switch (m_phase)
    {
      case MP_PRE:
        if (--m_left == 0u) { m_e1 = err; m_phase = MP_PULSE; m_left = m_n; }
        break;
      case MP_PULSE:
        fire_for(ev, meas_retard);
        meas_fired++;
        if (--m_left == 0u) { m_e2 = err; m_phase = MP_POST; m_left = m_w; }
        break;
      case MP_POST:
        if (--m_left == 0u) { m_e3 = err; meas_finish(); }
        break;
      default:
        st = CTRL_TRACK;
        break;
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
       deliver.  A direction that was never measured has no price of its own
       to bound it by, so borrow the other one's - otherwise the retard-only
       case, which is a supported configuration and the one a clock that
       gains actually needs, accumulates unbounded advance demand it can
       never spend, and then has to work off a phantom backlog before it
       fires again when the error finally reverses. */
    {
      int64_t hi = (ar > 0) ? ar : aa;
      int64_t lo = (aa > 0) ? aa : ar;
      if (hi > 0 && credit_ns >  8 * hi) credit_ns =  8 * hi;
      if (lo > 0 && credit_ns < -8 * lo) credit_ns = -8 * lo;
    }
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
