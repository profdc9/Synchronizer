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
   arrives.

   This is a FLOOR, not typical slack: fire_for()'s wrap-forward loop stops
   at the first period boundary that clears it, so a placement whose
   centre-relative offset happens to land just above the floor gets almost
   none of this margin, while a neighbour ten thousand microseconds away
   can get a whole extra period of it for free just because it fell on the
   other side of the boundary.  A 20 ms floor and a stalled USB-CDC
   printf() (event echo, easily tens of ms if the host is slow to drain
   it) are enough to lose that thin sliver outright - reproducibly, since
   it is the same placement landing on the same side of the same
   boundary every time, not ordinary jitter.  Widened so an ordinary
   stall still leaves room. */
#define PULSE_LEAD_US       50000ull

static control_state st = CTRL_IDLE;

static uint64_t events;
static uint32_t missed;

/* --- the pendulum's own rate, tracked --------------------------------- ---
   drift_ppb used to come from the mean of the last 64 intervals, and a mean
   of intervals is (t_N - t_0)/N: only the endpoints, throwing away 62 of the
   64 samples.  At 2.27 ms of per-event timing noise that is 58 ppm, 5 s/day,
   which is why the reported drift wandered by more than the error it was
   meant to describe.

   Instead, a second NCO that tracks the pendulum rather than UTC.  It runs
   free at the nominal interval plus a learned rate, a type-2 loop pulls it
   onto the measured events, and the learned rate IS the period estimate -
   with every event contributing and the loop's whole integration behind it.

   This used to be purely a measuring instrument, feeding only feedforward,
   while a SEPARATE, undisciplined accumulator (exp_ns, since removed) ran
   the loss-of-lock check at a fixed nominal rate with no correction ever
   reaching it - so it drifted from real UTC by design, at whatever rate
   the pendulum differs from nominal, and eventually crossed the slip
   threshold on its own with no actual disturbance behind it.  rerr, this
   NCO's own residual, is what the slip check measures against now - it is
   already continuously disciplined, so it stays bounded under normal
   tracking and a real anomaly still shows up in it just as clearly.

   A note on the crystal.  Both this and the phase error are measured in
   disciplined UTC ns, so a crystal error is common-mode between them and
   the discipline path is self-correcting.  But the cancellation is only
   exact if the two estimators average over the same interval, and the
   timebase learns its ppb from NTP fixes minutes apart.  A crystal
   excursion faster than that leaks in here as apparent pendulum rate.  So
   this loop is deliberately kept slower than the timebase's rate tracking:
   a temperature transient the timebase has not caught yet is then mostly
   averaged away rather than fed forward as a rate change that is not
   really the pendulum's. */
#define RATE_SCALE      1024ll       /* rate is carried as ns per 1024 events */
static uint64_t rate_ns;             /* the tracking NCO's predicted event   */
static int64_t  rate_acc;            /* its fractional accumulator           */
static int64_t  rate_q;              /* learned offset, ns per 1024 events   */
static bool     rate_have;
/* Where swing N SHOULD land if the clock were exactly correct since
   acquisition: anchor_utc_ns + N * nominal_interval, advanced by the same
   fixed nom every event (and by k*nom when swings are folded in, same as
   rate_ns) - nothing here ever learns or adapts, so a persistent rate
   error cannot cancel itself out against it the way it does in rerr once
   rate_q has learned it. */
static uint64_t nominal_ns;
/* KICK's feedback signal: a fast-smoothed ev->utc_ns - nominal_ns, NOT
   rate_ns - nominal_ns - see the computation in track_event() for why
   rate_ns (ki-driven, tens of thousands of events to respond) was too
   slow to see its own corrections working before massively overshooting. */
static int64_t  sched_err_ns;
static int64_t  ff_last;
static uint32_t rate_n;              /* events since the tracker started     */
static int64_t  cmd_ns;
static int64_t  target_off;
static int64_t  last_err;
static int64_t  filt_err;
static int64_t  drift_ppb;

/* --- KICK mode: a one-directional hysteresis scheme that needs no
   measured authority at all - see kick_step() below. */
static bool     kick_active;
static bool     kick_dir_retard;     /* which way THIS correction episode
                                         is going - set when kick_active
                                         trips, meaningless while idle.  No
                                         longer a user setting: the
                                         controller picks direction itself
                                         from which threshold it hit. */
static uint32_t kick_since;
static int64_t  kick_filt_ns;       /* short EMA of demod_ns, time constant
                                        kick_min_swings - a single raw sample
                                        is noisy enough to flip sign event to
                                        event, so this is what actually feeds
                                        the accumulator below, not demod_ns
                                        itself.                            */
static int64_t  kick_accum_ns;      /* running sum of kick_filt_ns, never
                                        decayed - KICK's hysteresis reacts to
                                        THIS sign, not the EMA's.  A working
                                        kick changes the pendulum, so this
                                        crosses back through zero on its own
                                        once corrections are actually landing;
                                        it only grows without bound if they
                                        are not (wrong direction, too little
                                        authority per pulse, or not being
                                        delivered at all) - which is exactly
                                        the failure worth seeing, not
                                        something to average away.  EMA-then-
                                        sum converges to the same running
                                        total as summing demod_ns directly
                                        (bounded difference, not growing);
                                        the EMA stage only buys faster, less
                                        noisy convergence right after a
                                        reset, before the sum itself has had
                                        enough terms to average noise out on
                                        its own.                           */

static uint32_t acq_run;
static uint64_t acq_prev_utc;

static bool ev_echo;
static bool actuator_on = true;    /* CONTROL Y/N idles the whole loop -
                                       this mutes only the corrective
                                       pulses; tracking runs either way   */

void control_set_echo(bool on) { ev_echo = on; }
bool control_echo(void) { return ev_echo; }

void control_set_actuator(bool on) { actuator_on = on; }
bool control_actuator(void) { return actuator_on; }

/* --- PHASELOG: a periodic one-line summary, independent of WATCH's
   per-event echo (which is far too verbose to leave running unattended -
   see the console-bytes-dropped counter in STATUS once it has been on for
   a while).  This is meant to be left on for hours: phase error and how
   many corrective pulses KICK fired since the last line, once every
   phaselog_secs. */
static uint32_t phaselog_secs;      /* 0 = off                            */
static uint64_t phaselog_last_us;
static uint32_t disc_pulses;        /* corrective pulses fired by KICK's
                                        kick_step - not manual PULSE       */
static uint32_t phaselog_last_pulses;

void control_set_phaselog(uint32_t secs)
{
  phaselog_secs = secs;
  phaselog_last_us = 0ull;          /* rearm: next line is a full interval
                                        from now, not whatever is left of
                                        an old one */
  phaselog_last_pulses = disc_pulses;
}

uint32_t control_phaselog(void) { return phaselog_secs; }

static uint64_t event_ns_nominal(void)
{
  return cfg_event_interval_ns();
}

void control_init(void)
{
  control_reset();
  st = cfg.control_enabled ? CTRL_ACQUIRE : CTRL_IDLE;
}

void control_reset(void)
{
  events = 0; missed = 0;
  cmd_ns = 0;
  rate_have = false; rate_q = 0; rate_acc = 0; rate_n = 0;
  last_err = 0; filt_err = 0; drift_ppb = 0;
  kick_active = false; kick_dir_retard = false; kick_since = 0; kick_filt_ns = 0; kick_accum_ns = 0;
  nominal_ns = 0; sched_err_ns = 0;
  acq_run = 0; acq_prev_utc = 0;
  if (st == CTRL_TRACK) st = CTRL_ACQUIRE;
}

void control_enable(bool on)
{
  cfg.control_enabled = on ? 1u : 0u;
  if (on) { if (st == CTRL_IDLE) { control_reset(); st = CTRL_ACQUIRE; } }
  else    { st = CTRL_IDLE; drive_all_off(); }
}

void control_set_offset_ns(int64_t o) { target_off = o; }

/* Zero KICK's active/idle latch and since-last-kick counter.  Call whenever
   KICK's own parameters change: state built up under the old settings
   should not carry over into the new ones. */
void control_kick_reset(void)
{
  kick_active = false; kick_dir_retard = false; kick_since = 0; kick_filt_ns = 0; kick_accum_ns = 0;
}

/* See doc comment in control.h - test only. */
void control_force_sched_err_ns(int64_t ns)
{
  sched_err_ns = ns;
}

const char *control_state_name(control_state s)
{
  switch (s)
  {
    case CTRL_IDLE:    return "idle";
    case CTRL_ACQUIRE: return "acquire";
    case CTRL_TRACK:   return "track";
    case CTRL_HOLD:    return "hold";
  }
  return "?";
}

/* Whatever forced this - lost NTP time, a schedule slip too large to
   trust, the detector going quiet, or an explicit control_blind() - the
   loop is leaving TRACK/ACQUIRE in a way nothing downstream expects.
   Every site that forces CTRL_HOLD goes through here so none of them can
   forget to say so. */
static void enter_hold(const char *why)
{
  control_state was = st;

  /* Only worth a line when it actually cost a lock, not on every explicit
     CONTROL N or the one hold before the first fix ever lands. */
  if (was == CTRL_TRACK)
    printf("hold: %s\r\n", why);
  st = CTRL_HOLD;
  drive_all_off();             /* nothing queued should fire while blind */
}

control_state control_blind(void)
{
  control_state was = st;

  if (st == CTRL_TRACK || st == CTRL_ACQUIRE)
    enter_hold("blind requested");
  return was;
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
   the drive coil and arrives sooner.

   Returns whether the pulse was actually accepted.  drive_pulse_at() can
   refuse - too wide, or the duty budget spent - and every caller used to
   assume success: a MEASURE counted a refused pulse the same as a real one,
   so its "per pulse" figure was silently divided by more pulses than were
   ever delivered, and the credit-spending path deducted a price for a
   correction that never happened.  Both bugs are invisible in STATUS
   (fired/refused are counted, just never cross-checked against what a
   caller assumed), and both look identical to ordinary measurement noise
   from the outside - which is exactly what made them so easy to blame on
   the statistics instead of the plumbing. */
static bool fire_for(const sense_event *ev, bool retard)
{
  uint64_t period_us = cfg_period_ns() / 1000ull;
  uint64_t offset_us = (period_us * (uint64_t)cfg.drive_offset_ppt) / 1000ull;
  int64_t  centre    = (int64_t)(ev->t_us + offset_us);   /* bob at the drive coil */
  int64_t  signed_when;
  uint64_t when;

  /* Both placements are the SAME signed offset from this same centre now -
     see the comment on the config fields.  Retard and advance no longer
     use opposite-signed formulas; the only difference is which stored
     value gets read, so a given number always means the same physical
     instant whether it is being tested or spent as a retard placement or
     an advance one. */
  signed_when = centre + (retard ? (int64_t)cfg.pulse_retard_us
                                 : (int64_t)cfg.pulse_advance_us);

  /* Whatever that worked out to, it has to be far enough ahead to schedule -
     and "far enough ahead of ev->t_us" is not the same thing as "far enough
     ahead of now".  ev->t_us is the event's MIDPOINT, but the event is not
     reported here until its falling edge completes, at roughly
     t_us + width_us/2 - on this clock's ~400 ms wide events, close to
     200 ms after the timestamp being measured against.  A placement close
     enough to t_us to still clear the old fixed PULSE_LEAD_US margin can
     already be in the past by the time drive_pulse_at() actually checks it
     against time_us_64(), and every one of those pulses gets silently
     refused - which is exactly what a placement near +-half a period from
     centre can now reach, and the old design never could. */
  while (signed_when < (int64_t)(ev->t_us + ev->width_us / 2u + PULSE_LEAD_US))
    signed_when += (int64_t)period_us;
  when = (uint64_t)signed_when;

  /* Each direction gets its own width, set together by PW - see the
     config.h comment on pulse_advance_width_us/pulse_retard_width_us for
     why one width for both is the wrong shape given how asymmetric the
     two placements already are.  0 means that direction has deliberately
     been left unset, same idiom as the old AUTH prices: refuse rather
     than guess a width, which is what lets a retard-only installation
     (the common case - an attract-only coil advances far more weakly)
     leave advance at 0 and safely never act on it. */
  {
    uint32_t width = retard ? cfg.pulse_retard_width_us : cfg.pulse_advance_width_us;
    if (!width) return false;
    return drive_pulse_at(when, width);
  }
}

/* KICK mode: a bang-bang hysteresis scheme that needs no measured
   authority at all.  It fires no more often than every kick_min_swings
   events.  There is no separate "advance mode" or "retard mode" to set -
   it picks direction itself, from whichever threshold it hit:

     - error at or past -thr (ahead of true time by kick_threshold_pct of
       a swing or more): start retarding, and keep retarding every
       kick_min_swings swings until the error is back above zero - not
       out to +thr, just past zero, since the trigger threshold's only
       job is to ignore ordinary measurement noise near zero, not to
       demand a full swing back the other way before it will stop.
     - error at or past +thr: start advancing the same way, releasing
       once it is back below zero.

   sched_err_ns follows rerr/filt_err's sign convention (built from the
   same rate_ns), which is the OPPOSITE of demod_ns's: negative means the
   hands are ahead (fast) and want retarding, positive means they are
   behind (slow) and want advancing.  See its declaration above
   track_event() for what it actually measures and why filt_err and
   demod_ns were each tried and rejected for this job before it. */
static void kick_step(const sense_event *ev)
{
  uint32_t need = cfg.kick_min_swings ? cfg.kick_min_swings : 5u;
  int64_t  nom  = (int64_t)event_ns_nominal();
  int64_t  thr  = (nom * (int64_t)(cfg.kick_threshold_pct
                                    ? cfg.kick_threshold_pct : 25u)) / 100ll;
  /* sched_err_ns: a fast-smoothed ev->utc_ns - nominal_ns - not rerr
     (which measures the tracking NCO against ITSELF and converges to zero
     once rate_q has learned whatever the pendulum is actually doing,
     correct or not), not demod_ns (noisy per-sample, and turned out
     sensitive to nearby magnets - see kick_filt_ns/kick_accum_ns above,
     kept as diagnostics only), and not rate_ns - nominal_ns either (tried
     first, but rate_ns's ki-driven integrator takes days to respond, so
     it stayed blind to its own corrections and massively overshot).
     nominal_ns never moves, so a persistent error still cannot cancel
     itself out here - it just keeps growing until a real correction
     lands - but this responds to that correction in a few events instead
     of tens of thousands. */
  int64_t  fe   = sched_err_ns;

  if (kick_since < 0xffffffffu) kick_since++;

  if (!kick_active)
  {
    if      (fe <= -thr) { kick_active = true; kick_dir_retard = true;  }
    else if (fe >=  thr) { kick_active = true; kick_dir_retard = false; }
  }
  else if (kick_dir_retard) { if (fe > 0) kick_active = false; }
  else                      { if (fe < 0) kick_active = false; }

  if (actuator_on && kick_active && kick_since >= need &&
      fire_for(ev, kick_dir_retard))
    { kick_since = 0; disc_pulses++; }
}

static void track_event(const sense_event *ev)
{
  int64_t nom = (int64_t)event_ns_nominal();
  int64_t kp  = (int64_t)(cfg.rate_kp_events ? cfg.rate_kp_events : 350u);
  int64_t ki  = 2ll * kp * kp;
  int64_t rerr, k;

  /* Should not happen in practice - acquire_event() always seeds the rate
     NCO before handing off to CTRL_TRACK - but a defensive fallback costs
     nothing: seed it here instead of computing a slip check against a
     reference that was never anchored to anything. */
  if (!rate_have)
  {
    rate_ns = ev->utc_ns; nominal_ns = ev->utc_ns;
    rate_acc = 0; rate_q = 0; rate_n = 0;
    rate_have = true;
    events++;
    return;
  }

  /* --- the tracking NCO ------------------------------------------------
     Type 2: advance the prediction by one interval (plus whatever rate has
     already been learned), then see how far off it landed.  That residual,
     rerr, is now the loss-of-lock signal too - it is already continuously
     disciplined against the pendulum's real rate, so it staying near zero
     is the normal case and a value near a whole extra interval away is a
     genuine anomaly.  This replaces a separate, undisciplined accumulator
     (exp_ns) that only ever advanced at the fixed NOMINAL rate with no
     correction reaching it at all - so it drifted from utc_ns by design,
     at whatever rate the real pendulum differs from nominal, forever,
     until it crossed the slip threshold on its own and forced a "random"
     reacquire with no actual disturbance behind it. */
  rate_acc   += rate_q;
  rate_ns    += (uint64_t)(nom + rate_acc / RATE_SCALE);
  rate_acc   -= (rate_acc / RATE_SCALE) * RATE_SCALE;
  nominal_ns += (uint64_t)nom;

  rerr = (int64_t)ev->utc_ns - (int64_t)rate_ns;
  k    = (rerr >= 0) ? ((rerr + nom / 2) / nom) : ((rerr - nom / 2) / nom);

  if (ev_echo)
    printf("  utc %llu  rate_ns %llu  rerr %lld ns  k %lld\r\n",
           (unsigned long long)ev->utc_ns, (unsigned long long)rate_ns,
           (long long)rerr, (long long)k);

  if (k > MAX_SLIP_EVENTS || k < 0)
  {
    /* Either the detector is producing nonsense or we have drifted more
       than half an interval from where we thought we were.  Either way
       this is not the event we indexed, and the schedule only moves
       forward. */
    char why[40];
    snprintf(why, sizeof why, "schedule slip, k=%lld", (long long)k);
    enter_hold(why);
    return;
  }

  if (k > 0)
  {
    /* k swings were silently missed - fold their worth of nominal
       intervals into the prediction so the residual below is relative to
       THIS event, not still carrying the gap. */
    rate_ns    += (uint64_t)(k * nom);
    nominal_ns += (uint64_t)(k * nom);
    rerr       -= k * nom;
    missed     += (uint32_t)k;
  }

  last_err   = rerr;
  filt_err  += (rerr - filt_err) / 8;
  events++;

  rate_ns = (uint64_t)((int64_t)rate_ns + rerr / kp); /* phase pull  */
  rate_q += (rerr * RATE_SCALE) / ki;                 /* rate learn  */
  if (rate_n < 0xffffffffu) rate_n++;

  /* The learned offset, as parts per billion of the nominal interval. */
  drift_ppb = (rate_q * 1000000000ll) / (RATE_SCALE * nom);

  /* Direct comparison against the fixed schedule, smoothed with a short,
     fast average (time constant kick_min_swings) - NOT rate_ns, whose
     ki-driven integrator (2*kp*kp, tens of thousands of events) takes on
     the order of days to respond at this clock's rate.  A real correction
     shows up in ev->utc_ns within the next event or two; routing this
     through rate_ns instead meant the feedback signal stayed blind to its
     own corrections actually working for far longer than a correction
     episode lasts, which is what caused KICK to massively overshoot
     before it ever noticed and released. */
  {
    int64_t kn2 = (int64_t)(cfg.kick_min_swings ? cfg.kick_min_swings : 5u);
    int64_t raw = (int64_t)ev->utc_ns - (int64_t)nominal_ns;
    sched_err_ns += (raw - sched_err_ns) / kn2;
  }

  /* cmd_ns is a live phase-error readout for STATUS; it is not itself a
     controller output (see the KICK mode comment on kick_step()). */
  cmd_ns = ev->demod_ns;
  {
    int64_t kn = (int64_t)(cfg.kick_min_swings ? cfg.kick_min_swings : 5u);
    kick_filt_ns  += (ev->demod_ns - kick_filt_ns) / kn;
    kick_accum_ns += kick_filt_ns;
  }

  if (cfg.control_enabled) kick_step(ev);
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
    /* Anchor the rate NCO on this event, so the loop starts at zero error
       and only has to hold it there - rate_ns is what the slip check
       measures against now, so this is the reset that matters; without
       it the first event after a reacquire would compare against
       whatever rate_ns was frozen at when tracking last stopped, which
       free-runs every event track_event() is not called, not just the
       ones spent in CTRL_HOLD, and would report a huge, spurious slip on
       the very next event. */
    rate_ns    = ev->utc_ns;
    nominal_ns = ev->utc_ns;
    rate_have  = true;
    rate_acc   = 0;
    rate_q     = 0;
    rate_n     = 0;
    events    = 0;
    missed    = 0;
    cmd_ns    = 0;
    last_err  = 0;
    filt_err  = 0;
    drift_ppb = 0;
    kick_active     = false;
    kick_dir_retard = false;
    kick_since      = 0;
    kick_filt_ns  = 0;
    kick_accum_ns = 0;
    st = CTRL_TRACK;
  }
}

void control_poll(void)
{
  sense_event ev;

  while (sense_next_event(&ev))
  {
    /* fire_for() below has to schedule against this same event before its
       wrap-forward margin (PULSE_LEAD_US) runs out, and a USB-CDC printf()
       can block for tens of milliseconds if the host is slow to drain it -
       long enough to eat that margin outright for a placement that had
       little of it to begin with. */
    if (ev_echo)
      printf("ev %lu  t %llu us  peak %u  base %u  width %lu us  gap %lld us"
             "  demod %ld ns\r\n",
             (unsigned long)ev.seq, (unsigned long long)ev.t_us,
             (unsigned)ev.peak, (unsigned)ev.baseline,
             (unsigned long)ev.width_us,
             (long long)sense_mean_interval_us(1),
             (long)ev.demod_ns);

    if (st == CTRL_IDLE) continue;

    if (!tb_have_time() || ev.utc_ns == 0ull)
    {
      if (st != CTRL_HOLD) enter_hold("no UTC time on this event");
      continue;
    }

    if (st == CTRL_HOLD) { st = CTRL_ACQUIRE; acq_run = 0; acq_prev_utc = 0; }

    if (st == CTRL_ACQUIRE) acquire_event(&ev);
    else                    track_event(&ev);
  }

  /* No events for several swings means the detector has lost the bob. */
  if (st == CTRL_TRACK && sense_last_event_us() != 0ull)
  {
    uint64_t quiet = time_us_64() - sense_last_event_us();
    if (quiet > (event_ns_nominal() / 1000ull) * 5ull)
    {
      char why[40];
      snprintf(why, sizeof why, "no event for %llu us", (unsigned long long)quiet);
      enter_hold(why);
    }
  }

  if (phaselog_secs > 0u)
  {
    uint64_t now = time_us_64();
    if (phaselog_last_us == 0ull) phaselog_last_us = now;
    else if (now - phaselog_last_us >= (uint64_t)phaselog_secs * 1000000ull)
    {
      uint32_t fired = disc_pulses - phaselog_last_pulses;
      /* filt_err (edge-based, rerr) and kick_filt_ns (phase-based, demod_ns)
         have OPPOSITE sign conventions - see the comment on kick_filt_ns
         above.  Negate filt_err first so both are in "positive = hands
         ahead, wants retarding" terms before comparing; what is left is
         how much the two measurement methods actually disagree right now. */
      printf("phaselog: %s  err %lld us  filt %lld us  kickfilt %lld us"
             "  kickaccum %lld us  diff %lld us  uncorrected %lld us"
             "  %lu pulses in the last %lu s (%lu total)\r\n",
             control_state_name(st), (long long)(last_err / 1000),
             (long long)(filt_err / 1000), (long long)(kick_filt_ns / 1000),
             (long long)(kick_accum_ns / 1000),
             (long long)((-filt_err - kick_filt_ns) / 1000),
             (long long)(sched_err_ns / 1000),
             (unsigned long)fired,
             (unsigned long)phaselog_secs, (unsigned long)disc_pulses);
      phaselog_last_pulses = disc_pulses;
      phaselog_last_us = now;
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
  o->drift_ppb        = drift_ppb;
  o->ff_ns            = ff_last;
  o->rate_n           = rate_n;
  o->rate_ready       = (uint8_t)((rate_have && rate_n >
      4u * (cfg.rate_kp_events ? cfg.rate_kp_events : 350u)) ? 1u : 0u);
  o->pulses           = drive_pulse_count();
  o->missed           = missed;
  o->target_offset_ns = target_off;
  o->kick_active       = kick_active ? 1u : 0u;
  o->kick_dir_retard   = kick_dir_retard ? 1u : 0u;
  o->kick_since        = kick_since;
  o->kick_filt_ns      = kick_filt_ns;
  o->kick_accum_ns     = kick_accum_ns;
  o->sched_err_ns      = sched_err_ns;
}
