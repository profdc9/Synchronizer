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

static int64_t  integ;

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
static int64_t  ff_last;
static uint32_t rate_n;              /* events since the tracker started     */
static int64_t  cmd_ns;
static int64_t  credit_ns;
static int64_t  target_off;
static int64_t  last_err;
static int64_t  filt_err;
static int64_t  drift_ppb;

/* --- KICK mode: a one-directional hysteresis scheme that needs no
   measured authority at all - see kick_step() below. */
static bool     kick_active;
static uint32_t kick_since;

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
   the steps.

   Two things make that actually work, and neither was here at first.

   The signal is the tracking NCO's residual, not the phase error against
   UTC.  During a measurement the NCO stops learning and stops being pulled,
   so it free-runs at the rate it had learned before any of this started: a
   flywheel the pulses cannot move.  What is left in the residual is exactly
   what the pulses put in, with the pendulum's own rate already subtracted
   and no curvature from the discipline loop hunting underneath.  A pulse's
   phase kick is then a step in that residual and its amplitude effect a
   change of slope - separated by construction rather than by fitting after
   the fact.

   And each window is fitted by least squares rather than read off its
   endpoints.  With 2.27 ms of per-event timing noise and a kick of tens of
   microseconds, endpoint differencing is hopeless - it was why the same
   measurement came back with different answers.  A fit over N samples
   estimates the level to sigma/sqrt(N) and the slope to sigma*sqrt(12/N^3),
   and it also hands back the residual RMS, so the measurement can report
   its own error bar instead of leaving the reader to guess. */
static void pts_finish(void);

typedef enum { MP_NONE = 0, MP_PRE, MP_PULSE, MP_POST } meas_phase;
static meas_phase m_phase;
static uint32_t   m_w;              /* swings in each observation window */
static uint32_t   m_n;              /* swings of pulsing                 */
static uint32_t   m_left;
static int64_t    meas_resid;       /* coasting NCO's residual, ns       */
static bool     meas_retard;
static uint32_t meas_fired;

/* Straight-line least squares over y(k) = c + m*k, k = 0..n-1.  Slopes are
   carried as ns per THOUSAND events: the real thing is a fraction of a
   nanosecond per event and integer division would throw all of it away. */
typedef struct { uint32_t n; int64_t sy, sny, syy; } meas_fit;
static meas_fit m_f0, m_f1;         /* the before and after windows      */

static void fit_reset(meas_fit *f) { f->n = 0u; f->sy = 0; f->sny = 0; f->syy = 0; }

static void fit_add(meas_fit *f, int64_t y)
{
  f->sy  += y;
  f->sny += (int64_t)f->n * y;
  f->syy += y * y;
  f->n++;
}

static int64_t isqrt64(int64_t v)
{
  int64_t x, y;
  if (v <= 0) return 0;
  x = v; y = (x + 1) / 2;
  while (y < x) { x = y; y = (x + v / x) / 2; }
  return x;
}

static int64_t fit_slope(const meas_fit *f)   /* ns per 1000 events */
{
  int64_t n = (int64_t)f->n;
  if (n < 2) return 0;
  return (1000ll * (12ll * f->sny - 6ll * (n - 1ll) * f->sy)) / (n * (n * n - 1ll));
}

/* The fitted line, at half-event resolution so it can be evaluated on the
   boundary BETWEEN two events.  k2 is twice the sample index and is meant
   to run outside the window - extrapolating to the edges of the pulsing
   interval is the whole point of fitting. */
static int64_t fit_at(const meas_fit *f, int64_t k2)
{
  int64_t n = (int64_t)f->n;
  if (n < 1) return 0;
  if (n < 2) return f->sy;
  return f->sy / n + (fit_slope(f) * (k2 - (n - 1ll))) / 2000ll;
}

/* RMS of the residuals about the fit: the per-event timing noise, which is
   what every error bar below is built out of. */
static int64_t fit_sigma(const meas_fit *f)
{
  int64_t n = (int64_t)f->n, m, sse, sxx;
  if (n < 3) return 0;
  m   = fit_slope(f) / 1000ll;
  sxx = (n * (n * n - 1ll)) / 12ll;
  sse = f->syy - (f->sy / n) * f->sy - m * m * sxx;
  if (sse < 0) sse = 0;
  return isqrt64(sse / (n - 2ll));
}

/* One sigma on the per-pulse kick.

   Written out, the whole estimate collapses to something simple: the two
   window MEANS, differenced, minus the mean of the two slopes times the
   centre-to-centre separation W+M.  The extrapolations to the edges of the
   pulsing interval and the ramp charged against it are the same slope term
   twice, and they add rather than being independent - which is why treating
   them separately understated the error bar.

   var = sigma^2 * [ 2/W + 6(W+M)^2 / (W(W^2-1)) ], and the bracket is
   carried in parts per million so it can stay in integers. */
static int64_t meas_kick_sd(int64_t sigma, uint32_t w, uint32_t m)
{
  int64_t W = (int64_t)w, M = (int64_t)m, b;
  if (W < 3 || M < 1) return 0;
  b  = 2000000ll / W;
  b += (6000000ll * (W + M) * (W + M)) / (W * (W * W - 1ll));
  return (sigma * isqrt64(b)) / (1000ll * M);
}

/* One sigma on the RATE change, in ppb.  Two independent slopes, each with
   variance sigma^2*12/(W^3-W), so the difference carries sqrt(24/(W^3-W)) -
   and that falls only as W^(3/2), which is why a window long enough to
   measure the kick to a few percent still puts the rate change deep in the
   noise.  It is printed with this beside it rather than left to look like
   a measurement it is not. */
static int64_t meas_rate_sd_ppb(int64_t sigma, uint32_t w, int64_t nom)
{
  int64_t W = (int64_t)w, k;
  if (W < 3 || nom <= 0) return 0;
  k = isqrt64(24000000000000ll / (W * W * W - W));   /* sqrt(24/(W^3-W))*1e6 */
  return (sigma * k) / (nom / 1000ll);
}

/* --- PTIME sweep: a MEASURE at each of several pulse placements --------- */
#define PTS_MAX      10u
static bool     pts_on;
static bool     pts_retard;
static int32_t  pts_lo, pts_hi;
static uint32_t pts_steps, pts_i, pts_swings;
static uint32_t pts_settle;        /* swings still to wait before the next */
static int32_t  pts_us[PTS_MAX];
static int64_t  pts_ns[PTS_MAX];   /* kick, signed, ns per pulse           */
static int64_t  pts_sd[PTS_MAX];   /* and its one sigma                    */
static int64_t  pts_ppb[PTS_MAX];  /* what the pulsing did to the RATE     */
static int64_t  pts_ppbsd[PTS_MAX];
static int32_t  pts_restore;

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
   many corrective pulses fired - AUTH spends or KICK kicks, whichever
   mode is running - since the last line, once every phaselog_secs. */
static uint32_t phaselog_secs;      /* 0 = off                            */
static uint64_t phaselog_last_us;
static uint32_t disc_pulses;        /* corrective pulses fired by AUTH's
                                        spend or KICK's kick_step - not
                                        MEASURE/PTIMESCAN/manual PULSE     */
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
  integ = 0; cmd_ns = 0; credit_ns = 0;
  rate_have = false; rate_q = 0; rate_acc = 0; rate_n = 0;
  last_err = 0; filt_err = 0; drift_ppb = 0;
  kick_active = false; kick_since = 0;
  acq_run = 0; acq_prev_utc = 0;
  meas_fired = 0; meas_resid = 0; m_phase = MP_NONE;
  pts_on = false; pts_settle = 0;
  fit_reset(&m_f0); fit_reset(&m_f1);
  if (st == CTRL_TRACK || st == CTRL_MEASURE) st = CTRL_ACQUIRE;
}

void control_enable(bool on)
{
  cfg.control_enabled = on ? 1u : 0u;
  if (on) { if (st == CTRL_IDLE) { control_reset(); st = CTRL_ACQUIRE; } }
  else    { st = CTRL_IDLE; drive_all_off(); }
}

void control_set_offset_ns(int64_t o) { target_off = o; }

/* Called when AUTH changes.  A newly-set price makes the OLD credit mean
   something different than when it was accumulated - and if credit was
   sitting at zero because neither direction had a price yet, it is about
   to start meaning something for the first time.  Either way the honest
   thing is to start the spend fresh rather than dump whatever built up
   under the old (or absent) authority onto the actuator in one burst. */
void control_clear_credit(void)
{
  credit_ns = 0; integ = 0;
  kick_active = false; kick_since = 0;
}

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

/* Whatever forced this - lost NTP time, a schedule slip too large to
   trust, the detector going quiet, or an explicit control_blind() - the
   loop is leaving TRACK/ACQUIRE/MEASURE in a way nothing downstream
   expects, and anything mid-flight has to be torn down with it, not just
   the state variable.  A measurement across the gap is garbage, so
   m_phase is cleared; a PTIMESCAN sweep built out of one is worse than
   garbage, because nothing outside CTRL_MEASURE ever calls
   pts_start_point() again - left set, pts_on would sit true forever,
   silently waiting for a meas_finish() that can now never come.  That
   is exactly what used to happen: this cleanup lived only in
   control_blind(), so a schedule slip or a lost NTP fix mid-sweep
   orphaned it instead, and PTIMESCAN looked hung with no error, no
   abandonment message, nothing - while STATUS and ordinary tracking
   carried on as if nothing had happened.  Every site that forces
   CTRL_HOLD goes through here now so none of them can forget either
   half again. */
static void enter_hold(const char *why)
{
  control_state was = st;

  m_phase = MP_NONE;
  if (pts_on)
  {
    pts_on = false;
    if (pts_retard) cfg.pulse_retard_us  = pts_restore;
    else            cfg.pulse_advance_us = pts_restore;
    printf("PTIMESCAN sweep abandoned - lost lock; placement restored to "
           "%ld us\r\n", (long)pts_restore);
  }
  /* Only worth a line when it actually cost a lock, not on every explicit
     CONTROL N or the one hold before the first fix ever lands. */
  if (was == CTRL_TRACK || was == CTRL_MEASURE)
    printf("hold: %s\r\n", why);
  st = CTRL_HOLD;
  drive_all_off();             /* nothing queued should fire while blind */
}

control_state control_blind(void)
{
  control_state was = st;

  if (st == CTRL_TRACK || st == CTRL_ACQUIRE || st == CTRL_MEASURE)
    enter_hold("blind requested");
  return was;
}

/* Fit the before and after windows, extrapolate each to its edge of the
   pulsing interval, charge the mean of the two slopes against the interval,
   and what is left is the kick.

   utc_ns re-anchors the tracking NCO on the way out: it has been coasting
   through the measurement on purpose, and the pulses really did move the
   pendulum, so its phase is genuinely stale by the end.  The learned RATE
   is kept - it is still the best estimate we have, and it is what the
   coasting depended on. */
static void meas_finish(uint64_t utc_ns)
{
  int64_t W    = (int64_t)m_w;
  int64_t M    = (int64_t)m_n;
  int64_t s0   = fit_slope(&m_f0);                 /* ns per 1000 events   */
  int64_t s1   = fit_slope(&m_f1);
  int64_t pre  = fit_at(&m_f0, 2ll * W - 1ll);     /* just before pulse 1  */
  int64_t post = fit_at(&m_f1, -1ll);              /* just after pulse M   */
  int64_t ramp = ((s0 + s1) * M) / 2000ll;
  int64_t kick = post - pre - ramp;
  int64_t per  = meas_fired ? kick / (int64_t)meas_fired : 0;
  int64_t sig  = (fit_sigma(&m_f0) + fit_sigma(&m_f1)) / 2;
  int64_t sd   = meas_kick_sd(sig, m_w, meas_fired);
  int64_t nom  = (int64_t)event_ns_nominal();
  int64_t ppb  = nom ? ((s1 - s0) * 1000000ll) / nom : 0;
  int64_t mag  = (per < 0) ? -per : per;
  /* Positive phase error means the event came late, so a RETARD pulse
     should push the kick positive and an ADVANCE pulse negative.  This is
     the number that says whether the pulse did the job asked of it, and it
     is the only one worth ranking or reporting an AUTH from. */
  int64_t good = meas_retard ? per : -per;

  m_phase = MP_NONE;
  st      = CTRL_TRACK;
  rate_ns = utc_ns;
  rate_acc = 0;

  printf("%s authority over %lu pulses\r\n",
         meas_retard ? "retard" : "advance", (unsigned long)meas_fired);
  if (meas_fired < m_n)
    printf("   only %lu of %lu asked-for pulses actually fired - %lu were\r\n"
           "   refused (PW too wide for the duty budget, or this placement too\r\n"
           "   close to the event to still be in the future once it is\r\n"
           "   reported - see fire_for()); the count above and everything\r\n"
           "   below already reflect what really happened\r\n",
           (unsigned long)meas_fired, (unsigned long)m_n,
           (unsigned long)(m_n - meas_fired));
  printf("   event noise %lld us; windows of %lu swings either side\r\n",
         (long long)(sig / 1000), (unsigned long)m_w);
  printf("   rate before %lld, after %lld ns per 1000 swings"
         "  (moved %lld +/- %lld ppb)\r\n",
         (long long)s0, (long long)s1, (long long)ppb,
         (long long)meas_rate_sd_ppb(sig, m_w, nom));
  printf("   phase across pulsing %lld us, of which %lld us was rate\r\n",
         (long long)((post - pre) / 1000), (long long)(ramp / 1000));
  printf("   kick %lld us -> %lld +/- %lld ns per pulse\r\n",
         (long long)(kick / 1000), (long long)per, (long long)sd);

  if (pts_on)
  {
    pts_us[pts_i]  = pts_retard ? cfg.pulse_retard_us : cfg.pulse_advance_us;
    pts_ns[pts_i]  = per;
    pts_sd[pts_i]  = sd;
    pts_ppb[pts_i] = ppb;
    pts_ppbsd[pts_i] = meas_rate_sd_ppb(sig, m_w, nom);
    pts_i++;
    if (pts_i >= pts_steps) pts_finish();
    else pts_settle = 12u;          /* let the swing recover before the next */
    return;
  }

  if (good <= 0)
    printf("   WRONG SIGN: this placement %s the clock.  the pulse is on the\r\n"
           "   other side of the bob's turning point from where it needs to be\r\n",
           meas_retard ? "advanced" : "retarded");
  else if (good < 2 * sd)
    printf("   but that is inside the noise - run it again with more swings\r\n"
           "   before believing it\r\n");
  else
    printf("   AUTH %lld %lld   then SAVE to keep it\r\n",
           (long long)(meas_retard ? cfg.auth_advance_ns : mag),
           (long long)(meas_retard ? mag : cfg.auth_retard_ns));
}

static int32_t pts_place(uint32_t i)
{
  return (pts_steps < 2u) ? pts_lo
       : pts_lo + (int32_t)(((int64_t)(pts_hi - pts_lo) * (int64_t)i) / (int64_t)(pts_steps - 1u));
}

static void pts_start_point(void)
{
  int32_t us = pts_place(pts_i);
  if (pts_retard) cfg.pulse_retard_us  = us;
  else            cfg.pulse_advance_us = us;
  printf("  %lu of %lu: placement %ld us\r\n",
         (unsigned long)(pts_i + 1u), (unsigned long)pts_steps,
         (long)us);
  if (!control_measure_authority(pts_swings, pts_retard))
  {
    pts_on = false;
    if (pts_retard) cfg.pulse_retard_us  = pts_restore;
    else            cfg.pulse_advance_us = pts_restore;
    printf("sweep abandoned; placement restored to %ld us\r\n", (long)pts_restore);
  }
}

/* A sweep answers two questions, not one: which placement in this range is
   the best ADVANCE and which is the best RETARD.  Sign is physical, not a
   matter of which direction the sweep was launched under - see the
   config.h comment on pulse_advance_us/pulse_retard_us, where a given
   placement means the same instant either way - so negative per is always
   advance and positive is always retard, regardless of pts_retard.
   Ranking within each side separately, rather than by raw magnitude, is
   what keeps a sign-flipped outlier from looking like the winner of the
   side it is not even on. */
static void pts_finish(void)
{
  uint32_t i, best_adv = 0u, best_ret = 0u;
  int64_t  bestneg = 0, bestpos = 0, span = 0;
  bool     have_adv = false, have_ret = false;

  pts_on = false;
  if (pts_retard) cfg.pulse_retard_us  = pts_restore;
  else            cfg.pulse_advance_us = pts_restore;

  for (i = 0; i < pts_steps; i++)
  {
    int64_t g = pts_ns[i];             /* negative = advance, positive = retard */
    int64_t a = (g < 0) ? -g : g;
    if (g < 0 && (!have_adv || g < bestneg)) { bestneg = g; best_adv = i; have_adv = true; }
    if (g > 0 && (!have_ret || g > bestpos)) { bestpos = g; best_ret = i; have_ret = true; }
    if (a > span) span = a;
  }

  printf("\r\n%s placement sweep\r\n", pts_retard ? "retard" : "advance");
  printf("%10s %12s %9s %9s   %s\r\n",
         "us", "ns/pulse", "+/-", "rate ppb", "advance <-- | --> retard");
  for (i = 0; i < pts_steps; i++)
  {
    /* Left is always advance, right is always retard - the picture shows
       what the placement physically did, not whether it matched what was
       asked for. */
    int64_t  g = pts_ns[i];
    int64_t  a = (g < 0) ? -g : g;
    uint32_t k, len = (uint32_t)(span ? (a * 14) / span : 0);
    char     bar[32];
    const char *tag = (i == best_adv && have_adv) ? "  <-- best advance"
                     : (i == best_ret && have_ret) ? "  <-- best retard"
                                                    : "";

    memset(bar, ' ', sizeof(bar) - 1u); bar[sizeof(bar) - 1u] = '\0';
    bar[15] = '|';
    if (len > 14u) len = 14u;
    for (k = 1u; k <= len; k++) bar[15 + (g < 0 ? -(int)k : (int)k)] = '#';

    printf("%10ld %12lld %9lld %9lld   [%s]%s\r\n",
           (long)pts_us[i], (long long)pts_ns[i],
           (long long)pts_sd[i], (long long)pts_ppb[i], bar, tag);
  }

  printf("placement restored to %ld us.\r\n", (long)pts_restore);

  if (have_adv && -bestneg >= 2 * pts_sd[best_adv])
    printf("best advance: %ld us -> %lld +/- %lld ns/pulse\r\n",
           (long)pts_us[best_adv], (long long)bestneg, (long long)pts_sd[best_adv]);
  else if (have_adv)
    printf("best advance (%ld us, %lld ns/pulse) is inside its own error bar -\r\n"
           "repeat with more swings before trusting it\r\n",
           (long)pts_us[best_adv], (long long)bestneg);
  else
    printf("no advance authority (negative kick) anywhere in this range\r\n");

  if (have_ret && bestpos >= 2 * pts_sd[best_ret])
    printf("best retard:  %ld us -> %lld +/- %lld ns/pulse\r\n",
           (long)pts_us[best_ret], (long long)bestpos, (long long)pts_sd[best_ret]);
  else if (have_ret)
    printf("best retard  (%ld us, %lld ns/pulse) is inside its own error bar -\r\n"
           "repeat with more swings before trusting it\r\n",
           (long)pts_us[best_ret], (long long)bestpos);
  else
    printf("no retard authority (positive kick) anywhere in this range\r\n");

  {
    int32_t adv_place = have_adv ? pts_place(best_adv) : cfg.pulse_advance_us;
    int32_t ret_place = have_ret ? pts_place(best_ret) : cfg.pulse_retard_us;
    printf("'PTIME %ld %ld' then SAVE to keep %s\r\n", (long)adv_place, (long)ret_place,
           (have_adv && have_ret) ? "both"
           : have_adv ? "the advance placement (retard left as-is)"
           : have_ret ? "the retard placement (advance left as-is)"
                      : "the current placement - nothing significant found");
  }

  printf("rate ppb is what the pulsing did to the pendulum's RATE - the\r\n"
         "amplitude side of the trade, in quadrature with the kick.  its own\r\n"
         "error falls only as swings^1.5, so read it as a hint about which\r\n"
         "placement is gentler, not as a number to trust on its own\r\n");
}

bool control_ptime_scan(bool retard, int32_t lo, int32_t hi,
                        uint32_t steps, uint32_t swings)
{
  int32_t half;

  if (st != CTRL_TRACK) return false;
  /* Every point in the sweep is a MEASURE, so the same precondition holds;
     check it here rather than letting the banner print and the first point
     abandon the sweep. */
  if (!rate_have || rate_n < (cfg.rate_kp_events ? cfg.rate_kp_events : 350u))
  {
    printf("the rate tracker has not settled yet - %lu of %lu swings\r\n",
           (unsigned long)rate_n,
           (unsigned long)(cfg.rate_kp_events ? cfg.rate_kp_events : 350u));
    return false;
  }
  if (steps < 2u) steps = 5u;
  if (steps > PTS_MAX) steps = PTS_MAX;
  if (swings < 5u) swings = 40u;
  if (swings > 200u) swings = 200u;
  if (lo == 0 && hi == 0) { lo = 10000; hi = 60000; }

  /* Either offset can now reach a full half period from centre in either
     direction - see the config.h comment - so the bound here scales with
     THIS clock's period rather than the old fixed +-65535. */
  half = (int32_t)(cfg_period_ns() / 1000ull / 2ull);
  if (lo < -half) lo = -half;
  if (hi >  half) hi =  half;
  if (hi <= lo) return false;

  pts_on      = true;
  pts_retard  = retard;
  pts_lo      = lo;
  pts_hi      = hi;
  pts_steps   = steps;
  pts_swings  = swings;
  pts_i       = 0u;
  pts_restore = retard ? cfg.pulse_retard_us : cfg.pulse_advance_us;

  printf("sweeping %s placement %ld..%ld us in %lu steps, %lu swings each\r\n",
         retard ? "retard" : "advance", (long)lo, (long)hi,
         (unsigned long)steps, (unsigned long)swings);
  printf("this takes about %lu seconds; keep away from the clock\r\n",
         (unsigned long)((steps * (swings + 12u) * (cfg_period_ns() / 1000000ull)) / 1000ull));

  /* Point 0 used to arm the very instant this command ran, with no settle -
     every OTHER point gets 12 swings after its predecessor's real pulses
     to let the rate tracker and the demod baseline settle before its own
     PRE window starts (see meas_finish()), but point 0 had nothing to
     settle FROM on purpose, since there was no previous point.  That
     reasoning misses whatever was going on right up to the moment this
     command was typed - a live spend-mode correction, the tail of a
     RATEKP reset, a config change - which point 0's PRE window then
     measured as if it were signal.  Every later point got 12 clean swings
     of insulation from exactly this kind of thing; point 0 deserves the
     same, not zero.  Falling through the same settle path point 1..N-1 use
     keeps this one honest instead of a special case. */
  pts_settle = 12u;
  return true;
}

bool control_measure_authority(uint32_t n, bool retard)
{
  uint32_t kp = cfg.rate_kp_events ? cfg.rate_kp_events : 350u;

  if (st != CTRL_TRACK)
  {
    printf("the loop has to be tracking first; it is %s\r\n",
           control_state_name(st));
    return false;
  }
  if (n == 0u || n > 500u) { printf("1..500 pulses\r\n"); return false; }

  /* The measurement coasts on the tracking NCO's learned rate, so there has
     to BE one.  Measuring before the tracker has settled charges the
     pendulum's unknown rate against the pulses and calls the difference
     authority, which is how this came back with a different answer every
     time it was run. */
  if (!rate_have || rate_n < kp)
  {
    printf("the rate tracker has not settled yet - %lu of %lu swings.\r\n"
           "about %lu s to go; STATUS shows it as 'rate n'\r\n",
           (unsigned long)rate_n, (unsigned long)kp,
           (unsigned long)(((uint64_t)(kp - (rate_have ? rate_n : 0u))
                            * (cfg_period_ns() / 1000000ull)) / 1000ull));
    return false;
  }

  /* Equal thirds.  The error bar goes as (W+M)/(M*sqrt(W^3)) for windows of
     W and a pulsing run of M, and for a fixed total number of swings that
     is flattest at W = M - within a few percent of optimal anywhere near
     it, and a factor of two better than the half-length windows this used
     to run.  So a MEASURE of n pulses costs 3n swings and says so. */
  m_n     = n;
  m_w     = n;
  if (m_w < 8u)   m_w = 8u;
  if (m_w > 500u) m_w = 500u;
  m_phase = MP_PRE;
  m_left  = m_w;
  fit_reset(&m_f0);
  fit_reset(&m_f1);

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

  return drive_pulse_at(when, cfg.pulse_us);
}

/* KICK mode: a one-directional hysteresis scheme that needs no measured
   authority at all - the alternative to spending credit_ns against a
   MEASURE'd price.  It fires no more often than every kick_min_swings
   events, and only while the phase error sits on the side of zero this
   direction is meant to correct; once ev->demod_ns crosses back to the
   other side it goes idle and waits for the error to return before
   resuming, so a single direction cannot overshoot and then fight itself
   back the other way - it just stops and waits.

   ev->demod_ns follows the same sign convention as credit_ns above:
   positive means the hands are ahead (fast) and want retarding, negative
   means they are behind (slow) and want advancing. */
static void kick_step(const sense_event *ev)
{
  uint32_t need = cfg.kick_min_swings ? cfg.kick_min_swings : 5u;

  if (kick_since < 0xffffffffu) kick_since++;

  if (cfg.kick_retard)
  {
    /* retard mode: active while the hands are ahead of true time */
    if (kick_active) { if (ev->demod_ns <= 0) kick_active = false; }
    else              { if (ev->demod_ns >  0) kick_active = true;  }
    if (actuator_on && kick_active && kick_since >= need && fire_for(ev, true))
      { kick_since = 0; disc_pulses++; }
  }
  else
  {
    /* advance mode: active while the hands are behind true time */
    if (kick_active) { if (ev->demod_ns >= 0) kick_active = false; }
    else              { if (ev->demod_ns <  0) kick_active = true;  }
    if (actuator_on && kick_active && kick_since >= need && fire_for(ev, false))
      { kick_since = 0; disc_pulses++; }
  }
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
    rate_ns = ev->utc_ns; rate_acc = 0; rate_q = 0; rate_n = 0;
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
  rate_acc += rate_q;
  rate_ns  += (uint64_t)(nom + rate_acc / RATE_SCALE);
  rate_acc -= (rate_acc / RATE_SCALE) * RATE_SCALE;

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
    rate_ns += (uint64_t)(k * nom);
    rerr    -= k * nom;
    missed  += (uint32_t)k;
  }

  meas_resid = rerr;
  last_err   = rerr;
  filt_err  += (rerr - filt_err) / 8;
  events++;

  /* Through a measurement the NCO is the instrument, not the subject.
     Neither correction is applied, so it free-runs on the rate it had
     already learned and the pulses cannot pull it; meas_resid is then
     the phase the pulses put in, with the pendulum's own rate gone. */
  if (st != CTRL_MEASURE)
  {
    rate_ns = (uint64_t)((int64_t)rate_ns + rerr / kp); /* phase pull  */
    rate_q += (rerr * RATE_SCALE) / ki;                 /* rate learn  */
    if (rate_n < 0xffffffffu) rate_n++;
  }

  /* The learned offset, as parts per billion of the nominal interval. */
  drift_ppb = (rate_q * 1000000000ll) / (RATE_SCALE * nom);

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
        /* ev->demod_ns, not meas_resid: a delay-and-multiply phase reading
           off the sense coil's own continuously smoothed I/Q, independent
           of the threshold crossings meas_resid is built from.  Measured
           live against the same hardware, it carries about a ninth of the
           per-event noise - see the demod comment in sense.c.  fit_slope/
           fit_at still remove whatever constant rate this runs at relative
           to nominal, same as always; only the noise on the level goes
           down. */
        fit_add(&m_f0, ev->demod_ns);
        if (--m_left == 0u) { m_phase = MP_PULSE; m_left = m_n; }
        break;
      case MP_PULSE:
        /* Only a pulse that actually fired belongs in the count the kick
           gets divided by - see the comment on fire_for(). */
        if (fire_for(ev, meas_retard)) meas_fired++;
        if (--m_left == 0u) { m_phase = MP_POST; m_left = m_w; }
        break;
      case MP_POST:
        fit_add(&m_f1, ev->demod_ns);
        if (--m_left == 0u) meas_finish(ev->utc_ns);
        break;
      default:
        st = CTRL_TRACK;
        break;
    }
    return;
  }

  /* --- turn the phase error into pulses -------------------------------
     Two independent algorithms live here, selected by cfg.control_mode.
     cmd_ns is a live phase-error readout for STATUS either way; it is not
     itself a controller output any more (see the KICK mode comment). */
  cmd_ns = ev->demod_ns;

  if (cfg.control_mode == 1u)
  {
    /* KICK: see kick_step() above.  credit_ns is the AUTH-mode
       accumulator and has no meaning here; hold it at zero so STATUS
       does not show a stale or misleading number. */
    credit_ns = 0;
    if (cfg.control_enabled) kick_step(ev);
    return;
  }

  /* --- AUTH: spend the phase error in whole pulses ---------------------
     credit_ns used to accumulate a PI-plus-feedforward loop's OUTPUT
     (a gained, rate-compensated command built from err/rate_q).  It now
     accumulates the phase error ITSELF, straight from ev->demod_ns - the
     same low-noise, UTC-referenced reading MEASURE uses - and the
     threshold-spend logic below is the only "controller" left: pure
     accumulate-and-correct, no proportional term, no separate rate
     estimate.  kp_swings/ki_swings/slew_limit_ppm are no longer read
     anywhere; they are harmless to leave set, just without effect. */
  credit_ns += ev->demod_ns;

  /* Positive credit means the hands are ahead and want retarding.  Each
     direction is spent at its own measured price; a direction that has
     not been measured simply cannot be spent, which is what lets a
     retard-only installation still discipline a clock that gains. */
  {
    int64_t ar = (int64_t)cfg.auth_retard_ns;
    int64_t aa = (int64_t)cfg.auth_advance_ns;

    if (cfg.control_enabled && actuator_on)
    {
      /* Only spend the credit if the pulse actually fired - a refusal here
         used to still deduct the price, so the loop believed it had paid
         for a correction the pendulum never received and quietly fell
         behind by however much that pulse was worth. */
      if      (ar > 0 && credit_ns >= ar  && fire_for(ev, true))  { credit_ns -= ar; disc_pulses++; }
      else if (aa > 0 && credit_ns <= -aa && fire_for(ev, false)) { credit_ns += aa; disc_pulses++; }
    }

    /* This clamp has to run whether or not control is enabled, and whether
       or not a price is known yet, because credit_ns accumulates
       unconditionally above.

       With NEITHER direction measured, nothing can ever be spent - hold
       the credit at zero rather than letting it run up an unpayable
       backlog that gets dumped on the actuator in one burst the moment
       AUTH finally gives it a price.

       With one direction measured, borrow its price to bound the other -
       otherwise the retard-only case, which is a supported configuration
       and the one a clock that gains actually needs, accumulates
       unbounded advance demand it can never spend, and then has to work
       off a phantom backlog before it fires again when the error finally
       reverses. */
    if (ar <= 0 && aa <= 0) { credit_ns = 0; }
    else
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
    /* Anchor the rate NCO on this event, so the loop starts at zero error
       and only has to hold it there - rate_ns is what the slip check
       measures against now, so this is the reset that matters; without
       it the first event after a reacquire would compare against
       whatever rate_ns was frozen at when tracking last stopped, which
       free-runs every event track_event() is not called, not just the
       ones spent in CTRL_HOLD, and would report a huge, spurious slip on
       the very next event. */
    rate_ns   = ev->utc_ns;
    rate_have = true;
    rate_acc  = 0;
    rate_q    = 0;
    rate_n    = 0;
    events    = 0;
    missed    = 0;
    integ     = 0;
    cmd_ns    = 0;
    credit_ns = 0;
    last_err  = 0;
    filt_err  = 0;
    drift_ppb = 0;
    kick_active = false;
    kick_since  = 0;
    st = CTRL_TRACK;
  }
}

void control_poll(void)
{
  sense_event ev;

  while (sense_next_event(&ev))
  {
    /* Never during CTRL_MEASURE: fire_for() below has to schedule against
       this same event before its wrap-forward margin (PULSE_LEAD_US) runs
       out, and a USB-CDC printf() can block for tens of milliseconds if
       the host is slow to drain it - long enough to eat that margin
       outright for a placement that had little of it to begin with. */
    if (ev_echo && st != CTRL_MEASURE)
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
  if ((st == CTRL_TRACK || st == CTRL_MEASURE) && sense_last_event_us() != 0ull)
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
      printf("phaselog: %s  err %lld us  filt %lld us  %lu pulses in the"
             " last %lu s (%lu total)\r\n",
             control_state_name(st), (long long)(last_err / 1000),
             (long long)(filt_err / 1000), (unsigned long)fired,
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
  o->credit_ns        = credit_ns;
  o->drift_ppb        = drift_ppb;
  o->ff_ns            = ff_last;
  o->rate_n           = rate_n;
  o->rate_ready       = (uint8_t)((rate_have && rate_n >
      4u * (cfg.rate_kp_events ? cfg.rate_kp_events : 350u)) ? 1u : 0u);
  o->pulses           = drive_pulse_count();
  o->missed           = missed;
  o->target_offset_ns = target_off;
  o->kick_active       = kick_active ? 1u : 0u;
  o->kick_since        = kick_since;
}
