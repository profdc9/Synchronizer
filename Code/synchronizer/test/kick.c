/* kick.c - host test: does KICK trip in the right direction, keep pulsing
   until corrected, and release?

   Synthetic swings are pushed through the REAL control loop - ../src/control.c
   is compiled unmodified against the stubs beside this file - so what is
   tested is the code that runs on the board.

   AUTH/MEASURE/PTIMESCAN are gone (see git history if that measured-
   authority path is ever wanted back); KICK's bang-bang hysteresis against
   sched_err_ns is now the only correction algorithm, so this is what the
   host test exercises.  It also checks that a genuine schedule slip still
   drops the loop into hold and that it re-acquires afterward, since that
   path is shared with the removed MEASURE/PTIMESCAN state machine and was
   touched while pulling them out. */

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
#include <math.h>
#include "pico/stdlib.h"
#include "config.h"
#include "sense.h"
#include "control.h"

extern uint64_t stub_now_us;
extern uint32_t stub_pulses;
void stub_push(const sense_event *e);

static unsigned long long seed = 12345;
static double gr(void)
{ double s = 0; int i;
  for (i = 0; i < 12; i++) { seed = seed*6364136223846793005ull+1442695040888963407ull;
    s += (double)((seed >> 33) & 0xffffff) / 16777216.0; }
  return s - 6.0; }

/* 2.27 ms of per-swing timing noise, same scale as the development clock -
   the regime that made a slow feedback signal overshoot before it noticed
   its own corrections working. */
#define NOISE_NS   2270000.0

static uint64_t t;
static double   phase;   /* actual - nominal, ns, what sched_err_ns tracks */

static void push_clean(uint64_t nom)
{
  sense_event e; memset(&e, 0, sizeof(e));
  t += nom;
  stub_now_us = t / 1000ull;
  e.t_us = stub_now_us; e.utc_ns = t + (uint64_t)(int64_t)(NOISE_NS * gr());
  stub_push(&e); control_poll();
}

/* One event of a pendulum running persistently off-rate by drift_ns_per_ev,
   with a kick_ns correction folded in on any pulse this event's drive
   fires - simulating the physical effect a real pulse has on the bob. */
static bool push_drifting(uint64_t nom, double drift_ns_per_ev, double kick_ns)
{
  sense_event e; memset(&e, 0, sizeof(e));
  uint32_t before = stub_pulses;
  bool fired;

  t += nom;
  phase += drift_ns_per_ev;
  stub_now_us = t / 1000ull;
  e.t_us = stub_now_us;
  e.utc_ns = t + (uint64_t)(int64_t)(phase + NOISE_NS * gr());
  stub_push(&e); control_poll();

  fired = stub_pulses != before;
  if (fired) phase += kick_ns;
  return fired;
}

static void base_config(void)
{
  memset(&cfg, 0, sizeof(cfg));
  cfg.beats_per_hour = 8400; cfg.beats_per_period = 2; cfg.events_per_period = 1;
  cfg.drive_offset_ppt = 500; cfg.rate_kp_events = 120; cfg.acquire_events = 12;
  cfg.acquire_tol_pct = 4; cfg.kp_swings = 4200; cfg.ki_swings = 12600;
  cfg.slew_limit_ppm = 500; cfg.pulse_us = 2000;
  cfg.pulse_advance_us = 40000; cfg.pulse_retard_us = 40000;
  cfg.kick_min_swings = 5; cfg.kick_threshold_pct = 5;
  cfg.control_enabled = 1;
}

/* Run a clock that drifts persistently at drift_ns_per_ev, apply kick_ns of
   correction on every pulse KICK fires, and check it trips the direction
   named by want_retard and releases once the drift is corrected. */
static int run_case(const char *name, double drift_ns_per_ev, double kick_ns,
                    bool want_retard)
{
  uint64_t nom;
  control_stats cs;
  int i;
  bool tripped = false, released = false;

  printf("\n== %s ==\n", name);
  base_config();
  nom = cfg_event_interval_ns();
  t = 0; phase = 0;
  seed = 12345;

  control_init();
  control_enable(true);

  for (i = 0; i < 700; i++) push_clean(nom);
  control_stats_get(&cs);
  printf("settled: state %s, uncorrected error %lld us\n",
         control_state_name(cs.state), (long long)(cs.sched_err_ns / 1000));
  if (cs.state != CTRL_TRACK)
  { printf("FAIL: never reached track\n"); return 1; }

  for (i = 0; i < 25000 && !released; i++)
  {
    push_drifting(nom, drift_ns_per_ev, kick_ns);
    control_stats_get(&cs);
    if (!tripped && cs.kick_active)
    {
      tripped = true;
      printf("tripped after %d events: %s, uncorrected error %lld us\n",
             i, cs.kick_dir_retard ? "retarding" : "advancing",
             (long long)(cs.sched_err_ns / 1000));
      if (cs.kick_dir_retard != want_retard)
      { printf("FAIL: wrong direction\n"); return 1; }
    }
    if (tripped && !cs.kick_active) released = true;
  }

  if (!tripped) { printf("FAIL: never tripped\n"); return 1; }
  if (!released) { printf("FAIL: never released\n"); return 1; }
  control_stats_get(&cs);
  printf("released: uncorrected error %lld us, %lu pulses fired\n",
         (long long)(cs.sched_err_ns / 1000), (unsigned long)stub_pulses);
  return 0;
}

/* A schedule slip - one event landing far from where the tracker expected -
   has to drop the loop into hold, and ordinary clean events afterward have
   to bring it back to track.  This path is shared with the now-removed
   MEASURE/PTIMESCAN state machine and was touched while pulling them out. */
static int run_slip_test(void)
{
  uint64_t nom;
  control_stats cs;
  int i;

  printf("\n== schedule slip drops to hold, then re-acquires ==\n");
  base_config();
  nom = cfg_event_interval_ns();
  t = 0; phase = 0;
  seed = 55;

  control_init();
  control_enable(true);
  for (i = 0; i < 700; i++) push_clean(nom);
  control_stats_get(&cs);
  if (cs.state != CTRL_TRACK) { printf("FAIL: never reached track\n"); return 1; }

  /* Jump the schedule by half the period plus one nominal interval - well
     past MAX_SLIP_EVENTS' tolerance in the "early" direction, which holds
     immediately. */
  {
    sense_event e; memset(&e, 0, sizeof(e));
    t += nom;
    stub_now_us = t / 1000ull;
    e.t_us = stub_now_us;
    e.utc_ns = t - nom / 2ull - nom;
    stub_push(&e); control_poll();
  }
  control_stats_get(&cs);
  printf("after slip: state %s\n", control_state_name(cs.state));
  if (cs.state != CTRL_HOLD) { printf("FAIL: slip did not hold\n"); return 1; }

  for (i = 0; i < 700; i++) push_clean(nom);
  control_stats_get(&cs);
  printf("after re-settling: state %s\n", control_state_name(cs.state));
  if (cs.state != CTRL_TRACK) { printf("FAIL: never re-acquired\n"); return 1; }
  return 0;
}

int main(void)
{
  int rc = 0;

  /* Running fast (bob arrives earlier than nominal every event) must trip
     a RETARD - sched_err_ns negative - and release once retard pulses
     (kick_ns positive: retarding delays the next arrival) pull it back
     past zero.  drift_ns_per_ev*kick_min_swings has to be smaller in
     magnitude than kick_ns or a correcting pulse every kick_min_swings
     events could never win against the drift between pulses. */
  rc |= run_case("persistently fast: KICK must retard", -3000.0, 90000.0, true);

  /* Running slow must trip an ADVANCE - sched_err_ns positive - and release
     once advance pulses (kick_ns negative) pull it back past zero. */
  rc |= run_case("persistently slow: KICK must advance", 3000.0, -90000.0, false);

  rc |= run_slip_test();

  printf("\n%s\n", rc ? "FAILED" : "ok");
  return rc;
}
