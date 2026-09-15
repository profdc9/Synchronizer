/* authority.c - host test: does MEASURE recover what was put in?

   Synthetic swings are pushed through the REAL control loop - ../src/control.c
   is compiled unmodified against the stubs beside this file - so what is
   tested is the code that runs on the board.

   Usage: authority [kick_ns [rate_ns_per_event [noise_ns [pulses [seed]]]]]
          SWEEP=1 authority ...        run PTIMESCAN instead
          SWEEP=1 WRONGSIDE=1 ...      with every placement on the wrong side
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

unsigned long long seed = 12345;
static double gr(void)
{ double s = 0; int i;
  for (i = 0; i < 12; i++) { seed = seed*6364136223846793005ull+1442695040888963407ull;
    s += (double)((seed >> 33) & 0xffffff) / 16777216.0; }
  return s - 6.0; }

int main(int argc, char **argv)
{
  double kick   = argc > 1 ? atof(argv[1]) : 89100.0;  /* ns per pulse   */
  double ramp   = argc > 2 ? atof(argv[2]) : 400.0;    /* ns/event after */
  double noise  = argc > 3 ? atof(argv[3]) : 2270000.0;
  int    pulses = argc > 4 ? atoi(argv[4]) : 60;
  if (argc > 5) seed = strtoull(argv[5], 0, 10);
  uint64_t nom;
  uint64_t t = 0;
  double  phase = 0, rate = 0;
  uint32_t before, n;
  int i;

  memset(&cfg, 0, sizeof(cfg));
  cfg.beats_per_hour = 8400; cfg.beats_per_period = 2; cfg.events_per_period = 1;
  cfg.drive_offset_ppt = 500; cfg.rate_kp_events = 120; cfg.acquire_events = 12;
  cfg.acquire_tol_pct = 4; cfg.kp_swings = 4200; cfg.ki_swings = 12600;
  cfg.slew_limit_ppm = 500; cfg.pulse_us = 2000;
  cfg.pulse_advance_us = 40000; cfg.pulse_retard_us = 40000;
  cfg.control_enabled = 0;
  nom = cfg_event_interval_ns();

  control_init();
  control_enable(true);

  /* Feed events until the tracker has settled well past kp. */
  for (i = 0; i < 700; i++)
  {
    sense_event e; memset(&e, 0, sizeof(e));
    t += nom;
    stub_now_us = t / 1000ull;
    e.t_us = stub_now_us; e.utc_ns = t + (uint64_t)(int64_t)(noise * gr());
    stub_push(&e); control_poll();
  }
  before = stub_pulses;

  if (getenv("SWEEP"))
  {
    /* A placement curve whose LARGEST magnitude is on the wrong side of
       zero - exactly the shape that made magnitude ranking pick a
       sign-flipped outlier as "best".  Peak of the useful lobe is at
       30000 us, +60000 ns; at 60000 us it is -140000 ns. */
    int wrong = getenv("WRONGSIDE") != 0;
    if (!control_ptime_scan(true, 10000, 60000, 5, 40))
    { printf("sweep refused\n"); return 1; }
    for (i = 0; i < 4000; i++)
    {
      sense_event e; uint32_t fb = stub_pulses; double d;
      memset(&e, 0, sizeof(e));
      t += nom; stub_now_us = t / 1000ull; e.t_us = stub_now_us;
      e.utc_ns = t + (uint64_t)(int64_t)(phase + noise * gr());
      stub_push(&e); control_poll();
      if (stub_pulses != fb)
      { d = ((double)cfg.pulse_retard_us - 30000.0) / 30000.0;
        /* Default: a useful lobe peaking at 30000 us, and a far LARGER
           excursion of the wrong sign at the end of the range - the shape
           that made magnitude ranking pick the outlier.  WRONGSIDE: the
           whole range on the wrong side of the turning point. */
        phase += wrong ? -(200000.0 - 100000.0 * d * d)
                       :  (60000.0  - 200000.0 * d * d); }
    }
    return 0;
  }

  if (!control_measure_authority((uint32_t)pulses, true))
  { printf("measure refused\n"); return 1; }

  /* 3*pulses events: the middle third really does get kicked. */
  n = 3u * (uint32_t)pulses;
  for (i = 0; i < (int)n; i++)
  {
    sense_event e; memset(&e, 0, sizeof(e));
    uint32_t fired_before = stub_pulses;
    t += nom;
    /* the pulse fired on the PREVIOUS event lands in this interval */
    stub_now_us = t / 1000ull;
    e.t_us = stub_now_us;
    e.utc_ns = t + (uint64_t)(int64_t)(phase + noise * gr());
    stub_push(&e); control_poll();
    if (stub_pulses != fired_before) { phase += kick; rate = ramp; }
    phase += rate;
  }
  printf("(%u pulses fired, truth %g ns each, rate step %g ns/event)\n",
         stub_pulses - before, kick, ramp);
  return 0;
}
