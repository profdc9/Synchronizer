/* timebase.c */

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

#include <stdlib.h>
#include "pico/stdlib.h"
#include "config.h"
#include "timebase.h"

/* Phase offsets larger than this are stepped rather than slewed - there is
   no point walking a cold start in gently. */
#define STEP_THRESHOLD_NS   (500ll * 1000 * 1000)

/* Fixes with a worse round trip than this are thrown away; the offset is
   only known to about half the round trip. */
#define MAX_RTT_US          400000u

/* A span shorter than this makes the rate term's normalization blow up
   disproportionately for two fixes landing close together - skip the rate
   update in that case; the phase pull below still applies regardless. */
#define MIN_RATE_SPAN_US    1000000u         /* 1 s */

#define PPB_LIMIT           150000          /* +/- 150 ppm, well past any xtal */

static uint64_t base_utc_ns;
static uint64_t base_us;
static int32_t  ppb;
static bool     have_time;
static uint32_t fixes;
static int64_t  last_offset;
static uint64_t last_fix_us;

void tb_init(int32_t seed_ppb)
{
  base_utc_ns = 0;
  base_us     = time_us_64();
  ppb         = seed_ppb;
  have_time   = false;
  fixes       = 0;
  last_offset = 0;
  last_fix_us = 0;
}

/* Project the model forward from its base to an arbitrary timer reading. */
static uint64_t project(uint64_t at_us)
{
  uint64_t d_us = at_us - base_us;
  int64_t  corr;

  /* d_us/1000 keeps the product inside 64 bits for any sane interval:
     1e6 ms * 150000 ppb / 1000 is about 1.5e11. */
  corr = ((int64_t)(d_us / 1000u) * (int64_t)ppb) / 1000;
  return base_utc_ns + d_us * 1000ull + (uint64_t)corr;
}

uint64_t tb_utc_ns(void)
{
  if (!have_time) return 0;
  return project(time_us_64());
}

bool tb_have_time(void) { return have_time; }
int32_t tb_ppb(void) { return ppb; }
int64_t tb_last_offset_ns(void) { return last_offset; }
uint32_t tb_fix_count(void) { return fixes; }
uint64_t tb_last_fix_us(void) { return last_fix_us; }

/* A type-2 loop, the same shape as control.c's pendulum tracking NCO: one
   error signal (offset - actual UTC from the server minus what the model
   currently predicts for that instant), split into a proportional pull on
   phase and an integral pull on rate.  kp is a time constant in FIXES
   (cfg.tb_kp_fixes) rather than seconds, because fixes do not land on a
   uniform clock; ki follows it the same critical-damping formula
   (2*kp*kp).  Each fix's offset is noisy from network jitter, but the
   crystal it measures only drifts with temperature, so a heavily damped
   integral costs nothing real and buys a lot of noise rejection. */
bool tb_apply_fix(uint64_t local_us, uint64_t server_utc_ns, uint32_t rtt_us)
{
  int64_t  offset;
  uint64_t span_us;
  int64_t  kp;

  if (rtt_us > MAX_RTT_US) return false;

  if (!have_time)
  {
    base_us     = local_us;
    base_utc_ns = server_utc_ns;
    have_time   = true;
    fixes       = 1;
    last_offset = 0;
    last_fix_us = local_us;
    return true;
  }

  span_us = local_us - last_fix_us;
  offset  = (int64_t)server_utc_ns - (int64_t)project(local_us);
  last_offset = offset;
  fixes++;
  last_fix_us = local_us;

  if (offset > STEP_THRESHOLD_NS || offset < -STEP_THRESHOLD_NS)
  {
    /* Something jumped - a reboot of the server, a bad sample that slipped
       the RTT gate, or our first fix after a long outage.  Step rather
       than let a single huge outlier corrupt the rate loop. */
    base_us     = local_us;
    base_utc_ns = server_utc_ns;
    return true;
  }

  kp = (int64_t)(cfg.tb_kp_fixes ? cfg.tb_kp_fixes : 20u);

  /* Integral: fold a heavily damped fraction of the implied rate error
     into ppb.  Skip it for an implausibly short span - two fixes landing
     close together would otherwise blow the normalization up rather than
     just being noisy like a normal one. */
  if (span_us >= MIN_RATE_SPAN_US)
  {
    int64_t ki      = 2ll * kp * kp;
    int64_t adj_ppb = (offset * 1000000ll) / ((int64_t)span_us * ki);
    ppb += (int32_t)adj_ppb;
    if (ppb >  PPB_LIMIT) ppb =  PPB_LIMIT;
    if (ppb < -PPB_LIMIT) ppb = -PPB_LIMIT;
  }

  /* Proportional: pull the phase toward the server's offset, re-basing
     onto the corrected model so the timebase stays continuous and
     monotonic rather than jumping. */
  base_utc_ns = project(local_us) + (uint64_t)(offset / kp);
  base_us     = local_us;

  return true;
}
