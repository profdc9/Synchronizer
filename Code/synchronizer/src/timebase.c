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
#include "timebase.h"

/* Phase offsets larger than this are stepped rather than slewed - there is
   no point walking a cold start in gently. */
#define STEP_THRESHOLD_NS   (500ll * 1000 * 1000)

/* Fixes with a worse round trip than this are thrown away; the offset is
   only known to about half the round trip. */
#define MAX_RTT_US          400000u

/* Rate estimation needs a decent baseline or the quotient is all noise. */
#define MIN_RATE_INTERVAL_US 120000000ull   /* two minutes */

#define PPB_LIMIT           150000          /* +/- 150 ppm, well past any xtal */

static uint64_t base_utc_ns;
static uint64_t base_us;
static int32_t  ppb;
static bool     have_time;
static uint32_t fixes;
static int64_t  last_offset;
static uint64_t last_fix_us;
static uint64_t rate_ref_us;      /* anchor for the next rate estimate */
static uint64_t rate_ref_utc_ns;

void tb_init(int32_t seed_ppb)
{
  base_utc_ns = 0;
  base_us     = time_us_64();
  ppb         = seed_ppb;
  have_time   = false;
  fixes       = 0;
  last_offset = 0;
  last_fix_us = 0;
  rate_ref_us = 0;
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

bool tb_apply_fix(uint64_t local_us, uint64_t server_utc_ns, uint32_t rtt_us)
{
  int64_t offset;

  if (rtt_us > MAX_RTT_US) return false;

  if (!have_time)
  {
    base_us     = local_us;
    base_utc_ns = server_utc_ns;
    have_time   = true;
    fixes       = 1;
    last_offset = 0;
    last_fix_us = local_us;
    rate_ref_us     = local_us;
    rate_ref_utc_ns = server_utc_ns;
    return true;
  }

  offset = (int64_t)server_utc_ns - (int64_t)project(local_us);
  last_offset = offset;
  fixes++;
  last_fix_us = local_us;

  if (offset > STEP_THRESHOLD_NS || offset < -STEP_THRESHOLD_NS)
  {
    /* Something jumped - a reboot of the server, a bad sample that slipped
       the RTT gate, or our first fix after a long outage.  Step, and start
       the rate estimate over rather than folding the jump into it. */
    base_us         = local_us;
    base_utc_ns     = server_utc_ns;
    rate_ref_us     = local_us;
    rate_ref_utc_ns = server_utc_ns;
    return true;
  }

  /* Rate: over the interval since the last anchor, how much did our model
     drift against the server?  That drift divided by the interval is the
     residual frequency error, which we fold into ppb. */
  if (rate_ref_us != 0 && (local_us - rate_ref_us) >= MIN_RATE_INTERVAL_US)
  {
    uint64_t span_us   = local_us - rate_ref_us;
    int64_t  model_ns  = (int64_t)(project(local_us) - rate_ref_utc_ns);
    int64_t  true_ns   = (int64_t)(server_utc_ns - rate_ref_utc_ns);
    int64_t  err_ns    = true_ns - model_ns;      /* we were slow if > 0 */
    int64_t  adj_ppb   = (err_ns * 1000) / (int64_t)(span_us);

    /* Fold in a fraction of the estimate; a single NTP pair is noisy. */
    ppb += (int32_t)(adj_ppb / 2);
    if (ppb >  PPB_LIMIT) ppb =  PPB_LIMIT;
    if (ppb < -PPB_LIMIT) ppb = -PPB_LIMIT;

    rate_ref_us     = local_us;
    rate_ref_utc_ns = server_utc_ns;
  }

  /* Re-base onto the corrected model, absorbing the residual phase offset
     so the timebase is continuous and monotonic rather than jumping. */
  base_utc_ns = project(local_us) + (uint64_t)(offset / 4);   /* slew 25% */
  base_us     = local_us;

  return true;
}
