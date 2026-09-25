/* chime.c */

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
#include <stdlib.h>
#include "pico/stdlib.h"
#include "config.h"
#include "timebase.h"
#include "chime.h"

#define DAY_S   86400
#define HALF_S  43200

void chime_init(void) { }

uint32_t chime_local_sod(void)
{
  int64_t s;
  if (!tb_have_time()) return CHIME_SOD_UNKNOWN;
  s = (int64_t)(tb_utc_ns() / 1000000000ull) + (int64_t)cfg.tz_offset_s;
  s %= DAY_S;
  if (s < 0) s += DAY_S;
  return (uint32_t)s;
}

/* Fold a difference in seconds into the half day either side of zero, so a
   clock that is a minute slow does not read as one that is 23h59m fast. */
static int32_t wrap_half_day(int32_t d)
{
  while (d >  HALF_S) d -= DAY_S;
  while (d < -HALF_S) d += DAY_S;
  return d;
}

bool chime_mark(uint32_t face_hour, uint32_t face_min)
{
  uint32_t sod = chime_local_sod();
  int32_t  best = 0;
  bool     have_best = false;
  uint32_t i;

  if (sod == CHIME_SOD_UNKNOWN) return false;
  if (face_hour > 23u || face_min > 59u) return false;

  /* A dial marked 1 to 12 cannot say which half of the day it means, so try
     both and keep whichever puts the clock nearer the truth.  An hour given
     as 13..23 is already unambiguous and the second candidate simply loses. */
  for (i = 0; i < 2u; i++)
  {
    uint32_t h = (face_hour + 12u * i) % 24u;
    int32_t  d = wrap_half_day((int32_t)(h * 3600u + face_min * 60u) - (int32_t)sod);
    if (!have_best || abs(d) < abs(best)) { best = d; have_best = true; }
  }

  /* The press trails the sound; add the configured allowance back on. */
  cfg.chime_offset_ms = best * 1000 + (int32_t)cfg.chime_latency_ms;
  cfg.chime_ref_utc   = tb_utc_ns() / 1000000000ull;
  cfg.chime_valid     = 1u;
  return true;
}

bool     chime_have(void)           { return cfg.chime_valid != 0u; }
int32_t  chime_offset_ms(void)      { return cfg.chime_offset_ms; }
uint64_t chime_marked_utc_ns(void)  { return cfg.chime_ref_utc * 1000000000ull; }
void     chime_forget(void)         { cfg.chime_valid = 0u; cfg.chime_offset_ms = 0; }

/* What the hands should read RIGHT NOW, given the last chime mark and how
   far off it found them - the same clock_sod chime_next() computes for
   itself to find the next strike, exposed on its own so a caller can show
   "the dial should currently say X" without needing the next-chime
   machinery too. */
bool chime_face_now_sod(uint32_t *face_sod)
{
  uint32_t sod = chime_local_sod();
  int32_t  clock_sod;

  if (!cfg.chime_valid || sod == CHIME_SOD_UNKNOWN) return false;

  clock_sod = ((int32_t)sod + cfg.chime_offset_ms / 1000) % DAY_S;
  if (clock_sod < 0) clock_sod += DAY_S;

  if (face_sod) *face_sod = (uint32_t)clock_sod;
  return true;
}

bool chime_next(uint32_t *seconds_away, uint32_t *face_sod, uint32_t *true_sod)
{
  uint32_t sod = chime_local_sod();
  uint32_t clock_sod_u;
  int32_t  interval, clock_sod, away, rel;

  if (!chime_face_now_sod(&clock_sod_u)) return false;
  clock_sod = (int32_t)clock_sod_u;

  interval = (int32_t)(cfg.chime_interval_min ? cfg.chime_interval_min : 60u) * 60;
  if (interval <= 0) return false;

  /* The movement does not necessarily strike exactly on the hour - a real
     striking train can release a few seconds to either side of it, a fixed
     mechanical property of this specific clock rather than anything the
     discipline loop touches.  chime_strike_offset_s is that quirk: it
     shifts which instant counts as "on the hour" for this purpose, so
     rel is 0 exactly when the hands are at a strike point rather than at
     the interval boundary itself. */
  rel = (clock_sod - cfg.chime_strike_offset_s) % interval;
  if (rel < 0) rel += interval;
  away = interval - rel;
  if (away <= 0) away += interval;

  if (seconds_away) *seconds_away = (uint32_t)away;
  if (face_sod)     *face_sod     = (uint32_t)((clock_sod + away) % DAY_S);
  if (true_sod)     *true_sod     = (uint32_t)(((int32_t)sod + away) % DAY_S);
  return true;
}
