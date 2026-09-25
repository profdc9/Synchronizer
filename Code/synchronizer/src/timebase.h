/* timebase.h - a UTC timebase disciplined against NTP */

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

#ifndef _TIMEBASE_H
#define _TIMEBASE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The RP2040 runs from a crystal that is good to perhaps 30 ppm, which is
   2.6 s/day - a quarter of the error we are trying to take out of the
   clock.  So the reference cannot be the raw timer: it has to be a model
   of UTC that NTP corrects in both phase and rate.

       utc_ns = base_utc_ns + elapsed_us * 1000 * (1 + ppb/1e9)

   Between fixes the learned rate carries the timebase, so a lost network
   does not immediately poison the discipline loop. */

void     tb_init(int32_t seed_ppb);

/* Disciplined UTC, nanoseconds since the Unix epoch.  Zero until the first
   fix lands; tb_have_time() says whether it means anything yet. */
uint64_t tb_utc_ns(void);
bool     tb_have_time(void);

/* Feed one NTP result: the local timer reading when the exchange happened,
   and the server's UTC for that instant.  Returns true if it was used. */
bool     tb_apply_fix(uint64_t local_us, uint64_t server_utc_ns, uint32_t rtt_us);

int32_t  tb_ppb(void);            /* current rate correction              */
int64_t  tb_last_offset_ns(void); /* offset of the last accepted fix      */
uint32_t tb_fix_count(void);
uint64_t tb_last_fix_us(void);    /* local timer reading of the last fix  */
/* Whether the last accepted fix was large enough to step the phase outright
   rather than slew it in proportionally - the branch with no gain at all,
   so a bad fix lands in the model in one shot. */
bool     tb_last_was_step(void);

/* The web page's NTP-correction graph reads through these - same cursor
   idiom as control.c's errhist/control_errhist_read().  tb_hist_seq() is
   the current cursor; tb_hist_read() fills offset_us_out[]/stepped_out[]
   with up to n samples from `from` onward (one per fix actually applied,
   oldest first) and reports the cursor to pass next time. */
uint32_t tb_hist_seq(void);
uint32_t tb_hist_read(uint32_t from, int32_t *offset_us_out, int8_t *stepped_out,
                      uint32_t n, uint32_t *next);

#ifdef __cplusplus
}
#endif

#endif /* _TIMEBASE_H */
