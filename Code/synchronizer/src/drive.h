/* drive.h - impulse coil pulse generation, with hard safety limits */

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

#ifndef _DRIVE_H
#define _DRIVE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO_PULSE high turns Q4 on, which pulls the IRF9540N gate to ground and
   energises the drive coil from +12 V through R6 (10R).

   R6 is the reason everything here is bounded.  The coil's DC resistance is
   not known until it is measured; if it came out low, R6 would be asked to
   dissipate several watts, and a pulse left on by a crashed or confused
   controller would destroy it.  So:

     - the pin idles low and is driven low again by an alarm, not by code
       returning from a loop;
     - a watchdog timer independently forces it low if it has somehow been
       high longer than DRIVE_MAX_PULSE_US;
     - a token bucket caps the long-run average duty cycle, so no sequence
       of legal pulses can add up to an illegal one.

   None of these should ever fire in normal operation.  Correcting 11 s/day
   needs about 113 us of retard per swing, delivered as one short pulse every
   few dozen swings. */

#define DRIVE_MAX_PULSE_US     50000u    /* absolute ceiling on one pulse  */
#define DRIVE_DUTY_PPM         20000u    /* 2% long-run average            */
#define DRIVE_BUCKET_US        250000u   /* burst allowance                */

typedef enum { DRIVE_ADVANCE = 0, DRIVE_RETARD = 1 } drive_dir;

void drive_init(void);

/* Fire immediately.  Returns false if the request was refused - too wide,
   or the duty budget is exhausted. */
bool drive_pulse(uint32_t width_us);

/* Fire once at an absolute local-timer instant.  Requests in the past, or
   further out than one swing, are refused. */
bool drive_pulse_at(uint64_t when_us, uint32_t width_us);

void drive_all_off(void);
bool drive_is_on(void);

/* "Does the magnet actually pull something in?" is a different question
   from anything above - it wants the coil held ON for whole seconds, which
   DRIVE_MAX_PULSE_US and the duty bucket exist specifically to prevent.
   Rather than widen those for everyone, this gets its own, much narrower
   door: a hard ceiling regardless of what is asked for, and it fires at
   most once per boot - the latch never clears itself, only a power cycle
   does, so running it again is a deliberate, visible act rather than a
   repeated command.  The ordinary watchdog still applies; it is just told
   about this pulse's real length instead of catching it at 55 ms. */
#define COIL_TEST_MAX_MS       5000u     /* absolute ceiling, whatever is asked */

bool drive_coil_test(uint32_t ms);       /* true if it actually fired */
bool drive_coil_test_used(void);         /* already spent this boot?  */

uint32_t drive_pulse_count(void);
uint32_t drive_refused_count(void);
uint32_t drive_budget_us(void);     /* what is left in the bucket */
uint64_t drive_last_pulse_us(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVE_H */
