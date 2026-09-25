/* chime.h - pin the clock's displayed time to UTC using its chime */

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

#ifndef _CHIME_H
#define _CHIME_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The loop keeps the pendulum on rate, which makes the clock KEEP good time
   but says nothing about what the hands actually read - that depends on
   where they were put.  The chime settles it - purely as a diagnostic.

   You say which hour the clock is about to strike, and press the button when
   you hear it.  The device stamps UTC at that instant, and the difference
   between the two is how far the hands are from the truth.  From then on it
   can say when the next chime is due and what the hands should currently
   read.

   There used to be a way to hand that figure to the control loop and have
   it walk the hands into agreement via corrective pulses.  Removed: the
   magnet's authority is a fraction of a millisecond per pulse, fired at
   most every kick_min_swings swings - correcting even one second that way
   is on the order of ten thousand pulses, hours to days.  A real chime
   discrepancy gets fixed by moving the hands, not by asking the loop to
   grind it out.

   A press is a human reaction behind the sound - a few hundred milliseconds,
   biased one way - which is why chime_latency_ms exists.  It defaults to
   zero: a silent fudge factor is worse than a known bias, and a second of
   error on the hands is not what this project is short of. */

void chime_init(void);

/* Record a chime heard now, for the clock-face time face_hour:face_min.
   An hour of 1..12 is taken as whichever of the two daily possibilities is
   nearer the truth, so a 12-hour dial needs no am/pm.  False if the
   timebase has no idea what time it is yet. */
bool chime_mark(uint32_t face_hour, uint32_t face_min);

bool    chime_have(void);
int32_t chime_offset_ms(void);     /* hands ahead of true local time */
uint64_t chime_marked_utc_ns(void);

/* What the hands should currently read - the last chime mark's offset
   applied to the true local time right now.  False if never measured. */
bool chime_face_now_sod(uint32_t *face_sod);

/* When the clock will next strike, given the offset last measured.
   seconds_away and the local seconds-of-day of both the face time it will
   show and the true time it will happen at.  False if never measured. */
bool chime_next(uint32_t *seconds_away, uint32_t *face_sod, uint32_t *true_sod);

void chime_forget(void);

/* Local seconds-of-day right now, or a large sentinel if time is unknown. */
#define CHIME_SOD_UNKNOWN 0xFFFFFFFFu
uint32_t chime_local_sod(void);

#ifdef __cplusplus
}
#endif

#endif /* _CHIME_H */
