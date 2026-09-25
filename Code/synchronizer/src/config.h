/* config.h - flash-backed persistent configuration */

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

#ifndef _CONFIG_H
#define _CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_MAGIC    0x53594e43u   /* "SYNC" */
#define CONFIG_VERSION  21u

/* Who we are on the network lives in its own sector, with its own magic and
   its own version that changes only when THESE fields change.

   The main configuration restores defaults whenever its layout changes,
   which is the right thing for tuning values - they are guesses anyway, and
   reading a stale layout would be worse.  It is the wrong thing for Wi-Fi
   credentials: it means every firmware update that adds a setting throws
   the device off the network and back to its setup access point, which
   somebody then has to re-provision from a phone.  Keeping them apart costs
   one flash sector. */
#define NETCFG_MAGIC    0x53594e4eu   /* "SYNN" */
#define NETCFG_VERSION  1u

#define CONFIG_SSID_LEN 33
#define CONFIG_PASS_LEN 65
#define CONFIG_HOST_LEN 48
#define CONFIG_NAME_LEN 32

typedef struct _synchronizer_config
{
  uint32_t magic;
  uint32_t version;

  /* --- the clock --------------------------------------------------

     Everything about the movement and the coil placement lives here, so
     the firmware never assumes the clock it was developed on.  The
     defaults describe that clock; they are a starting point, not a
     specification. */

  uint32_t beats_per_hour;      /* gear ratio: escapement beats per hour */
  uint8_t  beats_per_period;    /* beats per FULL swing; 2 for an anchor,
                                   deadbeat or pin-pallet escapement     */

  /* Where the coils sit on the swing.

     events_per_period is 1 for a sense coil at an extreme of the swing -
     the bob only reaches it once per full period - and 2 for one at the
     centre, which the bob crosses twice.

     drive_offset_ppt is how long after a sense event the bob reaches the
     DRIVE coil, in parts per thousand of a full period.  Coils at
     opposite extremes is 500.  Both at the same extreme is 0.  Sense at
     the centre and drive at an extreme is 250, which holds for either
     centre crossing. */
  uint8_t  events_per_period;
  uint16_t drive_offset_ppt;

  /* --- sense ------------------------------------------------------ */
  uint32_t tank_hz;             /* square wave driven onto GPIO_OSCIL    */
  /* Drive level, as the WIDTH of the pulse put on GPIO_OSCIL.  There is
     no gain trim in hardware and the tank impedance depends entirely on
     the coil someone wound, so the only way to keep the envelope off the
     rail is to drive it less hard.

     Nanoseconds rather than duty, because the PWM wrap changes with
     frequency: a fixed duty is a MOVING pulse width, and the switching
     pair has a threshold below which it does not respond at all - 56 ns
     works on the development board, 48 ns does nothing.  Mid-scan that
     turns an ordinary rounding step into a cliff, the drive stops, and
     the rest of the sweep reads as floor.  A width is constant across a
     scan by construction, and it is what the transistors react to.

     Clamped to half a period, which is full drive. */
  uint32_t tank_drive_ns;       /* 0 = no drive at all, a diagnostic     */
  uint16_t detect_threshold;    /* ADC counts below/above baseline       */
  uint8_t  detect_falling;      /* 1 if the bob makes amplitude DROP     */
  uint8_t  sense_enabled;
  /* Hysteresis, as a percentage of detect_threshold.  Without it, ripple
     sitting near the threshold ends the event early and the width gate
     then throws the whole event away - not a jittered timestamp, a lost
     swing.  The event is only over once the excursion has fallen this far
     BELOW the threshold.

     The threshold crossings themselves stay symmetric: both edges are
     timed at detect_threshold, and the event is timestamped at their
     midpoint, so the timing is unaffected by how deep the dip goes.  The
     hysteresis decides when the event has ENDED, never when it crossed. */
  uint8_t  detect_hyst_pct;     /* 0 = none, 30 is a good starting point */
  uint8_t  pad_det[3];
  uint32_t sample_hz;           /* envelope sampling rate                */

  /* Detector timing, as percentages of the expected interval between
     sense events, so they scale with the pendulum instead of assuming a
     fast one. */
  uint8_t  rearm_pct;           /* ignore events closer together than    */
  uint8_t  min_event_pct;       /* a bump shorter than this is noise     */
  uint8_t  max_event_pct;       /* longer than this is something stuck   */
  uint8_t  acquire_tol_pct;     /* gap tolerance while acquiring lock    */
  uint16_t acquire_events;      /* consecutive good events to lock       */

  /* What RESONANCE last measured, with nothing metallic near the coil.
     Kept so STATUS can show how far the drive has been detuned from the
     peak on purpose, and so a later scan can be compared with this one. */
  uint32_t tank_f0_hz;          /* peak of the resonance curve           */
  uint32_t tank_q_x10;          /* f0 / -3 dB bandwidth, times ten       */
  uint16_t tank_peak_adc;       /* envelope reading at the peak          */
  uint16_t tank_floor_adc;      /* envelope reading far off resonance    */

  /* --- drive ------------------------------------------------------ */
  uint32_t pulse_us;            /* width for manual PULSE/PULSETRACE, and
                                    the fallback below if a directional
                                    width is ever left at 0              */
  /* Both placements are offsets from the SAME centre - drive_offset_ppt's
     instant, halfway between sense events for this clock's opposite-
     extreme coils - not two independently-anchored windows.  Signed and
     wide (int32_t, not the old uint16_t) so either can reach anywhere
     within a full half-period of that centre in either direction: fixed
     at +-65535 us, the two used to be structurally unable to reach a
     ~300 ms band between them regardless of what was asked for, and a
     magnet moved from where the geometry was last measured can easily
     need more range than that to find again.
       retard:  when = centre + pulse_retard_us
       advance: when = centre + pulse_advance_us
     Same formula for both - a given number means the same physical instant
     whether it is stored as the retard placement or the advance one.  A
     scan run under one label tells you exactly where to look under the
     other, with no sign to flip to compare them; the two fields stay
     independent only because the loop must remember a price and a
     placement for each direction separately, not because the arithmetic
     differs between them.

     Advance and retard are NOT symmetric in what they're worth, though -
     an attract-only coil retards when the bob is on its side of centre
     and advances when the bob is at the far extreme, where the field is
     much weaker - the two can differ by an order of magnitude.  One
     number for both would make the loop believe it had paid for a
     correction it never delivered. */
  int32_t  pulse_advance_us;
  int32_t  pulse_retard_us;
  /* Separate WIDTHS, not just placements - the same order-of-magnitude
     asymmetry that makes one placement retard far harder than the other
     advances (see above) means a width chosen to be safe on the strong
     side is often too narrow to do anything useful on the weak one, and
     a width chosen for the weak side can be needlessly hard on the
     strong one.  Set together by PW.  0 deliberately disables that
     direction's corrections entirely (fire_for() in control.c refuses
     rather than guessing a width) - the same idiom the old AUTH prices
     used, and what lets a retard-only installation leave advance at 0
     and safely never act on it. */
  uint32_t pulse_advance_width_us;
  uint32_t pulse_retard_width_us;

  /* --- control loop -----------------------------------------------
     KICK is the only algorithm now - a bang-bang hysteresis scheme that
     needs no measured authority at all; see kick_min_swings/
     kick_threshold_pct below and the kick_step() comment in control.c.
     It picks advance or retard itself, at runtime, from which threshold
     it hit.  The AUTH mode this used to select against (spend the phase
     error against a measured ns-per-pulse price from MEASURE/PTIMESCAN)
     and the mode selector itself were removed once KICK proved out -
     see git history if that measured-authority path is ever wanted
     back. */
  uint8_t  control_enabled;
  uint8_t  pad_mode;            /* was control_mode (AUTH/KICK select);
                                    unused now that KICK is the only
                                    algorithm - left in place rather than
                                    reshuffling the layout             */
  uint8_t  pad_kickdir;         /* was a KICK direction setting; the
                                    controller now picks direction itself,
                                    so this is unused - left in place
                                    rather than reshuffling the layout */
  uint8_t  pad0;
  uint16_t kick_min_swings;     /* KICK mode: min swings between pulses    */
  /* KICK's hysteresis trips at +-this fraction of a swing (percent of the
     nominal interval) on filt_err (edge-based, rerr) - not demod_ns, which
     turned out vulnerable to sense-coil proximity artifacts a plain
     magnet near the coil could trigger.  25 (a quarter swing) is a wide
     enough band to sit comfortably above ordinary measurement noise.
     Reuses what used to be plain padding, so no config version bump. */
  uint16_t kick_threshold_pct;
  /* The loop is parameterised in swings rather than seconds because the
     pendulum, not the wall clock, is what it acts on.  kp_swings is the
     number of swings over which a standing phase error would be taken out
     by the proportional term alone; ki_swings sets the integral time. */
  uint32_t kp_swings;
  uint32_t ki_swings;
  int32_t  slew_limit_ppm;      /* cap on commanded rate correction      */

  /* Phase time constant, in events, of the NCO that tracks the PENDULUM -
     not the one the discipline loop steers against.  Its learned rate is
     the period estimate and the loop's feedforward term.

     Longer than it needs to be on purpose.  The estimate is only as good as
     the timebase it is measured against, and the timebase learns the
     crystal's rate from NTP fixes minutes apart.  A crystal excursion
     faster than that has not been corrected yet and would show up here as
     an apparent pendulum rate change; averaging over longer than the
     timebase's own settling leaves most of it behind.  350 events is about
     five minutes on a seconds-ish pendulum, which buys roughly 1 ppm. */
  uint32_t rate_kp_events;

  /* Detector filtering.  These were compile-time until the clock was about
     to go back to its own room, where changing them means fetching the
     board.  env_oversample conversions are taken per tick, sorted, and
     env_trim discarded from each end before averaging - trim of half the
     count minus one is a plain median, zero is an arithmetic mean.

     The trim is the one most likely to want changing in place: a drive coil
     pulsing a few inches from the sense coil is a far bigger impulsive
     source than anything on the board, and impulses are what trimming is
     for.  baseline_shift is the detector baseline's EMA, 2^n milliseconds,
     and wants raising if the baseline ever slides toward the mean of a
     swinging signal instead of sitting at its quiescent level. */
  uint8_t  env_oversample;      /* 4..32 conversions per tick            */
  uint8_t  env_trim;            /* dropped from each end, < half of that */
  uint8_t  baseline_shift;      /* EMA time constant, 2^n ms             */
  uint8_t  pad_env;

  /* --- the chime ---------------------------------------------------

     Where the hands actually are, measured by listening.  The loop keeps
     the pendulum on rate; this is what says whether the dial agrees. */
  uint32_t chime_interval_min;  /* 60 hourly, 30 half, 15 quarters       */
  int32_t  chime_offset_ms;     /* hands ahead of true local time        */
  uint32_t chime_latency_ms;    /* allowance for the press trailing it   */
  uint64_t chime_ref_utc;       /* unix seconds of the last mark         */
  uint8_t  chime_valid;
  uint8_t  pad1[3];

  /* --- network ---------------------------------------------------- */
  char     ssid[CONFIG_SSID_LEN];
  char     pass[CONFIG_PASS_LEN];
  char     ntp_host[CONFIG_HOST_LEN];

  /* Key for the provisioning access point the device raises when it has no
     credentials, or cannot use the ones it has.  It is not a secret worth
     much - it is printed in the README - but it does keep your home Wi-Fi
     password from crossing an open link in the clear while you type it in. */
  char     ap_pass[CONFIG_PASS_LEN];

  /* Advertised over mDNS, so the device answers to <hostname>.local and
     shows up in service browsers instead of only at an IP address.  Must
     be a single DNS label: letters, digits and hyphens. */
  char     hostname[CONFIG_NAME_LEN];
  int32_t  tz_offset_s;         /* for display only; discipline is UTC   */

  /* Type-2 loop time constant for the NTP-disciplined timebase, in FIXES
     rather than seconds - fixes do not land on a uniform clock the way
     pendulum swings do, but treating each accepted one as one "event" is
     otherwise the same shape as control.c's rate_kp_events, and ki follows
     it the same way (2*kp*kp).  Each individual fix's offset is noisy
     (network jitter); the crystal it is measuring only drifts with
     temperature, so there is nothing to lose by damping hard. */
  uint32_t tb_kp_fixes;

  /* --- learned ---------------------------------------------------- */
  int32_t  xtal_ppb;            /* last known crystal error, seeds boot  */

  uint32_t crc;                 /* over everything above                 */
} synchronizer_config;

extern synchronizer_config cfg;

void config_load(void);         /* loads, or installs defaults           */
bool config_save_network(void); /* the credentials sector alone          */

/* Derived from the configuration above; the single source of truth for
   every piece of code that needs to know how fast this clock runs. */
uint64_t cfg_period_ns(void);        /* one full pendulum swing          */
uint64_t cfg_event_interval_ns(void);/* between successive sense events  */
void     cfg_event_ratio(uint64_t *num, uint64_t *den); /* exact form    */
bool config_save(void);         /* writes the flash sector               */
void config_defaults(void);     /* in RAM only; call config_save to keep */

#ifdef __cplusplus
}
#endif

#endif /* _CONFIG_H */
