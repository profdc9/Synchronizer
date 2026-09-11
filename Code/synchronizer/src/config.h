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
#define CONFIG_VERSION  4u

#define CONFIG_SSID_LEN 33
#define CONFIG_PASS_LEN 65
#define CONFIG_HOST_LEN 48

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
  uint16_t detect_threshold;    /* ADC counts below/above baseline       */
  uint8_t  detect_falling;      /* 1 if the bob makes amplitude DROP     */
  uint8_t  sense_enabled;
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
  uint16_t pulse_us;            /* width of one correction pulse         */
  uint16_t pulse_advance_us;    /* fire this long BEFORE expected arrival */
  uint16_t pulse_retard_us;     /* fire this long AFTER the bob leaves   */
  int32_t  pulse_authority_ns;  /* measured phase step per pulse, ns     */

  /* --- control loop ----------------------------------------------- */
  uint8_t  control_enabled;
  uint8_t  pad0[3];
  /* The loop is parameterised in swings rather than seconds because the
     pendulum, not the wall clock, is what it acts on.  kp_swings is the
     number of swings over which a standing phase error would be taken out
     by the proportional term alone; ki_swings sets the integral time. */
  uint32_t kp_swings;
  uint32_t ki_swings;
  int32_t  slew_limit_ppm;      /* cap on commanded rate correction      */

  /* --- network ---------------------------------------------------- */
  char     ssid[CONFIG_SSID_LEN];
  char     pass[CONFIG_PASS_LEN];
  char     ntp_host[CONFIG_HOST_LEN];

  /* Key for the provisioning access point the device raises when it has no
     credentials, or cannot use the ones it has.  It is not a secret worth
     much - it is printed in the README - but it does keep your home Wi-Fi
     password from crossing an open link in the clear while you type it in. */
  char     ap_pass[CONFIG_PASS_LEN];
  int32_t  tz_offset_s;         /* for display only; discipline is UTC   */

  /* --- learned ---------------------------------------------------- */
  int32_t  xtal_ppb;            /* last known crystal error, seeds boot  */

  uint32_t crc;                 /* over everything above                 */
} synchronizer_config;

extern synchronizer_config cfg;

void config_load(void);         /* loads, or installs defaults           */

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
