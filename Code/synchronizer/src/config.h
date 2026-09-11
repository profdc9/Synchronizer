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
#define CONFIG_VERSION  1u

#define CONFIG_SSID_LEN 33
#define CONFIG_PASS_LEN 65
#define CONFIG_HOST_LEN 48

typedef struct _synchronizer_config
{
  uint32_t magic;
  uint32_t version;

  /* --- the clock -------------------------------------------------- */
  uint32_t beats_per_hour;      /* gear ratio; 8400 for this movement    */

  /* --- sense ------------------------------------------------------ */
  uint32_t tank_hz;             /* square wave driven onto GPIO_OSCIL    */
  uint16_t detect_threshold;    /* ADC counts below/above baseline       */
  uint8_t  detect_falling;      /* 1 if the bob makes amplitude DROP     */
  uint8_t  sense_enabled;

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
  int32_t  tz_offset_s;         /* for display only; discipline is UTC   */

  /* --- learned ---------------------------------------------------- */
  int32_t  xtal_ppb;            /* last known crystal error, seeds boot  */

  uint32_t crc;                 /* over everything above                 */
} synchronizer_config;

extern synchronizer_config cfg;

void config_load(void);         /* loads, or installs defaults           */
bool config_save(void);         /* writes the flash sector               */
void config_defaults(void);     /* in RAM only; call config_save to keep */

#ifdef __cplusplus
}
#endif

#endif /* _CONFIG_H */
