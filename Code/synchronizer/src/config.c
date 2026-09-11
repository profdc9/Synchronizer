/* config.c - persistent configuration in the last flash sector */

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

#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "board.h"
#include "config.h"

synchronizer_config cfg;

/* The last sector of whatever flash the board has.  On a Pico W that is
   2 MB - 4 kB.  Nothing else in this project writes flash. */
#define CONFIG_FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define CONFIG_FLASH_ADDR    ((const uint8_t *)(XIP_BASE + CONFIG_FLASH_OFFSET))

static uint32_t config_crc(const synchronizer_config *c)
{
  /* CRC-32, bitwise; runs once per save so speed does not matter. */
  const uint8_t *p = (const uint8_t *)c;
  size_t n = offsetof(synchronizer_config, crc);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++)
  {
    crc ^= p[i];
    for (int b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
  }
  return ~crc;
}

void config_defaults(void)
{
  memset(&cfg, '\0', sizeof(cfg));
  cfg.magic   = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;

  cfg.beats_per_hour = DEFAULT_BEATS_PER_HOUR;

  /* The tank frequency is a property of the coil you wind and of C3 || C4,
     so there is no meaningful default - "sweep" finds it and "save" keeps
     it.  20 kHz is only a starting point for the first sweep. */
  cfg.tank_hz          = 20000u;
  cfg.tank_f0_hz       = 0u;      /* nothing measured yet                */
  cfg.tank_q_x10       = 0u;
  cfg.tank_peak_adc    = 0u;
  cfg.tank_floor_adc   = 0u;
  cfg.detect_threshold = 200u;
  cfg.detect_falling   = 1u;     /* detuning the tank lowers its impedance */
  cfg.sense_enabled    = 1u;

  /* Deliberately timid.  The coil current runs through R6 (10R) and the
     coil resistance is unknown until it is measured, so the first pulses
     should be short.  Widen it once you know what the coil draws. */
  cfg.pulse_us           = 2000u;
  cfg.pulse_advance_us   = 40000u;
  cfg.pulse_retard_us    = 40000u;
  cfg.pulse_authority_ns = 0;     /* unknown until "authority" is run      */

  cfg.control_enabled = 0u;       /* never runs until it is switched on    */
  cfg.kp_swings       = 4200u;    /* one hour of swings                  */
  cfg.ki_swings       = 12600u;   /* three hours                         */
  cfg.slew_limit_ppm  = 500;      /* 0.43 ms per swing, a hard ceiling   */

  strncpy(cfg.ntp_host, "pool.ntp.org", CONFIG_HOST_LEN - 1);
  cfg.tz_offset_s = 0;
  cfg.xtal_ppb    = 0;
}

void config_load(void)
{
  const synchronizer_config *f = (const synchronizer_config *)CONFIG_FLASH_ADDR;

  if (f->magic == CONFIG_MAGIC && f->version == CONFIG_VERSION &&
      f->crc == config_crc(f))
  {
    memcpy(&cfg, f, sizeof(cfg));
    return;
  }
  config_defaults();
}

bool config_save(void)
{
  static uint8_t page[FLASH_SECTOR_SIZE];
  uint32_t ints;

  if (sizeof(cfg) > FLASH_SECTOR_SIZE) return false;

  cfg.magic   = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.crc     = config_crc(&cfg);

  memset(page, 0xFF, sizeof(page));
  memcpy(page, &cfg, sizeof(cfg));

  /* Erase and program must not be interrupted by anything that executes
     from flash, and on a Pico W the cyw43 background handler does exactly
     that.  Mask interrupts across the whole operation. */
  ints = save_and_disable_interrupts();
  flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(CONFIG_FLASH_OFFSET, page, FLASH_SECTOR_SIZE);
  restore_interrupts(ints);

  return memcmp(CONFIG_FLASH_ADDR, &cfg, sizeof(cfg)) == 0;
}
