/* drive.c */

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

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "board.h"
#include "config.h"
#include "drive.h"
#include "sense.h"

static volatile bool     pin_on;
static volatile uint64_t on_since_us;
static volatile uint32_t pulses, refused;
static volatile uint64_t last_pulse_us;

static volatile uint32_t bucket_us;      /* remaining duty allowance */
static volatile uint64_t bucket_ts_us;

static alarm_id_t        off_alarm = -1;
static alarm_id_t        on_alarm  = -1;
static repeating_timer_t guard_timer;

/* What the watchdog in guard_cb() currently treats as "too long".  Always
   DRIVE_MAX_PULSE_US except for the few seconds a coil test is actually
   running, so the watchdog still backstops that pulse too instead of
   tripping partway through it. */
static volatile uint32_t guard_ceiling_us = DRIVE_MAX_PULSE_US;
static volatile bool     coil_test_used;
static volatile bool     coil_test_active;  /* sense is blinded for this, not a real pulse */

static void coil_off(void)
{
  gpio_put(GPIO_PULSE, 0);
  pin_on = false;
}

static void coil_on(void)
{
  on_since_us = time_us_64();
  pin_on = true;
  gpio_put(GPIO_PULSE, 1);
}

static void bucket_refill(void)
{
  uint64_t now = time_us_64();
  uint64_t dt  = now - bucket_ts_us;
  uint64_t add;

  if (dt < 1000u) return;
  add = (dt * DRIVE_DUTY_PPM) / 1000000u;
  bucket_ts_us = now;
  if (add == 0u) return;
  if (bucket_us + add > DRIVE_BUCKET_US) bucket_us = DRIVE_BUCKET_US;
  else bucket_us += (uint32_t)add;
}

static int64_t off_cb(alarm_id_t id, void *user)
{
  (void)id; (void)user;
  coil_off();
  off_alarm = -1;
  return 0;
}

/* Independent of the alarm above: if the pin has been high too long for any
   reason at all, drop it. */
static bool guard_cb(repeating_timer_t *rt)
{
  (void)rt;
  bucket_refill();
  if (pin_on && (time_us_64() - on_since_us) > (uint64_t)(guard_ceiling_us + 5000u))
    coil_off();
  return true;
}

static bool start_pulse(uint32_t width_us)
{
  if (width_us == 0u || width_us > DRIVE_MAX_PULSE_US) { refused++; return false; }

  bucket_refill();
  if (bucket_us < width_us) { refused++; return false; }
  bucket_us -= width_us;

  guard_ceiling_us = DRIVE_MAX_PULSE_US;   /* an ordinary pulse always gets the ordinary ceiling */
  if (off_alarm > 0) { cancel_alarm(off_alarm); off_alarm = -1; }
  coil_on();
  off_alarm = add_alarm_in_us(width_us, off_cb, NULL, true);
  if (off_alarm < 0) { coil_off(); refused++; return false; }   /* never start what cannot be stopped */

  pulses++;
  last_pulse_us = on_since_us;
  return true;
}

static int64_t on_cb(alarm_id_t id, void *user)
{
  (void)id;
  on_alarm = -1;
  start_pulse((uint32_t)(uintptr_t)user);
  return 0;
}

void drive_init(void)
{
  gpio_init(GPIO_PULSE);
  gpio_put(GPIO_PULSE, 0);
  gpio_set_dir(GPIO_PULSE, GPIO_OUT);
  gpio_put(GPIO_PULSE, 0);
  pin_on = false;

  pulses = refused = 0;
  last_pulse_us = 0;
  bucket_us    = DRIVE_BUCKET_US;
  bucket_ts_us = time_us_64();
  guard_ceiling_us = DRIVE_MAX_PULSE_US;
  coil_test_used   = false;
  coil_test_active = false;

  add_repeating_timer_ms(-5, guard_cb, NULL, &guard_timer);
}

bool drive_pulse(uint32_t width_us) { return start_pulse(width_us); }

bool drive_pulse_at(uint64_t when_us, uint32_t width_us)
{
  uint64_t now = time_us_64();
  int64_t  dt  = (int64_t)(when_us - now);

  if (width_us == 0u || width_us > DRIVE_MAX_PULSE_US) { refused++; return false; }
  if (dt < 0) { refused++; return false; }
  /* Was a flat 2 s "two swings", which is less than ONE swing on a
     seconds pendulum.  Follow the configured period instead. */
  {
    int64_t horizon = (int64_t)(cfg_period_ns() / 1000ull) * 2ll;
    if (horizon < 100000ll) horizon = 100000ll;
    if (dt > horizon) { refused++; return false; }
  }

  if (on_alarm > 0) { cancel_alarm(on_alarm); on_alarm = -1; }
  if (dt < 200ll) return start_pulse(width_us);

  on_alarm = add_alarm_in_us((uint64_t)dt, on_cb, (void *)(uintptr_t)width_us, true);
  if (on_alarm < 0) { refused++; return false; }
  return true;
}

static int64_t coil_test_off_cb(alarm_id_t id, void *user)
{
  (void)id; (void)user;
  coil_off();
  guard_ceiling_us = DRIVE_MAX_PULSE_US;   /* hand the watchdog back its normal ceiling */
  off_alarm = -1;
  if (coil_test_active) { sense_diag_end(); coil_test_active = false; }
  return 0;
}

bool drive_coil_test(uint32_t ms)
{
  uint32_t width_us;

  if (coil_test_used) return false;        /* once per boot, no exceptions   */
  if (ms == 0u) return false;
  if (ms > COIL_TEST_MAX_MS) ms = COIL_TEST_MAX_MS;
  width_us = ms * 1000u;

  if (pin_on) return false;                /* never stack onto a live pulse  */
  if (on_alarm  >= 0) { cancel_alarm(on_alarm);  on_alarm  = -1; }
  if (off_alarm > 0)  { cancel_alarm(off_alarm); off_alarm = -1; }

  coil_test_used = true;                   /* latched before the pin ever moves */
  guard_ceiling_us = width_us;             /* the watchdog covers this pulse too */

  /* Several continuous seconds right next to the sense coil is exactly the
     disturbance sense_diag_begin()/end() exist to bracket, even though
     nothing here touches the ADC - the coupling, not ADC ownership, is
     what the detector needs shielding from. */
  sense_diag_begin();
  coil_test_active = true;
  coil_on();
  off_alarm = add_alarm_in_us(width_us, coil_test_off_cb, NULL, true);
  if (off_alarm < 0)                       /* never start what cannot be stopped */
  {
    coil_off();
    guard_ceiling_us = DRIVE_MAX_PULSE_US;
    sense_diag_end();
    coil_test_active = false;
    return false;
  }

  pulses++;
  last_pulse_us = on_since_us;
  return true;
}

bool drive_coil_test_used(void) { return coil_test_used; }

void drive_all_off(void)
{
  if (on_alarm  >= 0) { cancel_alarm(on_alarm);  on_alarm  = -1; }
  if (off_alarm > 0) { cancel_alarm(off_alarm); off_alarm = -1; }
  coil_off();
  guard_ceiling_us = DRIVE_MAX_PULSE_US;   /* in case this cut a coil test short */
  if (coil_test_active) { sense_diag_end(); coil_test_active = false; }
}

bool     drive_is_on(void)         { return pin_on; }
uint32_t drive_pulse_count(void)   { return pulses; }
uint32_t drive_refused_count(void) { return refused; }
uint32_t drive_budget_us(void)     { bucket_refill(); return bucket_us; }
uint64_t drive_last_pulse_us(void) { return last_pulse_us; }
