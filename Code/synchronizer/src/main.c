/* main.c - Synchronizer: discipline a pendulum clock to NTP time */

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
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "board.h"
#include "config.h"
#include "sense.h"
#include "drive.h"
#include "timebase.h"
#include "netclock.h"
#include "control.h"
#include "cli.h"

/* Slow heartbeat on the Pico W's onboard LED: one blink per detected swing
   while events are arriving, steady off when they are not. */
static void led_task(void)
{
  static uint64_t shown;
  static bool     lit;
  uint64_t last;

  if (net_status() == NET_FAILED) return;   /* no cyw43, no LED */
  last = sense_last_event_us();

  if (last != shown && last != 0ull)
  {
    shown = last;
    lit   = true;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  }
  else if (lit && (time_us_64() - shown) > 60000ull)
  {
    lit = false;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  }
}

int main(void)
{
  stdio_init_all();

  /* Before anything else: the coil must be off and stay off.  Nothing
     downstream is allowed to assume a sane pin state. */
  drive_init();

  config_load();
  tb_init(cfg.xtal_ppb);
  sense_init();
  control_init();
  net_init();
  cli_init();

  for (;;)
  {
    cli_poll();
    control_poll();
    net_poll();
    led_task();

    /* threadsafe_background does the lwIP work in the background, but this
       keeps the driver serviced on builds that use polling instead. */
    cyw43_arch_poll();
  }
  return 0;
}
