/* conout.c - non-blocking, ring-buffered USB console output */

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

#include <stdbool.h>
#include "tusb.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdio/driver.h"
#include "conout.h"

/* A few screenfuls - big enough to absorb a WATCH-mode burst, or a whole
   STATUS/PULSETRACE reply, while the host catches up, without the producer
   ever waiting.  Long, open-ended tables (SWEEP over a wide range) are not
   expected to fit in any fixed size no matter how generous - those loops
   call conout_poll() every point instead, the same way they already call
   watchdog_update() every point; this buffer only has to cover what one
   command prints between one main-loop iteration and the next. */
#define CONOUT_SIZE  4096u
#define CONOUT_MASK  (CONOUT_SIZE - 1u)

static char     ring[CONOUT_SIZE];
static uint32_t head, tail;             /* free-running byte counters      */
static uint32_t dropped;

static void conout_out_chars(const char *buf, int len)
{
  int i;
  for (i = 0; i < len; i++)
  {
    if (head - tail >= CONOUT_SIZE) { dropped++; continue; }
    ring[head & CONOUT_MASK] = buf[i];
    head++;
  }
}

/* Shared by conout_poll() (called every main loop iteration) and
   conout_out_flush() (called by stdio_flush(), including the boot-time
   STEP() lines printed before the main loop even starts) - either way,
   push as much as TinyUSB's own buffer has room for right now and stop;
   never wait for the host. */
static void conout_drain(void)
{
  bool wrote = false;

  /* Unlike the SDK's own stdio_usb_out_chars(), which checks this before
     ever touching tud_cdc_write(), draining without it let bytes get staged
     into TinyUSB's own FIFO during the flaky window right after boot,
     before the host has actually connected - and a connection reset in
     that window could replay them once it settled, which is what made the
     very first fix's line print twice.  Nothing is lost by waiting: the
     ring just holds the backlog until a real connection exists, instead of
     the SDK driver's own behaviour of silently dropping it. */
  if (!tud_cdc_connected()) return;

  while (head != tail)
  {
    uint32_t used  = head - tail;
    uint32_t avail = tud_cdc_write_available();
    uint32_t chunk, n;

    if (!avail) break;

    chunk = CONOUT_SIZE - (tail & CONOUT_MASK);  /* to end of ring, or less */
    if (chunk > used)  chunk = used;
    if (chunk > avail) chunk = avail;

    n = (uint32_t)tud_cdc_write(&ring[tail & CONOUT_MASK], chunk);
    tail += n;
    wrote = true;
    if (n < chunk) break;       /* TinyUSB's own FIFO filled mid-chunk */
  }
  if (wrote) tud_cdc_write_flush();
}

static void conout_out_flush(void) { conout_drain(); }

/* Defined (non-static) in the SDK's stdio_usb.c but not declared in
   pico/stdio_usb.h - it is what the input side of &stdio_usb used to reach
   before being disabled below, and it is unaffected by that: it reads
   straight from the TinyUSB CDC RX FIFO regardless of which driver is
   "active" for output. */
extern int stdio_usb_in_chars(char *buf, int len);

static int conout_in_chars(char *buf, int len)
{
  return stdio_usb_in_chars(buf, len);
}

static stdio_driver_t conout_driver = {
  .out_chars = conout_out_chars,
  .out_flush = conout_out_flush,
  .in_chars  = conout_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
  .crlf_enabled = PICO_STDIO_USB_DEFAULT_CRLF,
#endif
};

void conout_init(void)
{
  /* stdio_usb's own tud_task() background scheduling (its low-priority IRQ)
     is set up by stdio_usb_init(), already run via stdio_init_all(), and is
     independent of whether &stdio_usb is the active output driver - only
     its blocking out_chars path is being replaced here. */
  stdio_set_driver_enabled(&stdio_usb, false);
  stdio_set_driver_enabled(&conout_driver, true);
}

void conout_poll(void) { conout_drain(); }

uint32_t conout_dropped(void) { return dropped; }
