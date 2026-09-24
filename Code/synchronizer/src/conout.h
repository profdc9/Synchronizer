/* conout.h - non-blocking, ring-buffered USB console output */

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

#ifndef _CONOUT_H
#define _CONOUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The SDK's own stdio_usb driver writes straight into the TinyUSB CDC FIFO
   and, when it is full, spins calling tud_task() until either it drains or
   PICO_STDIO_USB_STDOUT_TIMEOUT_US (500 ms by default) elapses - so any
   printf() can block the caller, and therefore the whole main loop, for up
   to half a second whenever the host is slow to drain the port.  That is
   long enough to eat an in-flight NTP exchange's RTT budget (see the "ntp
   fix" logging in netclock.c) or a PTIMESCAN pulse's scheduling margin.

   This replaces that driver's output path with a RAM ring buffer: printf()
   becomes a bounded memcpy that never waits on the host, and conout_poll()
   pushes whatever is queued out over USB a little at a time, non-blocking,
   from the main loop.  Input (stdio_usb_in_chars) is untouched - only
   output goes through the ring. */
void conout_init(void);

/* Drain what's queued toward the host - never blocks.  Call every main
   loop iteration, alongside net_poll()/cyw43_arch_poll(). */
void conout_poll(void);

/* Bytes discarded because the ring filled up before conout_poll() could
   drain them - normally 0.  A nonzero count means some console output was
   lost to a slow or disconnected terminal; nothing timing-critical reads
   this, it is purely for STATUS. */
uint32_t conout_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* _CONOUT_H */
