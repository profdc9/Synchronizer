/* webui.h - the browser interface served from the Pico W */

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

#ifndef _WEBUI_H
#define _WEBUI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A resonance scan takes about three seconds and an authority measurement
   runs for tens of swings.  Neither can happen inside an HTTP handler, so
   a request only queues the job; web_poll() runs it from the main loop and
   the page watches "job" in the status for it to finish. */

void web_init(void);
void web_poll(void);
bool web_busy(void);
const char *web_job_name(void);

/* The pages, in flash.  web_setup is the provisioning form served while
   the device is being its own access point. */
extern const char web_page[];
extern const uint32_t web_page_len;
extern const char web_setup[];
extern const uint32_t web_setup_len;
/* Hash of the page source, so a browser tab left open across a reflash
   can notice that its JavaScript is older than the firmware. */
extern const char web_version[];

#ifdef __cplusplus
}
#endif

#endif /* _WEBUI_H */
