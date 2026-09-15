/* cli.h */

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

#ifndef _CLI_H
#define _CLI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void cli_init(void);
void cli_poll(void);

/* Run one command line as if it had been typed, and collect whatever it
   printed.

   The SDK's stdio layer fans every printf out to all registered drivers,
   so capturing output needs no change to any of the printf calls scattered
   through the firmware: a capture driver is registered alongside USB and
   simply switched on around the command.  Output still reaches the serial
   console at the same time, so a command issued from a browser is visible
   to anyone watching the wire.

   Injection goes through tinycl's own getchar hook, so the web console and
   the serial console share one parser and one command table - there is no
   second syntax to keep in step.

   Returns false if the command was refused; *outlen is always set. */
bool cli_run_captured(const char *cmd, char *out, uint32_t outsz, uint32_t *outlen);

/* The console ring.  Everything printed goes in, including what the control
   loop prints long after its command returned - which per-command capture
   cannot see, and which is the only record of a measurement finishing.
   Read from a cursor; `next` comes back as the cursor for the next call.
   Ask from 0 for whatever is still held. */
uint32_t cli_log_seq(void);
uint32_t cli_log_read(uint32_t from, char *out, uint32_t outsz, uint32_t *next);

#ifdef __cplusplus
}
#endif

#endif /* _CLI_H */
