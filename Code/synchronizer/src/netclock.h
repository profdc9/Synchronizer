/* netclock.h - WiFi association and the NTP client */

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

#ifndef _NETCLOCK_H
#define _NETCLOCK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  NET_OFF = 0,
  NET_CONNECTING,
  NET_ONLINE,
  NET_FAILED
} net_state;

void        net_init(void);
void        net_poll(void);
net_state   net_status(void);
const char *net_status_name(void);
const char *net_ip(void);
void        net_request_sync(void);     /* force an NTP exchange now */
uint32_t    net_ntp_ok(void);
uint32_t    net_ntp_fail(void);
uint32_t    net_last_rtt_us(void);
void        net_reconnect(void);        /* after credentials change */

#ifdef __cplusplus
}
#endif

#endif /* _NETCLOCK_H */
