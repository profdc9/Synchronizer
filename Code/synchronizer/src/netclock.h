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
  NET_AP,          /* raising the provisioning access point */
  NET_FAILED
} net_state;

#define NET_SCAN_MAX 16

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

/* --- provisioning ------------------------------------------------------

   With no stored credentials - or after repeatedly failing to use the ones
   it has, which is what a mistyped password looks like - the device raises
   its own access point and serves the setup page on it.  Joining that
   network and opening any page reaches the form.  Nothing else is needed:
   no serial terminal, no app. */

bool        net_in_ap(void);
const char *net_ap_ssid(void);          /* includes the board's unique id */

/* Accept credentials and go try them.  Refused unless the AP is up, so a
   stranger on your LAN cannot repoint the device at their network. */
bool        net_provision(const char *ssid, const char *pass);

void        net_ap_force(bool on);      /* raise or drop the AP by hand */

/* Best-effort list of nearby networks, to save typing an SSID on a phone.
   Scanning is not always possible while the AP is up, so an empty list is
   normal and the setup page always offers a plain text field. */
void        net_scan_start(void);
bool        net_scan_busy(void);
uint32_t    net_scan_count(void);
bool        net_scan_get(uint32_t i, const char **ssid, int16_t *rssi);

#ifdef __cplusplus
}
#endif

#endif /* _NETCLOCK_H */
