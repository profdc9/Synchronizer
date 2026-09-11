/* dhcpserver.h - the minimum DHCP server the provisioning AP needs */

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

#ifndef _DHCPSERVER_H
#define _DHCPSERVER_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip/ip_addr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A phone that joins the provisioning access point will not ask nicely for
   an address - it expects DHCP, and without it the browser never gets far
   enough to load the page.  This is just enough of a server to hand out a
   handful of leases on the AP subnet: DISCOVER/OFFER, REQUEST/ACK, and
   RELEASE.  Nothing else. */

#define DHCP_LEASES  8

void dhcpserver_start(const ip4_addr_t *gateway, const ip4_addr_t *mask);
void dhcpserver_stop(void);
bool dhcpserver_running(void);
uint32_t dhcpserver_leases(void);

#ifdef __cplusplus
}
#endif

#endif /* _DHCPSERVER_H */
