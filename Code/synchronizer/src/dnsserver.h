/* dnsserver.h - answer every lookup with our own address */

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

#ifndef _DNSSERVER_H
#define _DNSSERVER_H

#include <stdbool.h>
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Phones decide whether a network has working internet by fetching a known
   URL.  Pointing every name at ourselves makes that check land on the
   provisioning page, which is what pops the "sign in to network" sheet
   instead of leaving the user to guess an IP address.

   Only used while the provisioning AP is up. */

void dnsserver_start(struct netif *nif, const ip4_addr_t *ours);
void dnsserver_stop(void);
bool dnsserver_running(void);
void dnsserver_stats(uint32_t *queries, uint32_t *answers);

#ifdef __cplusplus
}
#endif

#endif /* _DNSSERVER_H */
