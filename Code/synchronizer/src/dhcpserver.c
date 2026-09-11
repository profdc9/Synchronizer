/* dhcpserver.c */

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

#include <string.h>
#include "pico/stdlib.h"
#include "lwip/udp.h"
#include "lwip/netif.h"
#include "dhcpserver.h"

#define PORT_SERVER   67
#define PORT_CLIENT   68
#define LEASE_SECS    (60u * 60u)
#define MAGIC         0x63825363u

/* BOOTP/DHCP message.  The options run past the end of this struct. */
typedef struct
{
  uint8_t  op, htype, hlen, hops;
  uint32_t xid;
  uint16_t secs, flags;
  uint8_t  ciaddr[4], yiaddr[4], siaddr[4], giaddr[4];
  uint8_t  chaddr[16];
  uint8_t  sname[64], file[128];
  uint8_t  options[312];
} dhcp_msg;

typedef struct { uint8_t mac[6]; uint32_t expiry_s; bool used; } lease;

static struct udp_pcb *pcb;
static lease     leases[DHCP_LEASES];
static ip4_addr_t gw, nm;
static uint32_t  lease_count;

/* --- option walking ---------------------------------------------------- */

static const uint8_t *opt_find(const dhcp_msg *m, uint16_t len, uint8_t want, uint8_t *olen)
{
  const uint8_t *p   = m->options;
  const uint8_t *end = (const uint8_t *)m + len;

  if ((uint32_t)(end - p) < 4u) return NULL;
  p += 4;                                     /* skip the magic cookie */
  while (p + 1 < end && *p != 0xFF)
  {
    uint8_t code = p[0], l = p[1];
    if (code == 0) { p++; continue; }         /* pad */
    if (p + 2 + l > end) break;
    if (code == want) { if (olen) *olen = l; return p + 2; }
    p += 2 + l;
  }
  return NULL;
}

static uint8_t *opt_put(uint8_t *p, uint8_t code, uint8_t len, const void *val)
{
  *p++ = code; *p++ = len;
  memcpy(p, val, len);
  return p + len;
}

static uint8_t *opt_put_u32(uint8_t *p, uint8_t code, uint32_t v)
{
  uint8_t b[4];
  b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16);
  b[2] = (uint8_t)(v >> 8);  b[3] = (uint8_t)v;
  return opt_put(p, code, 4, b);
}

/* --- leases ------------------------------------------------------------ */

/* Addresses run from gateway+2 upward, so .1 stays ours. */
static uint8_t lease_index(const uint8_t *mac)
{
  uint32_t now = (uint32_t)(time_us_64() / 1000000ull);
  int i, free_slot = -1;

  for (i = 0; i < DHCP_LEASES; i++)
  {
    if (leases[i].used && memcmp(leases[i].mac, mac, 6) == 0) return (uint8_t)i;
    if (!leases[i].used || leases[i].expiry_s < now) { if (free_slot < 0) free_slot = i; }
  }
  if (free_slot < 0) free_slot = 0;           /* steal the oldest slot */
  if (!leases[free_slot].used) lease_count++;
  memcpy(leases[free_slot].mac, mac, 6);
  leases[free_slot].used = true;
  return (uint8_t)free_slot;
}

/* --- the server -------------------------------------------------------- */

static void recv_cb(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port)
{
  dhcp_msg m;
  uint8_t  type = 0, idx;
  const uint8_t *o;
  uint8_t *w;
  struct pbuf *out;
  uint32_t server_ip = lwip_ntohl(ip4_addr_get_u32(&gw));
  uint32_t client_ip;

  (void)arg; (void)upcb; (void)addr; (void)port;

  if (p->tot_len < 240u || p->tot_len > sizeof(m)) { pbuf_free(p); return; }
  memset(&m, 0, sizeof(m));
  pbuf_copy_partial(p, &m, p->tot_len, 0);
  pbuf_free(p);

  if (m.op != 1u) return;                     /* not a request */
  o = opt_find(&m, (uint16_t)sizeof(m), 53, NULL);
  if (!o) return;
  type = *o;

  if (type == 7u)                             /* RELEASE */
  {
    int i;
    for (i = 0; i < DHCP_LEASES; i++)
      if (leases[i].used && memcmp(leases[i].mac, m.chaddr, 6) == 0) leases[i].used = false;
    return;
  }
  if (type != 1u && type != 3u) return;       /* only DISCOVER and REQUEST */

  idx       = lease_index(m.chaddr);
  client_ip = (server_ip & 0xFFFFFF00u) | (uint32_t)((server_ip & 0xFFu) + 1u + idx);
  leases[idx].expiry_s = (uint32_t)(time_us_64() / 1000000ull) + LEASE_SECS;

  /* Build the reply on top of the request, as BOOTP expects. */
  m.op    = 2u;                               /* BOOTREPLY */
  m.secs  = 0;
  m.yiaddr[0] = (uint8_t)(client_ip >> 24); m.yiaddr[1] = (uint8_t)(client_ip >> 16);
  m.yiaddr[2] = (uint8_t)(client_ip >> 8);   m.yiaddr[3] = (uint8_t)client_ip;
  memset(m.options, 0, sizeof(m.options));

  w = m.options;
  *w++ = 0x63; *w++ = 0x82; *w++ = 0x53; *w++ = 0x63;
  {
    uint8_t reply = (type == 1u) ? 2u : 5u;   /* OFFER : ACK */
    w = opt_put(w, 53, 1, &reply);
  }
  w = opt_put_u32(w, 54, server_ip);                       /* server id   */
  w = opt_put_u32(w, 51, LEASE_SECS);                      /* lease time  */
  w = opt_put_u32(w, 1,  lwip_ntohl(ip4_addr_get_u32(&nm)));/* subnet mask */
  w = opt_put_u32(w, 3,  server_ip);                       /* router      */
  w = opt_put_u32(w, 6,  server_ip);                       /* DNS - us,
                                    so every lookup lands on the page    */
  *w++ = 0xFF;

  out = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(w - (uint8_t *)&m), PBUF_RAM);
  if (!out) return;
  memcpy(out->payload, &m, out->len);
  /* The client has no address yet, so this has to go to the broadcast. */
  udp_sendto(pcb, out, IP_ADDR_BROADCAST, PORT_CLIENT);
  pbuf_free(out);
}

void dhcpserver_start(const ip4_addr_t *gateway, const ip4_addr_t *mask)
{
  if (pcb) return;
  gw = *gateway; nm = *mask;
  memset(leases, 0, sizeof(leases));
  lease_count = 0;

  pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  if (!pcb) return;
  udp_bind(pcb, IP_ANY_TYPE, PORT_SERVER);
  udp_recv(pcb, recv_cb, NULL);
}

void dhcpserver_stop(void)
{
  if (!pcb) return;
  udp_remove(pcb);
  pcb = NULL;
}

bool dhcpserver_running(void) { return pcb != NULL; }
uint32_t dhcpserver_leases(void) { return lease_count; }
