/* dnsserver.c */

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
#include "pico/cyw43_arch.h"
#include "dnsserver.h"

#define DNS_PORT  53
#define DNS_MAX   512

static struct udp_pcb *pcb;
static ip4_addr_t self_ip;
static uint32_t   q_count, a_count;

static void recv_cb(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port)
{
  uint8_t  msg[DNS_MAX];
  uint16_t len, qend, qdcount;
  uint8_t *w;
  struct pbuf *out;

  (void)arg; (void)upcb;

  if (!p) return;
  q_count++;
  len = p->tot_len;
  if (len < 12u || len > sizeof(msg) - 16u) { pbuf_free(p); return; }
  pbuf_copy_partial(p, msg, len, 0);
  pbuf_free(p);

  if (msg[2] & 0x80u) return;                 /* already a response */
  qdcount = (uint16_t)((msg[4] << 8) | msg[5]);
  if (qdcount != 1u) return;                  /* one question, or nothing */

  /* Walk the QNAME labels to find where the question ends. */
  qend = 12u;
  while (qend < len && msg[qend] != 0u)
  {
    if ((msg[qend] & 0xC0u) != 0u) return;     /* no compression in a query */
    qend = (uint16_t)(qend + msg[qend] + 1u);
  }
  qend = (uint16_t)(qend + 1u + 4u);           /* root label + QTYPE + QCLASS */
  if (qend > len) return;

  msg[2] = 0x84u;                              /* response, authoritative */
  msg[3] = 0x00u;
  msg[6] = 0x00u; msg[7] = 0x01u;              /* one answer */
  msg[8] = 0x00u; msg[9] = 0x00u;              /* no authority */
  msg[10] = 0x00u; msg[11] = 0x00u;            /* no additional */

  w = msg + qend;
  *w++ = 0xC0u; *w++ = 0x0Cu;                  /* pointer back to the name */
  *w++ = 0x00u; *w++ = 0x01u;                  /* type A */
  *w++ = 0x00u; *w++ = 0x01u;                  /* class IN */
  *w++ = 0x00u; *w++ = 0x00u; *w++ = 0x00u; *w++ = 0x3Cu;   /* TTL 60 s */
  *w++ = 0x00u; *w++ = 0x04u;                  /* four bytes of address */
  memcpy(w, &self_ip.addr, 4);                 /* already network order */
  w += 4;

  out = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(w - msg), PBUF_RAM);
  if (!out) return;
  memcpy(out->payload, msg, out->len);
  udp_sendto(pcb, out, addr, port);
  a_count++;
  pbuf_free(out);
}

void dnsserver_start(struct netif *nif, const ip4_addr_t *ours)
{
  if (pcb) return;
  self_ip = *ours;
  q_count = a_count = 0;

  cyw43_arch_lwip_begin();
  pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  if (pcb)
  {
    udp_bind(pcb, IP_ANY_TYPE, DNS_PORT);
    if (nif) udp_bind_netif(pcb, nif);
    udp_recv(pcb, recv_cb, NULL);
  }
  cyw43_arch_lwip_end();
}

void dnsserver_stop(void)
{
  if (!pcb) return;
  cyw43_arch_lwip_begin();
  udp_remove(pcb);
  cyw43_arch_lwip_end();
  pcb = NULL;
}

void dnsserver_stats(uint32_t *queries, uint32_t *answers)
{
  if (queries) *queries = q_count;
  if (answers) *answers = a_count;
}

bool dnsserver_running(void) { return pcb != NULL; }
