/* netclock.c */

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

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "config.h"
#include "timebase.h"
#include "netclock.h"

#define NTP_PORT            123
#define NTP_MSG_LEN         48
#define NTP_DELTA_1900_1970 2208988800ull

/* Fast while the rate estimate is still being built, then back off.  The
   timebase needs at least two minutes between fixes to say anything useful
   about frequency, so there is no point hammering a public server. */
#define POLL_FAST_S         16u
#define POLL_SLOW_S         256u
#define FAST_FIXES          4u

#define CONNECT_TIMEOUT_MS  20000
#define NTP_TIMEOUT_MS      4000

static net_state    state = NET_OFF;
static struct udp_pcb *pcb;
static ip_addr_t    server_ip;
static bool         have_server;
static bool         dns_pending;

static uint64_t     send_us;
static bool         waiting;
static uint32_t     ok_count, fail_count, last_rtt;
static absolute_time_t next_poll, wait_deadline, retry_at;
static char         ipbuf[20];
static bool         force_sync;

/* --- NTP ------------------------------------------------------------- */

static void ntp_recv(void *arg, struct udp_pcb *p, struct pbuf *buf,
                     const ip_addr_t *addr, u16_t port)
{
  uint64_t recv_us = time_us_64();
  uint8_t  mode, stratum;
  uint8_t  ts[8];

  (void)arg; (void)p;

  if (buf->tot_len == NTP_MSG_LEN && waiting &&
      ip_addr_cmp(addr, &server_ip) && port == NTP_PORT)
  {
    pbuf_copy_partial(buf, &mode, 1, 0);
    pbuf_copy_partial(buf, &stratum, 1, 1);
    pbuf_copy_partial(buf, ts, 8, 40);          /* transmit timestamp */

    mode &= 0x07;
    if (mode == 4 && stratum != 0 && stratum < 16)
    {
      uint32_t secs = ((uint32_t)ts[0] << 24) | ((uint32_t)ts[1] << 16) |
                      ((uint32_t)ts[2] << 8)  |  (uint32_t)ts[3];
      uint32_t frac = ((uint32_t)ts[4] << 24) | ((uint32_t)ts[5] << 16) |
                      ((uint32_t)ts[6] << 8)  |  (uint32_t)ts[7];
      uint64_t rtt  = recv_us - send_us;
      uint64_t unix_ns;

      unix_ns  = ((uint64_t)secs - NTP_DELTA_1900_1970) * 1000000000ull;
      unix_ns += ((uint64_t)frac * 1000000000ull) >> 32;

      /* The server stamped the packet somewhere between our send and our
         receive; the midpoint is the best estimate we have, and the round
         trip is the uncertainty on it. */
      last_rtt = (uint32_t)rtt;
      if (tb_apply_fix(send_us + rtt / 2u, unix_ns, (uint32_t)rtt)) ok_count++;
      else fail_count++;
      waiting = false;
    }
  }
  pbuf_free(buf);
}

static void dns_done(const char *name, const ip_addr_t *addr, void *arg)
{
  (void)name; (void)arg;
  dns_pending = false;
  if (addr) { server_ip = *addr; have_server = true; }
}

static void ntp_send(void)
{
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
  uint8_t *req;

  if (!p) { fail_count++; return; }
  req = (uint8_t *)p->payload;
  memset(req, 0, NTP_MSG_LEN);
  req[0] = 0x1B;                      /* LI 0, VN 3, mode 3 (client) */

  cyw43_arch_lwip_begin();
  send_us = time_us_64();
  udp_sendto(pcb, p, &server_ip, NTP_PORT);
  cyw43_arch_lwip_end();
  pbuf_free(p);

  waiting = true;
  wait_deadline = make_timeout_time_ms(NTP_TIMEOUT_MS);
}

/* --- public ---------------------------------------------------------- */

void net_init(void)
{
  if (cyw43_arch_init())
  {
    state = NET_FAILED;
    return;
  }
  cyw43_arch_enable_sta_mode();

  cyw43_arch_lwip_begin();
  pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  if (pcb) udp_recv(pcb, ntp_recv, NULL);
  cyw43_arch_lwip_end();

  state      = NET_OFF;
  next_poll  = get_absolute_time();
  retry_at   = get_absolute_time();
  ok_count = fail_count = 0;
}

void net_reconnect(void)
{
  state    = NET_OFF;
  retry_at = get_absolute_time();
  have_server = false;
}

void net_request_sync(void) { force_sync = true; }

void net_poll(void)
{
  if (state == NET_FAILED) return;
  if (cfg.ssid[0] == '\0') return;

  if (state == NET_OFF)
  {
    if (absolute_time_diff_us(get_absolute_time(), retry_at) > 0) return;
    state = NET_CONNECTING;
    if (cyw43_arch_wifi_connect_timeout_ms(cfg.ssid, cfg.pass,
                                           CYW43_AUTH_WPA2_AES_PSK,
                                           CONNECT_TIMEOUT_MS))
    {
      state    = NET_OFF;
      retry_at = make_timeout_time_ms(15000);
      return;
    }
    state     = NET_ONLINE;
    next_poll = get_absolute_time();
    return;
  }

  if (state != NET_ONLINE) return;

  if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP)
  {
    state    = NET_OFF;
    retry_at = make_timeout_time_ms(5000);
    return;
  }

  if (waiting)
  {
    if (absolute_time_diff_us(get_absolute_time(), wait_deadline) < 0)
    {
      waiting = false;
      fail_count++;
      have_server = false;          /* try DNS again; the server may be gone */
    }
    return;
  }

  if (!force_sync && absolute_time_diff_us(get_absolute_time(), next_poll) > 0)
    return;

  if (!have_server)
  {
    err_t e;
    if (dns_pending) return;
    dns_pending = true;
    cyw43_arch_lwip_begin();
    e = dns_gethostbyname(cfg.ntp_host[0] ? cfg.ntp_host : "pool.ntp.org",
                          &server_ip, dns_done, NULL);
    cyw43_arch_lwip_end();
    if (e == ERR_OK)      { dns_pending = false; have_server = true; }
    else if (e != ERR_INPROGRESS) { dns_pending = false; fail_count++;
                                    next_poll = make_timeout_time_ms(10000); }
    return;
  }

  force_sync = false;
  next_poll  = make_timeout_time_ms(
                 1000u * ((tb_fix_count() < FAST_FIXES) ? POLL_FAST_S : POLL_SLOW_S));
  ntp_send();
}

net_state net_status(void) { return state; }

const char *net_status_name(void)
{
  switch (state)
  {
    case NET_OFF:        return cfg.ssid[0] ? "offline" : "no ssid";
    case NET_CONNECTING: return "connecting";
    case NET_ONLINE:     return "online";
    case NET_FAILED:     return "failed";
  }
  return "?";
}

const char *net_ip(void)
{
  if (state != NET_ONLINE) return "-";
  snprintf(ipbuf, sizeof(ipbuf), "%s",
           ip4addr_ntoa(netif_ip4_addr(netif_list)));
  return ipbuf;
}

uint32_t net_ntp_ok(void)       { return ok_count; }
uint32_t net_ntp_fail(void)     { return fail_count; }
uint32_t net_last_rtt_us(void)  { return last_rtt; }
