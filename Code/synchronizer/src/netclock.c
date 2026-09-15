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
#include "pico/unique_id.h"
#include "lwip/udp.h"
#include "lwip/dns.h"
#include "lwip/ip_addr.h"
#include "config.h"
#include "timebase.h"
#include "netclock.h"
#include "dhcpserver.h"
#include "dnsserver.h"
#include "httpd.h"
#include "lwip/apps/mdns.h"

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

/* Three failures in a row is what a mistyped password looks like.  Raise
   the provisioning AP rather than retrying forever with no way in. */
#define STA_FAILS_TO_AP     3
#define AP_RETRY_MS         60000u   /* rescue AP first re-tries this soon */
#define AP_RETRY_MAX_MS     900000u  /* ...backing off to this between tries */

static net_state    state = NET_OFF;
static struct udp_pcb *pcb;
static ip_addr_t    server_ip;
static bool         have_server;
static bool         dns_pending;

static uint64_t     send_us;
static bool         waiting;
static uint32_t     ok_count, fail_count, last_rtt;
static absolute_time_t next_poll, wait_deadline, retry_at;
static absolute_time_t ap_retry_at;
static uint32_t        ap_http_mark;
static uint32_t        ap_retry_ms = AP_RETRY_MS;
static char         ipbuf[20];
static bool         force_sync;

static uint32_t     sta_fails;
static absolute_time_t connect_deadline;
static bool         ap_up, ap_forced;

/* Requests that arrive from an HTTP handler cannot be carried out there.
   net_provision() would write flash with interrupts masked, and both it and
   net_ap_force() would tear down the very interface the reply still has to
   go out on.  They record what was asked for; net_poll() does it. */
typedef enum { PEND_NONE = 0, PEND_PROVISION, PEND_AP_ON, PEND_AP_OFF } pending_op;
static volatile pending_op pending;
static absolute_time_t     pending_at;
static char         ap_ssid[33];
static ip4_addr_t   ap_ip, ap_mask;

static bool         mdns_ready, mdns_on_sta, mdns_on_ap;

static char         scan_ssid[NET_SCAN_MAX][33];
static int16_t      scan_rssi[NET_SCAN_MAX];
static uint32_t     scan_count;
static bool         scan_running;

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

/* The driver defaults to CYW43_PERFORMANCE_PM, which is PM2 power save with
   a 200 ms sleep timer.  It leaves the board associated and holding a DHCP
   lease while quietly dropping unicast traffic: ARP goes unanswered, so
   nothing on the network can reach it even though it looks connected, and
   mDNS answers come and go with the beacon interval.

   This device is powered from the wall and wants both a responsive web
   interface and low-jitter NTP, so there is nothing to save power for.
   The driver reapplies its default whenever the first interface comes up,
   so this has to be reasserted rather than set once. */
static void power_save_off(void)
{
  cyw43_arch_lwip_begin();
  cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);
  cyw43_arch_lwip_end();
}

/* --- mDNS / DNS-SD ------------------------------------------------------ */

const char *net_hostname(void)
{
  return cfg.hostname[0] ? cfg.hostname : "synchronizer";
}

bool net_mdns_active(void) { return mdns_on_sta || mdns_on_ap; }

static void mdns_txt(struct mdns_service *service, void *userdata)
{
  (void)userdata;
  mdns_resp_add_service_txtitem(service, "path=/", 6);
}

static struct netif *itf(int which)
{
  return &cyw43_state.netif[which];
}

static void mdns_up(int which, bool *flag)
{
  struct netif *nif = itf(which);

  if (*flag || !netif_is_up(nif)) return;

  cyw43_arch_lwip_begin();
  if (!mdns_ready) { mdns_resp_init(); mdns_ready = true; }
  if (mdns_resp_add_netif(nif, net_hostname()) == ERR_OK)
  {
    mdns_resp_add_service(nif, net_hostname(), "_http", DNSSD_PROTO_TCP,
                          80, mdns_txt, NULL);
    mdns_resp_announce(nif);
    *flag = true;
  }
  cyw43_arch_lwip_end();
}

/* cyw43_tcpip_init makes whichever interface came up last the default, and
   disabling AP mode leaves that netif registered and still default even
   though its link is down.  lwIP sends anything off-subnet - a DNS lookup,
   an NTP exchange - by the default route, so after provisioning the traffic
   left by a dead interface and nothing ever resolved.  Point the default at
   whichever interface is actually carrying traffic. */
static void set_default_route(int which)
{
  struct netif *nif = itf(which);
  cyw43_arch_lwip_begin();
  if (netif_is_up(nif)) netif_set_default(nif);
  cyw43_arch_lwip_end();
}

static void mdns_down(int which, bool *flag)
{
  if (!*flag) return;
  cyw43_arch_lwip_begin();
  mdns_resp_remove_netif(itf(which));
  cyw43_arch_lwip_end();
  *flag = false;
}

/* A hostname has to survive being a DNS label: letters, digits and hyphens,
   no leading or trailing hyphen.  Anything else is dropped rather than
   rejected, so a name with a space in it still produces something usable. */
bool net_set_hostname(const char *name)
{
  char clean[CONFIG_NAME_LEN];
  uint32_t n = 0;

  if (!name) return false;
  while (*name && n < CONFIG_NAME_LEN - 1u)
  {
    char ch = *name++;
    if (ch >= 'A' && ch <= 'Z') ch = (char)(ch - 'A' + 'a');
    if ((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') ||
        (ch == '-' && n > 0u))
      clean[n++] = ch;
  }
  while (n > 0u && clean[n - 1u] == '-') n--;
  clean[n] = '\0';
  if (n == 0u) return false;

  strncpy(cfg.hostname, clean, CONFIG_NAME_LEN - 1);
  cfg.hostname[CONFIG_NAME_LEN - 1] = '\0';

  /* Re-announce under the new name wherever we are already answering. */
  cyw43_arch_lwip_begin();
  if (mdns_on_sta) { mdns_resp_rename_netif(itf(CYW43_ITF_STA), cfg.hostname);
                     mdns_resp_announce(itf(CYW43_ITF_STA)); }
  if (mdns_on_ap)  { mdns_resp_rename_netif(itf(CYW43_ITF_AP), cfg.hostname);
                     mdns_resp_announce(itf(CYW43_ITF_AP)); }
  cyw43_arch_lwip_end();
  return true;
}

/* --- scanning ---------------------------------------------------------- */

static int scan_cb(void *env, const cyw43_ev_scan_result_t *r)
{
  uint32_t i;
  (void)env;

  if (!r || r->ssid_len == 0u) return 0;
  for (i = 0; i < scan_count; i++)
    if (strncmp(scan_ssid[i], (const char *)r->ssid, r->ssid_len) == 0) return 0;
  if (scan_count >= NET_SCAN_MAX) return 0;

  memcpy(scan_ssid[scan_count], r->ssid,
         (r->ssid_len > 32u) ? 32u : r->ssid_len);
  scan_ssid[scan_count][(r->ssid_len > 32u) ? 32u : r->ssid_len] = '\0';
  scan_rssi[scan_count] = r->rssi;
  scan_count++;
  return 0;
}

/* Scanning asks the radio to leave its channel, which it cannot do while it
   is also being an access point.  Calling it there wedged the driver hard
   enough to take the main loop with it: no console, no DHCP, nothing but an
   SSID you could associate with.  So the list is gathered BEFORE the AP goes
   up, while the interface is still a station, and a request to rescan while
   the AP is up returns the cached list instead. */
void net_scan_start(void)
{
  cyw43_wifi_scan_options_t opts;

  if (ap_up) return;

  memset(&opts, 0, sizeof(opts));
  scan_count = 0;
  cyw43_arch_lwip_begin();
  if (cyw43_wifi_scan(&cyw43_state, &opts, NULL, scan_cb) == 0) scan_running = true;
  cyw43_arch_lwip_end();
}

/* Collect a list before becoming an access point.  Bounded, because a scan
   that never finishes must not stop the device from being set up. */
static void scan_before_ap(void)
{
  absolute_time_t give_up;

  if (ap_up) return;
  net_scan_start();
  give_up = make_timeout_time_ms(4000);
  while (scan_running && absolute_time_diff_us(get_absolute_time(), give_up) > 0)
  {
    sleep_ms(100);
    (void)net_scan_busy();
  }
  scan_running = false;
}

bool net_scan_busy(void)
{
  if (scan_running && !cyw43_wifi_scan_active(&cyw43_state)) scan_running = false;
  return scan_running;
}

uint32_t net_scan_count(void) { return scan_count; }

bool net_scan_get(uint32_t i, const char **ssid, int16_t *rssi)
{
  if (i >= scan_count) return false;
  *ssid = scan_ssid[i];
  *rssi = scan_rssi[i];
  return true;
}

/* --- the provisioning access point -------------------------------------- */

const char *net_ap_ssid(void)
{
  if (ap_ssid[0] == '\0')
  {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    snprintf(ap_ssid, sizeof(ap_ssid), "Synchronizer-%02X%02X",
             id.id[6], id.id[7]);
  }
  return ap_ssid;
}

bool net_in_ap(void) { return ap_up; }

static void ap_start(void)
{
  if (ap_up) return;

  scan_before_ap();      /* must happen while we are still a station */

  IP4_ADDR(&ap_ip,   192, 168, 4, 1);
  IP4_ADDR(&ap_mask, 255, 255, 255, 0);

  cyw43_arch_enable_ap_mode(net_ap_ssid(), cfg.ap_pass[0] ? cfg.ap_pass : "synchronizer",
                            CYW43_AUTH_WPA2_AES_PSK);

  cyw43_arch_lwip_begin();
  netif_set_addr(&cyw43_state.netif[CYW43_ITF_AP], &ap_ip, &ap_mask, &ap_ip);
  cyw43_arch_lwip_end();

  dhcpserver_start(&cyw43_state.netif[CYW43_ITF_AP], &ap_ip, &ap_mask);
  dnsserver_start(&cyw43_state.netif[CYW43_ITF_AP], &ap_ip);

  power_save_off();
  set_default_route(CYW43_ITF_AP);
  ap_up = true;
  state = NET_AP;
  mdns_up(CYW43_ITF_AP, &mdns_on_ap);
  ap_retry_at  = make_timeout_time_ms(ap_retry_ms);
  ap_http_mark = httpd_requests();
}

static void ap_stop(void)
{
  if (!ap_up) return;
  mdns_down(CYW43_ITF_AP, &mdns_on_ap);
  dnsserver_stop();
  dhcpserver_stop();
  cyw43_arch_disable_ap_mode();
  ap_up = false;
  set_default_route(CYW43_ITF_STA);   /* or nothing can reach the internet */
}

void net_ap_force(bool on)
{
  pending    = on ? PEND_AP_ON : PEND_AP_OFF;
  pending_at = make_timeout_time_ms(400);      /* let any reply get out */
}

bool net_provision(const char *ssid, const char *pass)
{
  if (!ap_up) return false;              /* AP only - never from your LAN */
  if (!ssid || ssid[0] == '\0') return false;

  strncpy(cfg.ssid, ssid, CONFIG_SSID_LEN - 1); cfg.ssid[CONFIG_SSID_LEN - 1] = '\0';
  strncpy(cfg.pass, pass ? pass : "", CONFIG_PASS_LEN - 1); cfg.pass[CONFIG_PASS_LEN - 1] = '\0';

  /* Saving and switching networks happens in net_poll, a moment from now,
     so the browser gets its answer before the access point disappears. */
  pending    = PEND_PROVISION;
  pending_at = make_timeout_time_ms(1200);
  return true;
}

static void pending_run(void)
{
  pending_op op = pending;

  if (op == PEND_NONE) return;
  if (absolute_time_diff_us(get_absolute_time(), pending_at) > 0) return;
  pending = PEND_NONE;

  switch (op)
  {
    case PEND_PROVISION:
      config_save();
      ap_forced   = false;
      sta_fails   = 0;
      ap_retry_ms = AP_RETRY_MS;
      ap_stop();
      state       = NET_OFF;
      have_server = false;
      retry_at    = get_absolute_time();
      break;
    case PEND_AP_ON:
      ap_forced = true;
      mdns_down(CYW43_ITF_STA, &mdns_on_sta);
      ap_start();
      break;
    case PEND_AP_OFF:
      ap_forced   = false;
      ap_stop();
      state       = NET_OFF;
      sta_fails   = 0;
      ap_retry_ms = AP_RETRY_MS;
      retry_at  = get_absolute_time();
      break;
    default:
      break;
  }
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
  power_save_off();

  cyw43_arch_lwip_begin();
  pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  if (pcb) udp_recv(pcb, ntp_recv, NULL);
  cyw43_arch_lwip_end();

  state      = NET_OFF;
  next_poll  = get_absolute_time();
  retry_at   = get_absolute_time();
  ok_count = fail_count = 0;
  sta_fails  = 0;

  /* A board that has never been told a network comes up as its own. */
  if (cfg.ssid[0] == '\0') ap_start();
}

void net_reconnect(void)
{
  mdns_down(CYW43_ITF_STA, &mdns_on_sta);
  ap_stop();
  ap_forced   = false;
  sta_fails   = 0;
  ap_retry_ms = AP_RETRY_MS;
  state       = NET_OFF;
  retry_at    = get_absolute_time();
  have_server = false;
}

void net_request_sync(void) { force_sync = true; }

/* The setup AP is raised when the configured network cannot be joined.  A
   wrong key looks exactly like a router that has been rebooted, and in the
   second case the board used to sit as an access point for ever while the
   network it wanted came back without it - healthy on serial, absent from
   the LAN.  So put the radio back on the configured network periodically;
   if it still will not join, the state machine raises the AP again a few
   seconds later and we wait out another interval.

   Not while somebody is using the AP: re-associating takes the radio off
   the AP's channel, which would strand a phone midway through provisioning.
   A lease, or any HTTP request since the last look, means hands off. */
static void ap_retry_sta(void)
{
  if (ap_forced || cfg.ssid[0] == '\0') return;
  if (absolute_time_diff_us(get_absolute_time(), ap_retry_at) > 0) return;

  if (dhcpserver_leases() > 0 || httpd_requests() != ap_http_mark)
  {
    ap_http_mark = httpd_requests();
    ap_retry_at  = make_timeout_time_ms(ap_retry_ms);
    return;
  }

  /* Back off.  A network that is down for a minute deserves a prompt retry;
     one that has been down all afternoon does not deserve a radio cycling
     every seventy-five seconds, and each cycle re-registers the AP netif
     under a fresh lwIP number.  Reset to the short interval the moment we
     get back on, in net_poll(). */
  if (ap_retry_ms < AP_RETRY_MAX_MS)
  {
    ap_retry_ms *= 2u;
    if (ap_retry_ms > AP_RETRY_MAX_MS) ap_retry_ms = AP_RETRY_MAX_MS;
  }

  ap_stop();
  state     = NET_OFF;
  sta_fails = 0;
  retry_at  = get_absolute_time();
}

uint32_t net_ap_retry_s(void)
{
  int64_t us;
  if (!ap_up || ap_forced || cfg.ssid[0] == '\0') return 0;
  us = absolute_time_diff_us(get_absolute_time(), ap_retry_at);
  return (us <= 0) ? 1u : (uint32_t)(us / 1000000) + 1u;
}

void net_poll(void)
{
  if (state == NET_FAILED) return;

  pending_run();

  if (ap_up)
  {
    (void)net_scan_busy();          /* let a finished scan settle */
    ap_retry_sta();                 /* a rescue is not a destination */
    return;                         /* no NTP while we are the network */
  }

  if (cfg.ssid[0] == '\0') { ap_start(); return; }

  if (state == NET_OFF)
  {
    if (absolute_time_diff_us(get_absolute_time(), retry_at) > 0) return;

    /* Asynchronous on purpose.  The blocking form parks the main loop for
       up to twenty seconds, which stops the console, the web server and the
       discipline loop dead - and outlives the watchdog, so joining a network
       rebooted the board instead of joining it. */
    if (cyw43_arch_wifi_connect_async(cfg.ssid, cfg.pass, CYW43_AUTH_WPA2_AES_PSK))
    {
      if (++sta_fails >= STA_FAILS_TO_AP) { ap_start(); return; }
      retry_at = make_timeout_time_ms(5000);
      return;
    }
    state            = NET_CONNECTING;
    connect_deadline = make_timeout_time_ms(CONNECT_TIMEOUT_MS);
    return;
  }

  if (state == NET_CONNECTING)
  {
    int link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

    if (link == CYW43_LINK_UP)
    {
      sta_fails   = 0;
      ap_retry_ms = AP_RETRY_MS;      /* the network is back; be eager again */
      state       = NET_ONLINE;
      next_poll   = get_absolute_time();
      power_save_off();
      set_default_route(CYW43_ITF_STA);
      mdns_up(CYW43_ITF_STA, &mdns_on_sta);
      return;
    }

    /* A negative status is a definite refusal - wrong key, no such network -
       so there is no point waiting out the timeout for it. */
    if (link < 0 || absolute_time_diff_us(get_absolute_time(), connect_deadline) <= 0)
    {
      state = NET_OFF;
      if (++sta_fails >= STA_FAILS_TO_AP) { ap_start(); return; }
      retry_at = make_timeout_time_ms(5000);
    }
    return;
  }

  if (state != NET_ONLINE) return;

  if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP)
  {
    mdns_down(CYW43_ITF_STA, &mdns_on_sta);
    state    = NET_OFF;
    retry_at = make_timeout_time_ms(5000);
    return;
  }

  /* DHCP may not have finished when the link first came up. */
  if (!mdns_on_sta) mdns_up(CYW43_ITF_STA, &mdns_on_sta);

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
    case NET_AP:         return "setup ap";
    case NET_FAILED:     return "failed";
  }
  return "?";
}

const char *net_ip(void)
{
  /* netif_list is whichever interface was registered last, not the one in
     use - ask for the station explicitly. */
  if (ap_up) return "192.168.4.1";
  if (state != NET_ONLINE) return "-";
  ip4addr_ntoa_r(netif_ip4_addr(itf(CYW43_ITF_STA)), ipbuf, sizeof(ipbuf));
  return ipbuf;
}

uint32_t net_ntp_ok(void)       { return ok_count; }
uint32_t net_ntp_fail(void)     { return fail_count; }
uint32_t net_last_rtt_us(void)  { return last_rtt; }
