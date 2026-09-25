/* webui.c - JSON API and request routing for the web interface */

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
#include <stdlib.h>
#include "pico/stdlib.h"
#include "board.h"
#include "config.h"
#include "sense.h"
#include "drive.h"
#include "timebase.h"
#include "netclock.h"
#include "control.h"
#include "chime.h"
#include "cli.h"
#include "httpd.h"
#include "webui.h"

typedef enum { JOB_NONE = 0, JOB_RESONANCE, JOB_SAVE, JOB_CLI } web_job;

/* A console command can print a lot - CAPTURE and SWEEP both run to a few
   kilobytes - and it runs from the main loop, never from an HTTP handler,
   because RESONANCE alone blocks for three seconds. */
#define CLI_OUT_MAX   4000u
#define CLI_CMD_MAX   128u

static char     cli_cmd[CLI_CMD_MAX];
static char     cli_out[CLI_OUT_MAX];
static uint32_t cli_out_len;
static uint32_t cli_serial;          /* bumped when a result is ready */

static volatile web_job job;
static uint32_t job_lo, job_hi;
static char     job_msg[80];

void web_init(void) { job = JOB_NONE; strcpy(job_msg, "idle"); }
bool web_busy(void) { return job != JOB_NONE; }

const char *web_job_name(void)
{
  switch (job)
  {
    case JOB_RESONANCE: return "resonance";
    case JOB_SAVE:      return "save";
    case JOB_CLI:       return "console";
    default:            return "idle";
  }
}

/* Runs from the main loop, where blocking is allowed. */
void web_poll(void)
{
  web_job j = job;

  if (j == JOB_NONE) return;

  if (j == JOB_RESONANCE)
  {
    sense_resonance r;
    sense_find_resonance(job_lo, job_hi, false, &r);
    if (r.saturated)
      snprintf(job_msg, sizeof(job_msg), "clipped at %u counts - not adopted", r.peak_adc);
    else if (!r.valid)
      snprintf(job_msg, sizeof(job_msg), "no clear peak - not adopted");
    else
      snprintf(job_msg, sizeof(job_msg), "peak %lu hz, Q %lu.%lu",
               (unsigned long)r.f0_hz, (unsigned long)(r.q_x10 / 10u),
               (unsigned long)(r.q_x10 % 10u));
  }
  else if (j == JOB_SAVE)
  {
    snprintf(job_msg, sizeof(job_msg), "%s", config_save() ? "saved" : "SAVE FAILED");
  }
  else if (j == JOB_CLI)
  {
    cli_run_captured(cli_cmd, cli_out, sizeof(cli_out), &cli_out_len);
    cli_serial++;
    snprintf(job_msg, sizeof(job_msg), "ran %s", cli_cmd);
  }

  job = JOB_NONE;
}

/* --- JSON ------------------------------------------------------------- */

/* Escape a string for use inside JSON quotes.  Everything that reaches the
   status object this way is arbitrary text - a job message quoting the
   command it ran, an SSID someone else chose, a hostname - and a single
   quote or backslash in any of them produces a document the page cannot
   parse, which breaks the whole interface rather than one field.
   Control characters and non-ASCII are dropped rather than \u-escaped;
   this is a status page, not a transport. */
static void json_esc(char *out, uint32_t outsz, const char *s)
{
  uint32_t u = 0;
  if (outsz == 0u) return;
  while (*s && u + 3u < outsz)
  {
    unsigned char ch = (unsigned char)*s++;
    if (ch == '"' || ch == '\\') { out[u++] = '\\'; out[u++] = (char)ch; }
    else if (ch >= 0x20u && ch < 0x7Fu) out[u++] = (char)ch;
  }
  out[u] = '\0';
}

static uint32_t json_status(char *b, uint32_t n)
{
  control_stats cs;
  const sense_resonance *r = sense_last_resonance();
  uint64_t mean = sense_mean_interval_us(64);
  char e_job[sizeof(job_msg) * 2], e_ssid[CONFIG_SSID_LEN * 2];
  char e_ap[70], e_host[CONFIG_NAME_LEN * 2], e_ntp[CONFIG_HOST_LEN * 2];
  int64_t  nom  = (int64_t)cfg_event_interval_ns();
  int64_t  ppb  = 0, sday = 0;
  uint32_t u = 0;

  control_stats_get(&cs);
  json_esc(e_job,  sizeof(e_job),  job_msg);
  json_esc(e_ssid, sizeof(e_ssid), cfg.ssid);
  json_esc(e_ap,   sizeof(e_ap),   net_ap_ssid());
  json_esc(e_host, sizeof(e_host), net_hostname());
  json_esc(e_ntp,  sizeof(e_ntp),  cfg.ntp_host);
  if (mean) { int64_t got = (int64_t)mean * 1000ll;
              ppb = ((got - nom) * 1000000000ll) / nom;
              sday = ((got - nom) * -86400ll) / nom; }

  u += (uint32_t)snprintf(b + u, n - u,
    "{\"job\":\"%s\",\"jobmsg\":\"%s\","
    "\"clock\":{\"bph\":%lu,\"bps\":%u,\"epp\":%u,\"ppt\":%u,"
      "\"period_ns\":%llu,\"interval_ns\":%llu,"
      "\"mean_us\":%llu,\"ppb\":%lld,\"sday\":%lld},",
    web_job_name(), e_job,
    (unsigned long)cfg.beats_per_hour, cfg.beats_per_period,
    cfg.events_per_period, cfg.drive_offset_ppt,
    (unsigned long long)cfg_period_ns(), (unsigned long long)cfg_event_interval_ns(),
    (unsigned long long)mean, (long long)ppb, (long long)sday);

  u += (uint32_t)snprintf(b + u, n - u,
    "\"sense\":{\"on\":%d,\"tank\":%lu,\"duty\":%lu,\"f0\":%lu,\"q10\":%lu,"
      "\"peak\":%u,\"floor\":%u,\"base\":%u,\"last\":%u,\"thresh\":%u,"
      "\"falling\":%d,\"events\":%lu,\"dropped\":%lu,"
      "\"rearm\":%lu,\"minb\":%lu,\"maxb\":%lu,"
      "\"pct\":[%u,%u,%u],\"acq\":[%u,%u]},",
    sense_enabled() ? 1 : 0, (unsigned long)sense_tank_hz(),
    (unsigned long)sense_drive_ns(),
    (unsigned long)cfg.tank_f0_hz, (unsigned long)cfg.tank_q_x10,
    cfg.tank_peak_adc, cfg.tank_floor_adc,
    sense_baseline(), sense_last_sample(), cfg.detect_threshold,
    cfg.detect_falling ? 1 : 0,
    (unsigned long)sense_event_count(), (unsigned long)sense_overrun_count(),
    (unsigned long)sense_rearm_us(), (unsigned long)sense_min_event_us(),
    (unsigned long)sense_max_event_us(),
    cfg.rearm_pct, cfg.min_event_pct, cfg.max_event_pct,
    cfg.acquire_events, cfg.acquire_tol_pct);

  u += (uint32_t)snprintf(b + u, n - u,
    "\"drive\":{\"on\":%d,\"pw\":%u,\"adv\":%ld,\"ret\":%ld,"
      "\"pulses\":%lu,\"refused\":%lu,\"budget\":%lu},",
    drive_is_on() ? 1 : 0, cfg.pulse_us, (long)cfg.pulse_advance_us,
    (long)cfg.pulse_retard_us,
    (unsigned long)drive_pulse_count(), (unsigned long)drive_refused_count(),
    (unsigned long)drive_budget_us());

  u += (uint32_t)snprintf(b + u, n - u,
    "\"time\":{\"net\":\"%s\",\"ip\":\"%s\",\"ok\":%lu,\"fail\":%lu,"
      "\"rtt\":%lu,\"have\":%d,\"unix\":%llu,\"ppb\":%ld,\"fixes\":%lu,"
      "\"offset_us\":%lld,\"tz\":%ld},",
    net_status_name(), net_ip(), (unsigned long)net_ntp_ok(),
    (unsigned long)net_ntp_fail(), (unsigned long)net_last_rtt_us(),
    tb_have_time() ? 1 : 0, (unsigned long long)(tb_utc_ns() / 1000000000ull),
    (long)tb_ppb(), (unsigned long)tb_fix_count(),
    (long long)(tb_last_offset_ns() / 1000), (long)(cfg.tz_offset_s / 60));

  u += (uint32_t)snprintf(b + u, n - u,
    "\"loop\":{\"state\":\"%s\",\"on\":%d,\"act\":%d,"
      "\"kickact\":%d,\"kicksince\":%lu,\"kickn\":%u,\"kickdir\":%d,"
      "\"kickthr\":%u,"
      "\"kickfilt_us\":%lld,\"kickaccum_us\":%lld,\"diff_us\":%lld,"
      "\"schederr_us\":%lld,"
      "\"events\":%llu,\"missed\":%lu,"
      "\"err_us\":%lld,\"filt_us\":%lld,\"cmd_ns\":%lld,"
      "\"drift_ppb\":%lld,\"offset_ms\":%lld,"
      "\"kp\":%lu,\"ki\":%lu,\"slew\":%ld},",
    control_state_name(cs.state), cfg.control_enabled ? 1 : 0,
    control_actuator() ? 1 : 0,
    cs.kick_active ? 1 : 0, (unsigned long)cs.kick_since,
    (unsigned)(cfg.kick_min_swings ? cfg.kick_min_swings : 5u), cs.kick_dir_retard,
    (unsigned)(cfg.kick_threshold_pct ? cfg.kick_threshold_pct : 25u),
    (long long)(cs.kick_filt_ns / 1000), (long long)(cs.kick_accum_ns / 1000),
    (long long)((-cs.filt_err_ns - cs.kick_filt_ns) / 1000),
    (long long)(cs.sched_err_ns / 1000),
    (unsigned long long)cs.events, (unsigned long)cs.missed,
    (long long)(cs.err_ns / 1000), (long long)(cs.filt_err_ns / 1000),
    (long long)cs.cmd_ns_per_swing,
    (long long)cs.drift_ppb, (long long)(cs.target_offset_ns / 1000000),
    (unsigned long)cfg.kp_swings, (unsigned long)cfg.ki_swings,
    (long)cfg.slew_limit_ppm);

  {
    uint32_t away = 0, face = 0, real = 0;
    bool nxt = chime_next(&away, &face, &real);
    u += (uint32_t)snprintf(b + u, n - u,
      "\"chime\":{\"have\":%d,\"offset_ms\":%ld,\"interval\":%lu,"
        "\"latency\":%lu,\"next\":%d,\"away\":%lu,\"face_sod\":%lu,"
        "\"true_sod\":%lu,\"sod\":%lu},",
      chime_have() ? 1 : 0, (long)chime_offset_ms(),
      (unsigned long)cfg.chime_interval_min, (unsigned long)cfg.chime_latency_ms,
      nxt ? 1 : 0, (unsigned long)away, (unsigned long)face,
      (unsigned long)real, (unsigned long)chime_local_sod());
  }

  u += (uint32_t)snprintf(b + u, n - u,
    "\"res\":{\"valid\":%d,\"sat\":%d,\"f0\":%lu,\"lo\":%lu,\"hi\":%lu,"
      "\"q10\":%lu,\"peak\":%u,\"floor\":%u},"
    "\"ssid\":\"%s\",\"ntp\":\"%s\",\"ap\":%d,\"apssid\":\"%s\","
    "\"host\":\"%s\",\"mdns\":%d,\"cliseq\":%lu,\"web\":\"%s\"}",
    r->valid ? 1 : 0, r->saturated ? 1 : 0, (unsigned long)r->f0_hz,
    (unsigned long)r->f_lo_hz, (unsigned long)r->f_hi_hz,
    (unsigned long)r->q_x10, r->peak_adc, r->floor_adc,
    e_ssid, e_ntp, net_in_ap() ? 1 : 0, e_ap,
    e_host, net_mdns_active() ? 1 : 0, (unsigned long)cli_serial, web_version);

  return (u < n) ? u : (n - 1u);
}

static uint32_t json_wifi(char *b, uint32_t n)
{
  uint32_t i, cnt = net_scan_count(), u = 0;
  const char *ss; int16_t rssi;

  u += (uint32_t)snprintf(b + u, n - u, "{\"busy\":%s,\"n\":%lu,\"ssid\":[",
                          net_scan_busy() ? "true" : "false", (unsigned long)cnt);
  for (i = 0; i < cnt && u < n - 64u; i++)
  {
    char esc[70];
    net_scan_get(i, &ss, &rssi);
    json_esc(esc, sizeof(esc), ss);      /* SSIDs are arbitrary bytes */
    u += (uint32_t)snprintf(b + u, n - u, "%s\"%s\"", i ? "," : "", esc);
  }
  u += (uint32_t)snprintf(b + u, n - u, "],\"rssi\":[");
  for (i = 0; i < cnt && u < n - 16u; i++)
  { net_scan_get(i, &ss, &rssi);
    u += (uint32_t)snprintf(b + u, n - u, "%s%d", i ? "," : "", rssi); }
  u += (uint32_t)snprintf(b + u, n - u, "]}");
  return (u < n) ? u : (n - 1u);
}

static uint32_t json_scan(char *b, uint32_t n)
{
  uint32_t i, cnt = sense_scan_count(), u = 0;
  uint32_t hz; uint16_t adc;

  u += (uint32_t)snprintf(b + u, n - u, "{\"n\":%lu,\"hz\":[", (unsigned long)cnt);
  for (i = 0; i < cnt && u < n - 32u; i++)
  { sense_scan_point(i, &hz, &adc);
    u += (uint32_t)snprintf(b + u, n - u, "%s%lu", i ? "," : "", (unsigned long)hz); }
  u += (uint32_t)snprintf(b + u, n - u, "],\"adc\":[");
  for (i = 0; i < cnt && u < n - 16u; i++)
  { sense_scan_point(i, &hz, &adc);
    u += (uint32_t)snprintf(b + u, n - u, "%s%u", i ? "," : "", adc); }
  u += (uint32_t)snprintf(b + u, n - u, "]}");
  return (u < n) ? u : (n - 1u);
}

/* --- mutations -------------------------------------------------------- */

static bool set_u32(const char *q, const char *k, uint32_t *dst, uint32_t lo, uint32_t hi)
{
  char v[24];
  long x;
  if (!http_query_get(q, k, v, sizeof(v))) return false;
  x = strtol(v, NULL, 10);
  if (x < (long)lo) x = (long)lo;
  if (x > (long)hi) x = (long)hi;
  *dst = (uint32_t)x;
  return true;
}

static bool set_i32(const char *q, const char *k, int32_t *dst, int32_t lo, int32_t hi)
{
  char v[24];
  long x;
  if (!http_query_get(q, k, v, sizeof(v))) return false;
  x = strtol(v, NULL, 10);
  if (x < (long)lo) x = (long)lo;
  if (x > (long)hi) x = (long)hi;
  *dst = (int32_t)x;
  return true;
}

static bool apply_config(const char *q)
{
  uint32_t t;
  int32_t  ti, half;
  bool touched = false, reclock = false;

  /* adv/ret can each reach a full half period from centre in either
     direction now - see the pulse_advance_us/pulse_retard_us comment in
     config.h - so the bound here scales with this clock's period. */
  half = (int32_t)(cfg_period_ns() / 1000ull / 2ull);

  if (set_u32(q, "bph",    &t, 60u, 200000u))  { cfg.beats_per_hour = t;        touched = reclock = true; }
  if (set_u32(q, "bps",    &t, 1u, 8u))        { cfg.beats_per_period = (uint8_t)t;  touched = reclock = true; }
  if (set_u32(q, "epp",    &t, 1u, 4u))        { cfg.events_per_period = (uint8_t)t; touched = reclock = true; }
  if (set_u32(q, "ppt",    &t, 0u, 1000u))     { cfg.drive_offset_ppt = (uint16_t)t; touched = true; }
  if (set_u32(q, "rearm",  &t, 1u, 90u))       { cfg.rearm_pct = (uint8_t)t;     touched = reclock = true; }
  if (set_u32(q, "minb",   &t, 1u, 50u))       { cfg.min_event_pct = (uint8_t)t; touched = reclock = true; }
  if (set_u32(q, "maxb",   &t, 5u, 95u))       { cfg.max_event_pct = (uint8_t)t; touched = reclock = true; }
  if (set_u32(q, "acqn",   &t, 2u, 1000u))     { cfg.acquire_events = (uint16_t)t;   touched = true; }
  if (set_u32(q, "acqtol", &t, 1u, 25u))       { cfg.acquire_tol_pct = (uint8_t)t;   touched = true; }
  if (set_u32(q, "tank",   &t, 100u, 500000u)) { sense_set_tank_hz(t);           touched = true; }
  if (set_u32(q, "thresh", &t, 1u, 4000u))     { cfg.detect_threshold = (uint16_t)t; touched = true; }
  if (set_u32(q, "falling",&t, 0u, 1u))        { cfg.detect_falling = (uint8_t)t;    touched = true; }
  if (set_u32(q, "pw",     &t, 0u, DRIVE_MAX_PULSE_US)) { cfg.pulse_us = t;                touched = true; }
  if (set_i32(q, "adv",    &ti, -half, half))  { cfg.pulse_advance_us = ti; touched = true; }
  if (set_i32(q, "ret",    &ti, -half, half))  { cfg.pulse_retard_us  = ti; touched = true; }
  if (set_u32(q, "kp",     &t, 1u, 1000000u))  { cfg.kp_swings = t;              touched = true; }
  if (set_u32(q, "ki",     &t, 1u, 1000000u))  { cfg.ki_swings = t;              touched = true; }
  if (set_u32(q, "slew",   &t, 1u, 100000u))   { cfg.slew_limit_ppm = (int32_t)t;    touched = true; }
  if (set_u32(q, "kickn",  &t, 1u, 2000u))     { cfg.kick_min_swings = (uint16_t)t; touched = true; }
  if (set_u32(q, "kickthr",&t, 1u, 49u))       { cfg.kick_threshold_pct = (uint16_t)t; touched = true; }
  if (set_u32(q, "cint",   &t, 1u, 720u))      { cfg.chime_interval_min = t;      touched = true; }
  {
    char v[16];
    if (http_query_get(q, "tz", v, sizeof(v)))
    { long mins = strtol(v, NULL, 10);
      if (mins >= -840 && mins <= 840) { cfg.tz_offset_s = (int32_t)(mins * 60); touched = true; } }
  }
  if (set_u32(q, "clat",   &t, 0u, 5000u))     { cfg.chime_latency_ms = t;        touched = true; }
  {
    char v[24];
    if (http_query_get(q, "duty", v, sizeof(v)))
    { sense_set_drive_ns((uint32_t)strtol(v, NULL, 10)); touched = true; }
  }
  {
    char h[CONFIG_NAME_LEN];
    if (http_query_get(q, "host", h, sizeof(h)) && net_set_hostname(h)) touched = true;
  }

  if (reclock) { sense_refresh_timing(); control_init(); }
  return touched;
}

/* --- routing ----------------------------------------------------------- */

static void reply(http_response *o, int st, const char *ct, const char *b, uint32_t n)
{ o->status = st; o->ctype = ct; o->body = b; o->len = n; }

/* For constant bodies.  Hand-counting Content-Length is a bug waiting to
   happen: one byte short truncates the body and the client waits forever
   for the rest, or parses invalid JSON.  Two of these were already wrong. */
static void reply_lit(http_response *o, int st, const char *ct, const char *b)
{ o->status = st; o->ctype = ct; o->body = b; o->len = (uint32_t)strlen(b); }

void http_dispatch(const char *method, const char *path, const char *query,
                   char *scratch, uint32_t scratch_len, http_response *out)
{
  bool post = (strcmp(method, "POST") == 0);

  if (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0)
  {
    if (net_in_ap())
      reply(out, 200, "text/html; charset=utf-8", web_setup, web_setup_len);
    else
      reply(out, 200, "text/html; charset=utf-8", web_page, web_page_len);
    return;
  }

  /* The console ring, from a cursor.  First line is the cursor to pass
     back next time; everything after it is what has been printed since.
     This is how a measurement's result reaches a browser at all - it is
     printed from the control loop, long after the command returned. */
  if (strcmp(path, "/api/log") == 0)
  {
    uint32_t from = (uint32_t)http_query_int(query, "from", 0);
    uint32_t next = 0u, n, hdr;
    hdr = (uint32_t)snprintf(scratch, scratch_len, "%lu\n",
                             (unsigned long)cli_log_seq());
    n = cli_log_read(from, scratch + hdr, scratch_len - hdr - 1u, &next);
    reply(out, 200, "text/plain; charset=utf-8", scratch, hdr + n);
    return;
  }

  if (strcmp(path, "/api/cli") == 0)
  {
    if (!post)
    {
      /* Plain text, so this is pleasant from curl as well as the page. */
      uint32_t n = cli_out_len;
      if (n > scratch_len - 1u) n = scratch_len - 1u;
      memcpy(scratch, cli_out, n);
      scratch[n] = '\0';
      reply(out, 200, "text/plain; charset=utf-8", scratch, n);
      return;
    }

    if (job != JOB_NONE)
    { reply_lit(out, 503, "text/plain",
            "busy\r\n"); return; }
    if (!http_query_get(query, "cmd", cli_cmd, sizeof(cli_cmd)) || cli_cmd[0] == '\0')
    { reply_lit(out, 400, "text/plain",
            "no command\r\n"); return; }

    /* Wi-Fi credentials are deliberately settable only over the device's
       own access point.  Letting the console set them from your LAN would
       quietly undo that, so those two commands are refused here - they
       still work on the serial console. */
    {
      static const char *const denied[] = { "WIFI", "APKEY" };
      uint32_t i;
      for (i = 0; i < sizeof(denied) / sizeof(denied[0]); i++)
      {
        uint32_t l = (uint32_t)strlen(denied[i]);
        if (strncasecmp(cli_cmd, denied[i], l) == 0 &&
            (cli_cmd[l] == '\0' || cli_cmd[l] == ' '))
        {
          reply_lit(out, 403, "text/plain",
            "refused: set credentials on the serial console, or over the "
                "setup access point\r\n");
          return;
        }
      }
    }

    snprintf(job_msg, sizeof(job_msg), "running %s", cli_cmd);
    job = JOB_CLI;
    reply_lit(out, 202, "text/plain",
            "queued\r\n");
    return;
  }

  if (strcmp(path, "/api/scanwifi") == 0)
  {
    reply(out, 200, "application/json", scratch, json_wifi(scratch, scratch_len));
    return;
  }

  if (post && strcmp(path, "/api/wifi") == 0)
  {
    char ssid[CONFIG_SSID_LEN], pass[CONFIG_PASS_LEN];
    if (!http_query_get(query, "ssid", ssid, sizeof(ssid)))
    { reply_lit(out, 400, "application/json",
            "{\"ok\":false}"); return; }
    if (!http_query_get(query, "pass", pass, sizeof(pass))) pass[0] = '\0';
    /* net_provision refuses unless the AP is up, so credentials can only
       be set from the device's own network, never from your LAN. */
    if (net_provision(ssid, pass))
      reply_lit(out, 200, "application/json",
            "{\"ok\":true}");
    else
      reply_lit(out, 403, "application/json",
            "{\"ok\":false}");
    return;
  }

  if (strcmp(path, "/api/status") == 0)
  {
    reply(out, 200, "application/json", scratch, json_status(scratch, scratch_len));
    return;
  }

  if (strcmp(path, "/api/scan") == 0)
  {
    reply(out, 200, "application/json", scratch, json_scan(scratch, scratch_len));
    return;
  }

  if (post && strcmp(path, "/api/chime") == 0)
  {
    long h = http_query_int(query, "h", -1);
    long m = http_query_int(query, "m", 0);

    if (h < 0 || h > 23 || m < 0 || m > 59)
    { reply_lit(out, 400, "application/json", "{\"ok\":false,\"err\":\"bad time\"}"); return; }

    if (!chime_mark((uint32_t)h, (uint32_t)m))
    { reply_lit(out, 503, "application/json",
                "{\"ok\":false,\"err\":\"no time yet\"}"); return; }

    if (http_query_int(query, "apply", 0)) chime_apply_to_loop();
    reply_lit(out, 200, "application/json", "{\"ok\":true}");
    return;
  }

  if (post && strcmp(path, "/api/config") == 0)
  {
    bool ok = apply_config(query);
    reply(out, 200, "application/json",
          ok ? "{\"ok\":true}" : "{\"ok\":false}", ok ? 11u : 12u);
    return;
  }

  if (post && strcmp(path, "/api/action") == 0)
  {
    char act[24];
    if (!http_query_get(query, "do", act, sizeof(act)))
    { reply_lit(out, 400, "application/json",
            "{\"err\":\"no action\"}"); return; }

    if (strcmp(act, "control") == 0)
      control_enable(http_query_int(query, "on", 0) != 0);
    else if (strcmp(act, "sense") == 0)
    { cfg.sense_enabled = http_query_int(query, "on", 1) ? 1u : 0u;
      sense_enable(cfg.sense_enabled != 0u); }
    else if (strcmp(act, "pulse") == 0)
      drive_pulse((uint32_t)http_query_int(query, "us", cfg.pulse_us));
    else if (strcmp(act, "coiloff") == 0)
      drive_all_off();
    else if (strcmp(act, "offset") == 0)
      control_set_offset_ns((int64_t)http_query_int(query, "ms", 0) * 1000000ll);
    else if (strcmp(act, "sync") == 0)
      net_request_sync();
    else if (strcmp(act, "scanwifi") == 0)
      net_scan_start();
    else if (strcmp(act, "ap") == 0)
      net_ap_force(http_query_int(query, "on", 1) != 0);
    else if (strcmp(act, "resetloop") == 0)
      control_reset();
    else if (strcmp(act, "chimeapply") == 0)
      chime_apply_to_loop();
    else if (strcmp(act, "chimeforget") == 0)
      chime_forget();
    else if (strcmp(act, "resonance") == 0)
    {
      if (job != JOB_NONE)
      { reply_lit(out, 503, "application/json",
            "{\"err\":\"busy\"}"); return; }
      job_lo = (uint32_t)http_query_int(query, "lo", 20000);
      job_hi = (uint32_t)http_query_int(query, "hi", 120000);
      snprintf(job_msg, sizeof(job_msg), "scanning %lu-%lu hz",
               (unsigned long)job_lo, (unsigned long)job_hi);
      job = JOB_RESONANCE;
      reply_lit(out, 202, "application/json",
            "{\"ok\":true}");
      return;
    }
    else if (strcmp(act, "save") == 0)
    {
      if (job != JOB_NONE)
      { reply_lit(out, 503, "application/json",
            "{\"err\":\"busy\"}"); return; }
      snprintf(job_msg, sizeof(job_msg), "writing flash");
      job = JOB_SAVE;
      reply_lit(out, 202, "application/json",
            "{\"ok\":true}");
      return;
    }
    else
    { reply_lit(out, 400, "application/json",
            "{\"err\":\"unknown\"}"); return; }

    reply_lit(out, 200, "application/json",
            "{\"ok\":true}");
    return;
  }

  /* A phone decides a network needs signing in to by fetching a known URL
     and checking what comes back.  While the AP is up, answer every one of
     them with the setup page so that check fails and the portal opens,
     instead of leaving someone to guess an address. */
  if (net_in_ap() && !post)
  {
    reply(out, 200, "text/html; charset=utf-8", web_setup, web_setup_len);
    return;
  }

  reply_lit(out, 404, "text/plain",
            "not found");
}
