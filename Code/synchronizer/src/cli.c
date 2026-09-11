/* cli.c - the serial command line, over USB */

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
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include "board.h"
#include "config.h"
#include "sense.h"
#include "drive.h"
#include "timebase.h"
#include "netclock.h"
#include "control.h"
#include "tinycl.h"
#include "cli.h"

static void print_ns(const char *label, int64_t ns)
{
  int64_t us = ns / 1000;
  printf("%-22s %lld us\r\n", label, (long long)us);
}

static int status_cmd(int args, tinycl_parameter *tp, void *v)
{
  control_stats cs;
  uint64_t utc;
  uint64_t mean;

  (void)args; (void)tp; (void)v;
  control_stats_get(&cs);

  printf("\r\n-- clock ------------------------------------------------\r\n");
  printf("%-22s %lu bph, %u beats per swing\r\n", "gear ratio",
         (unsigned long)cfg.beats_per_hour, cfg.beats_per_period);
  printf("%-22s %llu ns  (%lu swings/hour)\r\n", "nominal swing",
         (unsigned long long)cfg_period_ns(),
         (unsigned long)(cfg.beats_per_hour / (cfg.beats_per_period ? cfg.beats_per_period : 2u)));
  printf("%-22s %u per swing, every %llu ns\r\n", "sense events",
         cfg.events_per_period, (unsigned long long)cfg_event_interval_ns());
  printf("%-22s %u ppt of a period after a sense event\r\n", "drive coil",
         cfg.drive_offset_ppt);
  printf("%-22s rearm %lu us, bump %lu..%lu us\r\n", "detector windows",
         (unsigned long)sense_rearm_us(), (unsigned long)sense_min_event_us(),
         (unsigned long)sense_max_event_us());
  mean = sense_mean_interval_us(64);
  if (mean)
  {
    int64_t nom = (int64_t)cfg_event_interval_ns();
    int64_t got = (int64_t)mean * 1000ll;
    printf("%-22s %llu us  (%+lld ppb, %+lld s/day)\r\n", "measured interval",
           (unsigned long long)mean,
           (long long)(((got - nom) * 1000000000ll) / nom),
           (long long)(((got - nom) * -86400ll) / nom));
  }
  else
    printf("%-22s (no events yet)\r\n", "measured interval");

  printf("\r\n-- sense ------------------------------------------------\r\n");
  printf("%-22s %s\r\n", "state", sense_enabled() ? "running" : "stopped");
  printf("%-22s %lu hz\r\n", "tank drive", (unsigned long)sense_tank_hz());
  if (cfg.tank_f0_hz)
    printf("%-22s %lu hz, Q %lu.%lu, peak %u over %u  (drive is %+ld hz off)\r\n",
           "tank resonance", (unsigned long)cfg.tank_f0_hz,
           (unsigned long)(cfg.tank_q_x10 / 10u), (unsigned long)(cfg.tank_q_x10 % 10u),
           cfg.tank_peak_adc, cfg.tank_floor_adc,
           (long)((int32_t)sense_tank_hz() - (int32_t)cfg.tank_f0_hz));
  else
    printf("%-22s not calibrated - run RESONANCE\r\n", "tank resonance");
  printf("%-22s %u (last sample %u, threshold %u, %s)\r\n", "baseline",
         sense_baseline(), sense_last_sample(), cfg.detect_threshold,
         cfg.detect_falling ? "falling" : "rising");
  printf("%-22s %lu  (%lu dropped)\r\n", "events",
         (unsigned long)sense_event_count(), (unsigned long)sense_overrun_count());

  printf("\r\n-- drive ------------------------------------------------\r\n");
  printf("%-22s %s\r\n", "coil", drive_is_on() ? "ON" : "off");
  printf("%-22s %u us wide, %u us advance, %u us retard\r\n", "pulse",
         cfg.pulse_us, cfg.pulse_advance_us, cfg.pulse_retard_us);
  printf("%-22s %ld ns per pulse%s\r\n", "authority",
         (long)cfg.pulse_authority_ns,
         cfg.pulse_authority_ns ? "" : "   (not measured - loop cannot act)");
  printf("%-22s %lu fired, %lu refused, %lu us budget\r\n", "pulses",
         (unsigned long)drive_pulse_count(), (unsigned long)drive_refused_count(),
         (unsigned long)drive_budget_us());

  printf("\r\n-- time -------------------------------------------------\r\n");
  printf("%-22s %s  %s\r\n", "network", net_status_name(), net_ip());
  printf("%-22s %lu ok, %lu failed, last rtt %lu us\r\n", "ntp",
         (unsigned long)net_ntp_ok(), (unsigned long)net_ntp_fail(),
         (unsigned long)net_last_rtt_us());
  utc = tb_utc_ns();
  if (tb_have_time())
  {
    uint64_t s = utc / 1000000000ull + (uint64_t)(int64_t)cfg.tz_offset_s;
    printf("%-22s %llu  (%02llu:%02llu:%02llu)\r\n", "unix time",
           (unsigned long long)(utc / 1000000000ull),
           (unsigned long long)((s / 3600ull) % 24ull),
           (unsigned long long)((s / 60ull) % 60ull),
           (unsigned long long)(s % 60ull));
    printf("%-22s %ld ppb, %lu fixes, last offset %lld us\r\n", "timebase",
           (long)tb_ppb(), (unsigned long)tb_fix_count(),
           (long long)(tb_last_offset_ns() / 1000));
  }
  else
    printf("%-22s not set\r\n", "unix time");

  printf("\r\n-- loop -------------------------------------------------\r\n");
  printf("%-22s %s\r\n", "state", control_state_name(cs.state));
  printf("%-22s %llu  (%lu missed)\r\n", "events", (unsigned long long)cs.events,
         (unsigned long)cs.missed);
  print_ns("phase error", cs.err_ns);
  print_ns("phase error (filtered)", cs.filt_err_ns);
  print_ns("command per swing", cs.cmd_ns_per_swing);
  print_ns("undelivered credit", cs.credit_ns);
  print_ns("hand offset target", cs.target_offset_ns);
  printf("%-22s %lld ppb\r\n", "pendulum drift", (long long)cs.drift_ppb);
  printf("%-22s kp %lu events, ki %lu events, slew %ld ppm\r\n", "gains",
         (unsigned long)cfg.kp_swings, (unsigned long)cfg.ki_swings,
         (long)cfg.slew_limit_ppm);
  printf("\r\n");
  return 1;
}

static int sweep_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_sweep((uint32_t)tp[0].ti.i, (uint32_t)tp[1].ti.i,
              (uint32_t)tp[2].ti.i, 25u);
  return 1;
}

static int capture_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_capture((uint32_t)tp[0].ti.i, (uint32_t)tp[1].ti.i);
  return 1;
}

static int resonance_cmd(int args, tinycl_parameter *tp, void *v)
{
  sense_resonance r;
  uint32_t lo = (uint32_t)tp[0].ti.i, hi = (uint32_t)tp[1].ti.i;

  (void)args; (void)v;
  if (lo == 0u) lo = 2000u;
  if (hi == 0u) hi = 80000u;

  printf("keep metal away from the sense coil - the bob, your hand, tools\r\n");
  sense_find_resonance(lo, hi, tp[2].tb.b, &r);

  printf("\r\npeak            %lu hz\r\n", (unsigned long)r.f0_hz);
  printf("amplitude       %u counts (floor %u, rise %u)\r\n",
         r.peak_adc, r.floor_adc, (unsigned)(r.peak_adc - r.floor_adc));
  if (r.f_lo_hz && r.f_hi_hz)
  {
    printf("-3 dB points    %lu .. %lu hz, bandwidth %lu hz\r\n",
           (unsigned long)r.f_lo_hz, (unsigned long)r.f_hi_hz,
           (unsigned long)(r.f_hi_hz - r.f_lo_hz));
    printf("Q               %lu.%lu\r\n",
           (unsigned long)(r.q_x10 / 10u), (unsigned long)(r.q_x10 % 10u));
    printf("steepest flanks %lu and %lu hz  (try these if the bob barely moves\r\n"
           "                the amplitude at the peak)\r\n",
           (unsigned long)(r.f0_hz - ((r.f_hi_hz - r.f_lo_hz) * 354u) / 1000u),
           (unsigned long)(r.f0_hz + ((r.f_hi_hz - r.f_lo_hz) * 354u) / 1000u));
  }
  else
    printf("-3 dB points    not found inside the scan - widen the range\r\n");

  if (r.saturated)
    printf("WARNING: the envelope railed at %u counts.  The peak is clipped and\r\n"
           "         both the frequency and Q are unreliable.  Reduce the drive\r\n"
           "         or the amplifier gain and scan again.\r\n", r.peak_adc);
  if (r.edge)
    printf("WARNING: the peak sat at the edge of the scan - widen the range.\r\n");
  if (!r.valid)
    printf("WARNING: no clear peak.  Check the coil is on J3 and C3 is fitted.\r\n");

  if (r.valid && !r.saturated)
    printf("\r\ndrive now at %lu hz - 'save' to keep it\r\n",
           (unsigned long)sense_tank_hz());
  else
    printf("\r\nresult not adopted; drive left at %lu hz\r\n",
           (unsigned long)sense_tank_hz());
  return 1;
}

static int tank_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_set_tank_hz((uint32_t)tp[0].ti.i);
  printf("tank %lu hz\r\n", (unsigned long)sense_tank_hz());
  return 1;
}

static int sense_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.sense_enabled = tp[0].tb.b ? 1u : 0u;
  sense_enable(tp[0].tb.b);
  printf("sense %s\r\n", sense_enabled() ? "on" : "off");
  return 1;
}

static int thresh_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.detect_threshold = (uint16_t)tp[0].ti.i;
  printf("threshold %u counts\r\n", cfg.detect_threshold);
  return 1;
}

static int dir_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.detect_falling = tp[0].tb.b ? 1u : 0u;
  printf("bob makes amplitude %s\r\n", cfg.detect_falling ? "fall" : "rise");
  return 1;
}

static int watch_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  control_set_echo(tp[0].tb.b);
  printf("event echo %s\r\n", control_echo() ? "on" : "off");
  return 1;
}

static int pulse_cmd(int args, tinycl_parameter *tp, void *v)
{
  uint32_t us = (uint32_t)tp[0].ti.i;
  (void)args; (void)v;
  if (us == 0u) us = cfg.pulse_us;
  printf("%s\r\n", drive_pulse(us) ? "fired" : "REFUSED (too wide, or duty budget spent)");
  return 1;
}

static int coiloff_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  drive_all_off();
  printf("coil off\r\n");
  return 1;
}

static int pw_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.pulse_us = (uint16_t)tp[0].ti.i;
  if (cfg.pulse_us > DRIVE_MAX_PULSE_US) cfg.pulse_us = DRIVE_MAX_PULSE_US;
  printf("pulse width %u us\r\n", cfg.pulse_us);
  return 1;
}

static int ptime_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.pulse_advance_us = (uint16_t)tp[0].ti.i;
  cfg.pulse_retard_us  = (uint16_t)tp[1].ti.i;
  printf("advance %u us before arrival, retard %u us after\r\n",
         cfg.pulse_advance_us, cfg.pulse_retard_us);
  return 1;
}

static int auth_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.pulse_authority_ns = (int32_t)tp[0].ti.i;
  printf("authority %ld ns per pulse\r\n", (long)cfg.pulse_authority_ns);
  return 1;
}

static int measure_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  if (!control_measure_authority((uint32_t)tp[0].ti.i, tp[1].tb.b))
    printf("needs the loop tracking first, and 1..500 swings\r\n");
  return 1;
}

static int control_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  control_enable(tp[0].tb.b);
  printf("control %s\r\n", cfg.control_enabled ? "on" : "off");
  return 1;
}

static int offset_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  control_set_offset_ns((int64_t)tp[0].ti.i * 1000000ll);
  printf("hands target offset %d ms\r\n", tp[0].ti.i);
  return 1;
}

/* Anything that changes the clock's rate or geometry has to ripple into
   the detector's windows and the loop's schedule. */
static void reclock(void)
{
  sense_refresh_timing();
  control_init();
  printf("swing %llu ns, sense event every %llu ns, windows %lu / %lu..%lu us\r\n",
         (unsigned long long)cfg_period_ns(),
         (unsigned long long)cfg_event_interval_ns(),
         (unsigned long)sense_rearm_us(), (unsigned long)sense_min_event_us(),
         (unsigned long)sense_max_event_us());
}

static int bph_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.beats_per_hour   = (uint32_t)tp[0].ti.i;
  cfg.beats_per_period = (uint8_t)(tp[1].ti.i ? tp[1].ti.i : 2);
  if (cfg.beats_per_hour == 0u) cfg.beats_per_hour = DEFAULT_BEATS_PER_HOUR;
  printf("%lu bph, %u beats per swing = %lu swings/hour\r\n",
         (unsigned long)cfg.beats_per_hour, cfg.beats_per_period,
         (unsigned long)(cfg.beats_per_hour / cfg.beats_per_period));
  reclock();
  return 1;
}

static int geometry_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.events_per_period = (uint8_t)tp[0].ti.i;
  if (cfg.events_per_period == 0u) cfg.events_per_period = 1u;
  if (cfg.events_per_period > 4u)  cfg.events_per_period = 4u;
  cfg.drive_offset_ppt  = (uint16_t)tp[1].ti.i;
  if (cfg.drive_offset_ppt > 1000u) cfg.drive_offset_ppt = 1000u;
  printf("sense coil reports %u time(s) per swing", cfg.events_per_period);
  printf(cfg.events_per_period == 1u ? " (at an extreme)\r\n"
                                     : " (crossing the centre)\r\n");
  printf("drive coil reached %u ppt of a period later", cfg.drive_offset_ppt);
  printf(cfg.drive_offset_ppt == 500u ? " (opposite extreme)\r\n"
       : cfg.drive_offset_ppt == 0u   ? " (same place as the sense coil)\r\n"
                                      : "\r\n");
  reclock();
  return 1;
}

static int windows_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.rearm_pct     = (uint8_t)tp[0].ti.i;
  cfg.min_event_pct = (uint8_t)tp[1].ti.i;
  cfg.max_event_pct = (uint8_t)tp[2].ti.i;
  reclock();
  return 1;
}

static int lock_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.acquire_events  = (uint16_t)tp[0].ti.i;
  cfg.acquire_tol_pct = (uint8_t)tp[1].ti.i;
  printf("lock after %u good events, gap tolerance %u%%\r\n",
         cfg.acquire_events, cfg.acquire_tol_pct);
  control_init();
  return 1;
}

static int sample_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.sample_hz = (uint32_t)tp[0].ti.i;
  printf("envelope sampling %lu hz - takes effect at the next reset\r\n",
         (unsigned long)cfg.sample_hz);
  return 1;
}

static int gains_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.kp_swings = (uint32_t)tp[0].ti.i;
  cfg.ki_swings = (uint32_t)tp[1].ti.i;
  printf("kp %lu swings, ki %lu swings\r\n",
         (unsigned long)cfg.kp_swings, (unsigned long)cfg.ki_swings);
  return 1;
}

static int slew_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.slew_limit_ppm = tp[0].ti.i;
  printf("slew limit %ld ppm\r\n", (long)cfg.slew_limit_ppm);
  return 1;
}

static int wifi_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  strncpy(cfg.ssid, tp[0].ts.str, CONFIG_SSID_LEN - 1);
  cfg.ssid[CONFIG_SSID_LEN - 1] = '\0';
  strncpy(cfg.pass, tp[1].ts.str, CONFIG_PASS_LEN - 1);
  cfg.pass[CONFIG_PASS_LEN - 1] = '\0';
  printf("ssid \"%s\", %u character key - 'save' to keep it\r\n",
         cfg.ssid, (unsigned)strlen(cfg.pass));
  net_reconnect();
  return 1;
}

static int ntp_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  strncpy(cfg.ntp_host, tp[0].ts.str, CONFIG_HOST_LEN - 1);
  cfg.ntp_host[CONFIG_HOST_LEN - 1] = '\0';
  printf("ntp server %s\r\n", cfg.ntp_host);
  net_reconnect();
  return 1;
}

static int sync_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  net_request_sync();
  printf("sync requested\r\n");
  return 1;
}

static int save_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  printf("%s\r\n", config_save() ? "saved" : "SAVE FAILED");
  return 1;
}

static int defaults_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  config_defaults();
  printf("defaults loaded into ram - 'save' to commit, or reboot to discard\r\n");
  return 1;
}

static int bootsel_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  drive_all_off();
  printf("entering bootsel\r\n");
  sleep_ms(100);
  reset_usb_boot(0, 0);
  return 1;
}

static int help_cmd(int args, tinycl_parameter *tp, void *v);

static const tinycl_command tcmds[] =
{
  { "HELP",     "this list",                              help_cmd,     {TINYCL_PARM_END} },
  { "STATUS",   "everything the device knows",            status_cmd,   {TINYCL_PARM_END} },
  { "RESONANCE","lo,hi,plot(y|n) - calibrate the tank",   resonance_cmd,{TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "SWEEP",    "from,to,step hz - raw table",            sweep_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "CAPTURE",  "rate_hz,count - raw tank waveform",      capture_cmd,  {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "TANK",     "hz - set tank drive frequency",          tank_cmd,     {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "SENSE",    "y|n - detector",                      sense_cmd,    {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "THRESH",   "counts - detection threshold",           thresh_cmd,   {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "DIR",      "y if the bob makes amplitude fall",     dir_cmd,      {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "WATCH",    "y|n - echo every swing",              watch_cmd,    {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "PULSE",    "us - fire the coil once, now",           pulse_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "COILOFF",  "drop the coil and cancel pending",       coiloff_cmd,  {TINYCL_PARM_END} },
  { "PW",       "us - correction pulse width",            pw_cmd,       {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "PTIME",    "advance_us,retard_us - pulse placing",   ptime_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "AUTH",     "ns - phase step one pulse buys",         auth_cmd,     {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "MEASURE",  "swings,retard(y|n) - measure that step",      measure_cmd,  {TINYCL_PARM_INT, TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "CONTROL",  "y|n - close the loop",                control_cmd,  {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "OFFSET",   "ms - walk the hands by this much",       offset_cmd,   {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "BPH",      "beats_per_hour,beats_per_swing",         bph_cmd,      {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "GEOMETRY", "events_per_swing,drive_offset_ppt",      geometry_cmd, {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "WINDOWS",  "rearm%,min_bump%,max_bump% of interval", windows_cmd,  {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "LOCK",     "events_to_lock,gap_tolerance%",          lock_cmd,     {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "SAMPLE",   "hz - envelope sampling rate",            sample_cmd,   {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "GAINS",    "kp_swings,ki_swings",                    gains_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "SLEW",     "ppm - cap on rate correction",           slew_cmd,     {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "WIFI",     "ssid,password",                          wifi_cmd,     {TINYCL_PARM_STR, TINYCL_PARM_STR, TINYCL_PARM_END} },
  { "NTP",      "hostname",                               ntp_cmd,      {TINYCL_PARM_STR, TINYCL_PARM_END} },
  { "SYNC",     "ask for an NTP exchange now",            sync_cmd,     {TINYCL_PARM_END} },
  { "SAVE",     "write configuration to flash",           save_cmd,     {TINYCL_PARM_END} },
  { "DEFAULTS", "load defaults into ram",                 defaults_cmd, {TINYCL_PARM_END} },
  { "BOOTSEL",  "reboot into the uf2 bootloader",         bootsel_cmd,  {TINYCL_PARM_END} },
};

static int help_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  tinycl_print_commands(sizeof(tcmds) / sizeof(tinycl_command), tcmds);
  return 1;
}

void cli_init(void)
{
  tinycl_do_echo = 1;
  printf("\r\nSynchronizer - pendulum clock discipline\r\n");
  printf("type HELP for commands\r\n> ");
}

void cli_poll(void)
{
  if (tinycl_task(sizeof(tcmds) / sizeof(tinycl_command), tcmds, NULL))
  {
    tinycl_do_echo = 1;
    tinycl_put_string("> ");
  }
}
