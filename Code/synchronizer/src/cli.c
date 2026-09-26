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

/* Defined below; every command that borrows the ADC calls it. */
static void note_diag_cost(void);
#include "board.h"
#include "config.h"
#include "sense.h"
#include "drive.h"
#include "timebase.h"
#include "netclock.h"
#include "conout.h"
#include "httpd.h"
#include "dhcpserver.h"
#include "dnsserver.h"
#include "lwip/netif.h"
#include "control.h"
#include "chime.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "tinycl.h"
#include "cli.h"

/* --- capturing what a command prints ------------------------------------

   Registered once, alongside stdio_usb, and inert until cap_on is set.
   out_chars can be reached from any context, so it does nothing but copy
   bytes into a caller-supplied buffer. */

static char    *cap_buf;
static uint32_t cap_sz, cap_len;
static bool     cap_on, cap_trunc;

/* Everything the console prints ALSO lands in a ring, whether or not a
   command asked for it - except what a captured command prints itself,
   which would otherwise appear twice.  The results that matter most - an
   authority measurement finishing, a placement sweep's table, the swing
   echo - are printed from the control loop long after the command that
   started them returned, so the per-command capture never sees them.
   That was fine while a USB cable was attached.  It is not fine once the
   clock is back in its own room and the only way in is wireless.

   The web console relies on that split: a command's own reply arrives once,
   via cap_buf and /api/cli; the ring (/api/log) is for everything else, so
   it only ever shows what the device printed on its own.  Letting a
   captured command's bytes into the ring too made its reply show up a
   second time there, which is what this cap_on guard prevents. */
#define LOG_SIZE   4096u
static char     log_buf[LOG_SIZE];
static uint32_t log_head;          /* next write position                 */
static uint32_t log_seq;           /* bytes ever written; the cursor      */

static void cap_out_chars(const char *buf, int len)
{
  int i;

  if (!cap_on)
  {
    for (i = 0; i < len; i++)
    {
      log_buf[log_head] = buf[i];
      log_head = (log_head + 1u) % LOG_SIZE;
      log_seq++;
    }
  }

  if (!cap_on || !cap_buf) return;
  for (i = 0; i < len; i++)
  {
    if (cap_len + 1u < cap_sz) cap_buf[cap_len++] = buf[i];
    else cap_trunc = true;
  }
}

static stdio_driver_t cap_driver =
{
  .out_chars = cap_out_chars,
};

/* tinycl pulls characters through this hook, so a command can be fed in
   from anywhere.  Declared in tinycl.c; not in its header. */
extern int tinycl_rppico_getchar(void *v);

static const char *feed_p;

static int feed_getchar(void *v)
{
  (void)v;
  if (!feed_p || *feed_p == '\0') return -1;
  return (int)(unsigned char)(*feed_p++);
}

static void print_ns(const char *label, int64_t ns)
{
  int64_t us = ns / 1000;
  printf("%-22s %lld us\r\n", label, (long long)us);
}

static void print_sod(const char *label, uint32_t sod)
{
  printf("%-22s %02lu:%02lu:%02lu\r\n", label,
         (unsigned long)(sod / 3600u), (unsigned long)((sod / 60u) % 60u),
         (unsigned long)(sod % 60u));
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
  printf("%-22s %u of %u conversions trimmed, baseline 2^%u ms\r\n", "filter",
         cfg.env_trim, cfg.env_oversample, cfg.baseline_shift);
  printf("%-22s %lu hz, pulse %lu ns (%lu counts, %lu ns actual)\r\n", "tank drive",
         (unsigned long)sense_tank_hz(), (unsigned long)sense_drive_ns(),
         (unsigned long)sense_drive_level(),
         (unsigned long)sense_drive_actual_ns());
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
  printf("%-22s %lu chatter, %lu rejected by the width gate\r\n", "",
         (unsigned long)sense_chatter_count(), (unsigned long)sense_rejected_count());
  printf("%-22s %lu  (%lu dropped)\r\n", "events",
         (unsigned long)sense_event_count(), (unsigned long)sense_overrun_count());

  printf("\r\n-- drive ------------------------------------------------\r\n");
  printf("%-22s %s\r\n", "coil", drive_is_on() ? "ON" : "off");
  if (cfg.pulse_advance_width_us)
    printf("%-22s width %u us, offset %ld us\r\n", "advance",
           cfg.pulse_advance_width_us, (long)cfg.pulse_advance_us);
  else
    printf("%-22s disabled (width 0), offset %ld us\r\n", "advance",
           (long)cfg.pulse_advance_us);
  if (cfg.pulse_retard_width_us)
    printf("%-22s width %u us, offset %ld us\r\n", "retard",
           cfg.pulse_retard_width_us, (long)cfg.pulse_retard_us);
  else
    printf("%-22s disabled (width 0), offset %ld us\r\n", "retard",
           (long)cfg.pulse_retard_us);
  printf("%-22s advance speeds up a slow clock; retard slows down a fast"
         " one\r\n", "");
  printf("%-22s %lu fired, %lu refused, %lu us budget\r\n", "pulses",
         (unsigned long)drive_pulse_count(), (unsigned long)drive_refused_count(),
         (unsigned long)drive_budget_us());

  printf("\r\n-- time -------------------------------------------------\r\n");
  printf("%-22s %s  %s\r\n", "network", net_status_name(), net_ip());
  printf("%-22s UTC%+ld:%02lu\r\n", "local time is",
         (long)(cfg.tz_offset_s / 3600), (unsigned long)((abs(cfg.tz_offset_s) / 60) % 60));
  printf("%-22s %s.local%s\r\n", "name", net_hostname(),
         net_mdns_active() ? "" : "   (not advertised - no interface up)");
  if (net_in_ap())
    printf("%-22s \"%s\", key \"%s\", page at 192.168.4.1\r\n",
           "setup ap", net_ap_ssid(), cfg.ap_pass);
  printf("%-22s %lu ok, %lu failed, last rtt %lu us\r\n", "ntp",
         (unsigned long)net_ntp_ok(), (unsigned long)net_ntp_fail(),
         (unsigned long)net_last_rtt_us());
  printf("%-22s %lu bytes dropped%s\r\n", "console",
         (unsigned long)conout_dropped(),
         conout_dropped() ? " - terminal fell behind, output was lost" : "");
  utc = tb_utc_ns();
  if (tb_have_time())
  {
    uint64_t s = utc / 1000000000ull + (uint64_t)(int64_t)cfg.tz_offset_s;
    printf("%-22s %llu  (%02llu:%02llu:%02llu)\r\n", "unix time",
           (unsigned long long)(utc / 1000000000ull),
           (unsigned long long)((s / 3600ull) % 24ull),
           (unsigned long long)((s / 60ull) % 60ull),
           (unsigned long long)(s % 60ull));
    printf("%-22s %ld ppb, %lu fixes, last offset %lld us, kp %lu fixes\r\n",
           "timebase", (long)tb_ppb(), (unsigned long)tb_fix_count(),
           (long long)(tb_last_offset_ns() / 1000),
           (unsigned long)(cfg.tb_kp_fixes ? cfg.tb_kp_fixes : 20u));
  }
  else
    printf("%-22s not set\r\n", "unix time");

  printf("\r\n-- chime ------------------------------------------------\r\n");
  if (chime_have())
  {
    uint32_t away, face, real, now_sod;
    int32_t  o = chime_offset_ms();
    printf("%-22s %ld.%03lu s %s\r\n", "hands are",
           (long)(o / 1000), (unsigned long)(abs(o) % 1000),
           (o >= 0) ? "fast" : "slow");
    if (chime_face_now_sod(&now_sod))
      print_sod("clock face reads", now_sod);
    if (chime_next(&away, &face, &real))
    {
      printf("%-22s %lu:%02lu from now\r\n", "next chime",
             (unsigned long)(away / 60u), (unsigned long)(away % 60u));
      print_sod("  striking", face);
      print_sod("  at true time", real);
    }
  }
  else
    printf("%-22s not measured - use CHIME once you hear it\r\n", "hands are");
  printf("%-22s every %lu min\r\n", "chimes", (unsigned long)cfg.chime_interval_min);
  if (cfg.chime_strike_offset_s)
    printf("%-22s %ld s %s the hour\r\n", "strike offset",
           (long)(cfg.chime_strike_offset_s < 0 ? -cfg.chime_strike_offset_s
                                                 : cfg.chime_strike_offset_s),
           cfg.chime_strike_offset_s < 0 ? "before" : "after");

  printf("\r\n-- loop -------------------------------------------------\r\n");
  printf("%-22s %s\r\n", "state", control_state_name(cs.state));
  if (!control_actuator())
    printf("%-22s muted - tracking continues, no pulses will fire\r\n", "actuator");
  /* A fast-smoothed comparison against a plain, never-adapting nominal
     schedule.  Grows without bound while the clock is genuinely off and
     nothing is correcting it; unlike "phase error" below, it cannot
     converge to zero just because the tracker has learned to predict a
     wrong rate, and unlike routing it through the tracking NCO, it
     notices a real correction working within a few events instead of
     days - it is the honest answer to "how wrong does the device think
     the clock is right now". */
  {
    long long us = (long long)(cs.sched_err_ns / 1000);
    printf("%-22s %lld us (%s)\r\n", "uncorrected error", us,
           us > 0 ? "slow" : (us < 0 ? "fast" : "on time"));
  }
  printf("%-22s %s, %u since last kick (min %u), trips at +-%u%%\r\n",
         "kick",
         cs.kick_active ? (cs.kick_dir_retard ? "retarding" : "advancing")
                        : "idle",
         (unsigned)cs.kick_since,
         (unsigned)(cfg.kick_min_swings ? cfg.kick_min_swings : 5u),
         (unsigned)(cfg.kick_threshold_pct ? cfg.kick_threshold_pct : 25u));
  /* err_ns/filt_err_ns (edge-based, rerr) and kick_filt_ns (phase-based,
     demod_ns) have opposite sign conventions - negate the former so both
     read "positive = hands ahead, wants retarding" before comparing. */
  printf("%-22s %lld us\r\n", "edge vs phase",
         (long long)((-cs.filt_err_ns - cs.kick_filt_ns) / 1000));
  printf("%-22s %llu  (%lu missed)\r\n", "events", (unsigned long long)cs.events,
         (unsigned long)cs.missed);
  print_ns("phase error", cs.err_ns);
  print_ns("phase error (filtered)", cs.filt_err_ns);
  print_ns("command per swing", cs.cmd_ns_per_swing);
  printf("%-22s %lld ppb  (%lld ms/day)\r\n", "pendulum drift",
         (long long)cs.drift_ppb, (long long)(cs.drift_ppb * 864ll / 10000ll));
  printf("%-22s %lld us per swing%s  (%lu events)\r\n", "feedforward",
         (long long)(cs.ff_ns / 1000), cs.rate_ready ? "" : " - still settling",
         (unsigned long)cs.rate_n);
  printf("%-22s kp %lu events, ki %lu events, slew %ld ppm\r\n", "gains",
         (unsigned long)cfg.kp_swings, (unsigned long)cfg.ki_swings,
         (long)cfg.slew_limit_ppm);
  printf("%-22s %lu events\r\n", "rate tracker kp",
         (unsigned long)cfg.rate_kp_events);
  printf("\r\n");
  return 1;
}

static int sweep_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_sweep((uint32_t)tp[0].ti.i, (uint32_t)tp[1].ti.i,
              (uint32_t)tp[2].ti.i, 25u);
  note_diag_cost();
  return 1;
}

static int capture_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_capture((uint32_t)tp[0].ti.i, (uint32_t)tp[1].ti.i);
  note_diag_cost();
  return 1;
}

static int resonance_cmd(int args, tinycl_parameter *tp, void *v)
{
  sense_resonance r;
  uint32_t lo = (uint32_t)tp[0].ti.i, hi = (uint32_t)tp[1].ti.i;

  (void)args; (void)v;
  if (lo == 0u) lo = 20000u;
  if (hi == 0u) hi = 120000u;

  printf("keep metal away from the sense coil - the bob, your hand, tools\r\n");
  sense_find_resonance(lo, hi, tp[2].tb.b, &r);
  note_diag_cost();

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

static int filter_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.env_oversample = (uint8_t)tp[0].ti.i;
  cfg.env_trim       = (uint8_t)tp[1].ti.i;
  cfg.baseline_shift = (uint8_t)tp[2].ti.i;
  if (cfg.env_oversample < 4u)  cfg.env_oversample = 4u;
  if (cfg.env_oversample > 32u) cfg.env_oversample = 32u;
  if (cfg.env_trim * 2u >= cfg.env_oversample)
    cfg.env_trim = (uint8_t)((cfg.env_oversample - 1u) / 2u);
  if (cfg.baseline_shift < 6u)  cfg.baseline_shift = 6u;
  if (cfg.baseline_shift > 20u) cfg.baseline_shift = 20u;
  printf("%u conversions, %u trimmed each end (%s), baseline 2^%u = %lu ms\r\n",
         cfg.env_oversample, cfg.env_trim,
         (cfg.env_trim == 0u) ? "plain mean"
           : ((cfg.env_trim * 2u + 2u >= cfg.env_oversample) ? "median" : "trimmed mean"),
         cfg.baseline_shift, (unsigned long)(1ul << cfg.baseline_shift));
  return 1;
}

static int hyst_cmd(int args, tinycl_parameter *tp, void *v)
{
  int32_t p = tp[0].ti.i;
  (void)args; (void)v;
  if (p < 0)  p = 0;
  if (p > 90) p = 90;
  cfg.detect_hyst_pct = (uint8_t)p;
  printf("hysteresis %u%% - event ends below %lu counts, timed at %u\r\n",
         cfg.detect_hyst_pct,
         (unsigned long)(cfg.detect_threshold
                         - (cfg.detect_threshold * cfg.detect_hyst_pct) / 100u),
         cfg.detect_threshold);
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

static int phaselog_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  control_set_phaselog((uint32_t)tp[0].ti.i);
  if (control_phaselog())
    printf("phaselog every %lu s\r\n", (unsigned long)control_phaselog());
  else
    printf("phaselog off\r\n");
  return 1;
}

static int actuator_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  control_set_actuator(tp[0].tb.b);
  printf("actuator %s%s\r\n", control_actuator() ? "on" : "muted",
         control_actuator() ? "" : " - tracking continues, no pulses will fire");
  return 1;
}

static int pulse_cmd(int args, tinycl_parameter *tp, void *v)
{
  uint32_t us = (uint32_t)tp[0].ti.i;
  (void)args; (void)v;
  printf("%s\r\n", drive_pulse(us) ? "fired" : "REFUSED (0, too wide, or duty budget spent)");
  return 1;
}

static int coiloff_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  drive_all_off();
  printf("coil off\r\n");
  return 1;
}

static int coiltest_cmd(int args, tinycl_parameter *tp, void *v)
{
  uint32_t ms = (uint32_t)tp[0].ti.i;
  (void)args; (void)v;

  if (strcmp(tp[1].ts.str, "YES") != 0)
  {
    printf("refused - this holds the coil on for real seconds, not a pulse;\r\n"
           "say YES to mean it: COILTEST ms YES\r\n");
    return 1;
  }
  if (drive_coil_test_used())
  {
    printf("already used this boot - power-cycle the board to test again\r\n");
    return 1;
  }
  if (ms > COIL_TEST_MAX_MS) ms = COIL_TEST_MAX_MS;
  if (drive_coil_test(ms))
  {
    printf("coil driven for %lu ms - watch or feel for the pull now\r\n",
           (unsigned long)ms);
    note_diag_cost();
  }
  else
    printf("refused\r\n");
  return 1;
}

static int pw_cmd(int args, tinycl_parameter *tp, void *v)
{
  uint32_t adv = (uint32_t)tp[0].ti.i, ret = (uint32_t)tp[1].ti.i;
  (void)args; (void)v;
  if (adv > DRIVE_MAX_PULSE_US) adv = DRIVE_MAX_PULSE_US;
  if (ret > DRIVE_MAX_PULSE_US) ret = DRIVE_MAX_PULSE_US;
  cfg.pulse_advance_width_us = adv;
  cfg.pulse_retard_width_us  = ret;
  printf("advance width %u us, retard width %u us%s%s\r\n", adv, ret,
         (!adv || !ret) ? " - " : "",
         !adv && !ret ? "neither direction will pulse"
         : !adv ? "advance disabled, can only retard"
         : !ret ? "retard disabled, can only advance" : "");
  return 1;
}

static int ptime_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.pulse_advance_us = (int32_t)tp[0].ti.i;
  cfg.pulse_retard_us  = (int32_t)tp[1].ti.i;
  printf("advance offset %ld us, retard offset %ld us, both from centre\r\n",
         (long)cfg.pulse_advance_us, (long)cfg.pulse_retard_us);
  return 1;
}

static int kick_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.kick_min_swings    = (uint16_t)tp[0].ti.i;
  cfg.kick_threshold_pct = (uint16_t)tp[1].ti.i;
  control_kick_reset();
  printf("kick: minimum %u swings between pulses, trips at +-%u%% of a"
         " swing - picks advance or retard itself\r\n",
         (unsigned)(cfg.kick_min_swings ? cfg.kick_min_swings : 5u),
         (unsigned)(cfg.kick_threshold_pct ? cfg.kick_threshold_pct : 25u));
  return 1;
}

static int forceerr_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  int64_t us = (int64_t)tp[0].ti.i;
  control_force_sched_err_ns(us * 1000ll);
  printf("uncorrected error forced to %lld us - KICK reacts on the next"
         " tracked event\r\n", (long long)us);
  return 1;
}

static int modscan_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_mod_scan((uint32_t)tp[0].ti.i, (uint32_t)tp[1].ti.i,
                 (uint32_t)tp[2].ti.i);
  note_diag_cost();
  return 1;
}


static int trace_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_trace((uint32_t)tp[0].ti.i);
  note_diag_cost();
  return 1;
}

static int pulsetrace_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_pulse_trace((uint32_t)tp[0].ti.i, (uint32_t)tp[1].ti.i);
  note_diag_cost();
  return 1;
}

/* Unlike every other trace command, this never borrows the ADC and never
   holds the loop - it just arms a trigger inside the real detector and
   waits, so note_diag_cost() (which reports on a diagnostic having just
   interrupted tracking) does not apply here. */
static int chattertrace_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  if (!sense_chatter_ready()) sense_chatter_arm();
  sense_chatter_dump();
  return 1;
}

/* Say what a diagnostic cost, once, in the same words everywhere.  The
   detector is blind while the ADC is borrowed, so these commands always
   cost some swings - the point is that it is visible rather than showing
   up later as an inexplicable rate. */
static void note_diag_cost(void)
{
  const char *what = sense_diag_interrupted();
  if (what) printf("  (%s)\r\n", what);
}

static int env_cmd(int args, tinycl_parameter *tp, void *v)
{
  sense_env_stats st;
  uint32_t pp;

  (void)args; (void)v;
  sense_envelope((uint32_t)tp[0].ti.i, &st);
  if (st.samples == 0u) { printf("no samples\r\n"); return 1; }

  pp = (uint32_t)st.max - (uint32_t)st.min;
  printf("envelope over %lu ms, %lu samples (%lu k/s)\r\n",
         (unsigned long)st.ms, (unsigned long)st.samples,
         (unsigned long)(st.samples / (st.ms ? st.ms : 1u)));
  printf("  min %u   mean %u   max %u   peak-to-peak %lu\r\n",
         st.min, st.mean, st.max, (unsigned long)pp);
  printf("  raw %u..%u (%lu) - filtering took out %lu counts\r\n",
         st.raw_min, st.raw_max,
         (unsigned long)(st.raw_max - st.raw_min),
         (unsigned long)((uint32_t)(st.raw_max - st.raw_min) > pp
                         ? (uint32_t)(st.raw_max - st.raw_min) - pp : 0u));
  if (st.mean)
    printf("  spread %lu.%lu%% of mean\r\n",
           (unsigned long)((pp * 100u) / st.mean),
           (unsigned long)(((pp * 1000u) / st.mean) % 10u));
  note_diag_cost();
  return 1;
}

static int drive_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  sense_set_drive_ns((uint32_t)tp[0].ti.i);
  printf("tank drive pulse %lu ns asked, %lu ns actual (%lu timer counts)\r\n",
         (unsigned long)sense_drive_ns(), (unsigned long)sense_drive_actual_ns(),
         (unsigned long)sense_drive_level());
  if (sense_drive_ns() > 0u && sense_drive_level() < 8u)
    printf("  (only %lu counts - each step is %lu ns, so this is coarse)\r\n",
           (unsigned long)sense_drive_level(),
           (unsigned long)(sense_drive_level()
                           ? sense_drive_actual_ns() / sense_drive_level() : 0u));
  return 1;
}

static int control_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  control_enable(tp[0].tb.b);
  printf("control %s\r\n", cfg.control_enabled ? "on" : "off");
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

/* The pendulum rate tracker's own time constant.  It has to stay slower
   than the timebase's NTP rate learning or a crystal temperature excursion
   the timebase has not caught yet is fed forward as pendulum rate, so this
   wants raising rather than lowering if the reported drift ever moves with
   the room. */
static int ratekp_cmd(int args, tinycl_parameter *tp, void *v)
{
  uint32_t n = (uint32_t)tp[0].ti.i;
  (void)args; (void)v;
  if (n < 20u)    n = 20u;
  if (n > 20000u) n = 20000u;
  cfg.rate_kp_events = n;
  control_reset();
  printf("rate tracker kp %lu events (~%lu s); tracker restarted, and\r\n"
         "MEASURE will not run again until it has settled\r\n",
         (unsigned long)n,
         (unsigned long)(((uint64_t)n * (cfg_period_ns() / 1000000ull)) / 1000ull));
  return 1;
}

/* The timebase's own type-2 loop time constant, in accepted NTP fixes
   rather than seconds - see the comment on tb_apply_fix().  Fixes are
   noisy (network jitter); the crystal they measure only drifts with
   temperature, so this wants raising rather than lowering if ppb ever
   looks like it is chasing individual fixes instead of settling. */
static int ntpkp_cmd(int args, tinycl_parameter *tp, void *v)
{
  uint32_t n = (uint32_t)tp[0].ti.i;
  (void)args; (void)v;
  if (n < 2u)     n = 2u;
  if (n > 2000u)  n = 2000u;
  cfg.tb_kp_fixes = n;
  printf("timebase kp %lu fixes\r\n", (unsigned long)n);
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

static int hostname_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  if (!net_set_hostname(tp[0].ts.str))
    printf("that leaves nothing usable as a name\r\n");
  else
    printf("now answering to %s.local - 'save' to keep it\r\n", net_hostname());
  return 1;
}

/* Everything needed to tell apart "the client never reached us", "we
   answered and it was ignored", and "the interface is not what I think". */
static int chime_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  if (!chime_mark((uint32_t)tp[0].ti.i, (uint32_t)tp[1].ti.i, (uint32_t)tp[2].ti.i))
  {
    printf("no time yet - the clock cannot be placed against UTC until NTP has a fix\r\n");
    return 1;
  }
  {
    int32_t o = chime_offset_ms();
    printf("hands are %ld.%03lu s %s\r\n",
           (long)(o / 1000), (unsigned long)(abs(o) % 1000),
           (o >= 0) ? "FAST" : "slow");
  }
  {
    uint32_t away, face, real;
    if (chime_next(&away, &face, &real))
    {
      printf("next chime in %lu:%02lu\r\n",
             (unsigned long)(away / 60u), (unsigned long)(away % 60u));
      print_sod("  it will strike", face);
      print_sod("  at true time", real);
    }
  }
  printf("'save' to keep it\r\n");
  return 1;
}

static int tz_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  cfg.tz_offset_s = tp[0].ti.i * 60;
  printf("local time is UTC%+ld:%02lu - the chime is read against this\r\n",
         (long)(cfg.tz_offset_s / 3600), (unsigned long)((abs(cfg.tz_offset_s) / 60) % 60));
  return 1;
}

static int chimeset_cmd(int args, tinycl_parameter *tp, void *v)
{
  int32_t off;
  (void)args; (void)v;
  cfg.chime_interval_min = (uint32_t)tp[0].ti.i;
  if (cfg.chime_interval_min == 0u || cfg.chime_interval_min > 720u)
    cfg.chime_interval_min = 60u;
  cfg.chime_latency_ms = (uint32_t)tp[1].ti.i;
  off = (int32_t)tp[2].ti.i;
  {
    int32_t half = (int32_t)(cfg.chime_interval_min * 60u) / 2;
    if (off >  half) off =  half;
    if (off < -half) off = -half;
  }
  cfg.chime_strike_offset_s = off;
  printf("chimes every %lu min, press allowance %lu ms, strikes",
         (unsigned long)cfg.chime_interval_min, (unsigned long)cfg.chime_latency_ms);
  if (off == 0) printf(" exactly on the hour\r\n");
  else printf(" %ld s %s the hour\r\n", (long)(off < 0 ? -off : off),
              off < 0 ? "before" : "after");
  return 1;
}

static int net_cmd(int args, tinycl_parameter *tp, void *v)
{
  struct netif *n;
  uint32_t a, b, d;
  char ip[20], mask[20], gw[20];

  (void)args; (void)tp; (void)v;

  printf("\r\n-- interfaces ------------------------------------------\r\n");
  for (n = netif_list; n; n = n->next)
  {
    /* ip4addr_ntoa returns a single static buffer, so three calls in one
       printf all render the same address.  Copy each out first. */
    ip4addr_ntoa_r(netif_ip4_addr(n), ip, sizeof(ip));
    ip4addr_ntoa_r(netif_ip4_netmask(n), mask, sizeof(mask));
    ip4addr_ntoa_r(netif_ip4_gw(n), gw, sizeof(gw));
    printf("  %c%c%u  %-15s mask %-15s gw %-15s %s%s%s%s\r\n",
           n->name[0], n->name[1], n->num, ip, mask, gw,
           netif_is_up(n)        ? "up "        : "DOWN ",
           netif_is_link_up(n)   ? "link "      : "nolink ",
           (n->flags & NETIF_FLAG_BROADCAST) ? "bcast " : "nobcast ",
           (n == netif_default)  ? "DEFAULT"    : "");
  }
  if (!netif_list) printf("  (none)\r\n");

  printf("\r\n-- servers ---------------------------------------------\r\n");
  dhcpserver_stats(&a, &b, &d);
  printf("  dhcp   %-8s rx %lu, replied %lu, ignored %lu, leases %lu\r\n",
         dhcpserver_running() ? "running" : "stopped",
         (unsigned long)a, (unsigned long)b, (unsigned long)d,
         (unsigned long)dhcpserver_leases());
  dnsserver_stats(&a, &b);
  printf("  dns    %-8s queries %lu, answered %lu\r\n",
         dnsserver_running() ? "running" : "stopped",
         (unsigned long)a, (unsigned long)b);
  printf("  http   %-8s requests %lu\r\n",
         httpd_running() ? "running" : "stopped",
         (unsigned long)httpd_requests());
  printf("  mdns   %-8s %s.local\r\n",
         net_mdns_active() ? "running" : "stopped", net_hostname());

  printf("\r\n-- wifi ------------------------------------------------\r\n");
  printf("  state  %s\r\n", net_status_name());
  if (net_in_ap())
  {
    uint32_t again = net_ap_retry_s();
    printf("  ap     \"%s\", key \"%s\"\r\n", net_ap_ssid(), cfg.ap_pass);
    if (again)
      printf("  retry  \"%s\" in %lu s\r\n", cfg.ssid, (unsigned long)again);
    else
      printf("  retry  never - %s\r\n",
             cfg.ssid[0] ? "held up by hand, 'ap n' to rejoin" : "no network configured");
  }
  else
    printf("  ssid   \"%s\"\r\n", cfg.ssid);
  printf("\r\n");
  return 1;
}

static int ap_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  net_ap_force(tp[0].tb.b);
  if (tp[0].tb.b)
    printf("access point \"%s\" up, key \"%s\", page at 192.168.4.1\r\n",
           net_ap_ssid(), cfg.ap_pass);
  else
    printf("access point down\r\n");
  return 1;
}

static int apkey_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)v;
  strncpy(cfg.ap_pass, tp[0].ts.str, CONFIG_PASS_LEN - 1);
  cfg.ap_pass[CONFIG_PASS_LEN - 1] = '\0';
  printf("setup ap key set (%u characters) - 'save' to keep it\r\n",
         (unsigned)strlen(cfg.ap_pass));
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

/* Everything a config-layout change (CONFIG_VERSION) or a bad flash wipes,
   as the exact commands that put it back - so a reflash is "paste this
   back in" rather than "remember what THRESH, WINDOWS, PTIME and KICK
   were before". WIFI/APKEY/HOSTNAME are deliberately left out: they live
   in their own flash sector (see config.h) and survive a version bump on
   their own, so replaying them here would be redundant at best and would
   echo the Wi-Fi password back over the console at worst. tank_f0_hz and
   friends are RESONANCE's measured output, not a setting - run RESONANCE
   again rather than trying to restore a number here. */
static int recreate_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  printf("# copy the lines below, in order, into a fresh device -\r\n"
         "# wifi/hostname survive a reset on their own and are not here\r\n");
  printf("BPH %lu %u\r\n",
         (unsigned long)cfg.beats_per_hour, cfg.beats_per_period);
  printf("GEOMETRY %u %u\r\n", cfg.events_per_period, cfg.drive_offset_ppt);
  printf("TANK %lu\r\n", (unsigned long)cfg.tank_hz);
  printf("DRIVE %lu\r\n", (unsigned long)cfg.tank_drive_ns);
  printf("THRESH %u\r\n", cfg.detect_threshold);
  printf("DIR %s\r\n", cfg.detect_falling ? "Y" : "N");
  printf("SENSE %s\r\n", cfg.sense_enabled ? "Y" : "N");
  printf("HYST %u\r\n", cfg.detect_hyst_pct);
  printf("FILTER %u %u %u\r\n",
         cfg.env_oversample, cfg.env_trim, cfg.baseline_shift);
  printf("SAMPLE %lu\r\n", (unsigned long)cfg.sample_hz);
  printf("WINDOWS %u %u %u\r\n",
         cfg.rearm_pct, cfg.min_event_pct, cfg.max_event_pct);
  printf("LOCK %u %u\r\n", cfg.acquire_events, cfg.acquire_tol_pct);
  printf("PW %lu %lu\r\n", (unsigned long)cfg.pulse_advance_width_us,
         (unsigned long)cfg.pulse_retard_width_us);
  printf("PTIME %ld %ld\r\n",
         (long)cfg.pulse_advance_us, (long)cfg.pulse_retard_us);
  printf("KICK %u %u\r\n",
         (unsigned)(cfg.kick_min_swings ? cfg.kick_min_swings : 5u),
         (unsigned)(cfg.kick_threshold_pct ? cfg.kick_threshold_pct : 25u));
  printf("RATEKP %lu\r\n", (unsigned long)cfg.rate_kp_events);
  printf("NTPKP %lu\r\n", (unsigned long)cfg.tb_kp_fixes);
  printf("GAINS %lu %lu\r\n",
         (unsigned long)cfg.kp_swings, (unsigned long)cfg.ki_swings);
  printf("SLEW %ld\r\n", (long)cfg.slew_limit_ppm);
  printf("CHIMESET %lu %lu %ld\r\n",
         (unsigned long)cfg.chime_interval_min,
         (unsigned long)cfg.chime_latency_ms,
         (long)cfg.chime_strike_offset_s);
  printf("TZ %ld\r\n", (long)(cfg.tz_offset_s / 60));
  printf("NTP %s\r\n", cfg.ntp_host);
  printf("CONTROL %s\r\n", cfg.control_enabled ? "Y" : "N");
  printf("SAVE\r\n");
  printf("# tank resonance (RESONANCE) and CHIME are measured live, not\r\n"
         "# stored settings - redo them, do not try to script them\r\n");
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
  { "RESONANCE","lo hi plot(y|n) - calibrate the tank",   resonance_cmd,{TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "SWEEP",    "from to step (hz) - raw table",            sweep_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "CAPTURE",  "rate_hz count - raw tank waveform",      capture_cmd,  {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "TANK",     "hz - set tank drive frequency",          tank_cmd,     {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "DRIVE",    "ns - tank drive pulse width (0 = off)",   drive_cmd,   {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "ENV",      "ms - envelope min/mean/max over a window", env_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "TRACE",    "ms - plot the envelope against time",      trace_cmd,  {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "PULSETRACE","pulse_us ms - trace, firing one pulse partway through", pulsetrace_cmd,
                {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "CHATTERTRACE","arm/dump a capture of the detector's own next chatter event",
                chattertrace_cmd, {TINYCL_PARM_END} },
  { "MODSCAN",  "lo hi steps - find the best drive (0 0 0)", modscan_cmd, {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "SENSE",    "y|n - detector",                      sense_cmd,    {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "THRESH",   "counts - detection threshold",           thresh_cmd,   {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "HYST",     "pct of threshold - event-end hysteresis", hyst_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "FILTER",   "oversample trim baseline_shift",          filter_cmd,  {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "DIR",      "y if the bob makes amplitude fall",     dir_cmd,      {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "WATCH",    "y|n - echo every swing",              watch_cmd,    {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "PHASELOG", "seconds - periodic phase/pulses log, 0 off", phaselog_cmd, {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "ACTUATOR", "y|n - mute corrective pulses only; tracking keeps running", actuator_cmd, {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "PULSE",    "us - fire the coil once, now",           pulse_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "COILOFF",  "drop the coil and cancel pending",       coiloff_cmd,  {TINYCL_PARM_END} },
  { "COILTEST", "ms YES - hold the coil on to feel it pull; once per boot", coiltest_cmd, {TINYCL_PARM_INT, TINYCL_PARM_STR, TINYCL_PARM_END} },
  { "PW",       "advance_us retard_us - correction pulse width, 0 disables that direction", pw_cmd,
                {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "PTIME",    "advance_us retard_us - pulse placing",   ptime_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "KICK",     "min_swings threshold_pct - hysteresis params, picks direction itself", kick_cmd, {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "FORCEERR", "us - test only: seed uncorrected error to trigger KICK", forceerr_cmd, {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "CONTROL",  "y|n - close the loop",                control_cmd,  {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "BPH",      "beats_per_hour beats_per_swing",         bph_cmd,      {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "GEOMETRY", "events_per_swing drive_offset_ppt",      geometry_cmd, {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "WINDOWS",  "rearm% min% max% of the interval", windows_cmd,  {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "LOCK",     "events_to_lock gap_tolerance%",          lock_cmd,     {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "SAMPLE",   "hz - envelope sampling rate",            sample_cmd,   {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "GAINS",    "kp_swings ki_swings",                    gains_cmd,    {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "SLEW",     "ppm - cap on rate correction",           slew_cmd,     {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "RATEKP",   "events - rate tracker time constant",    ratekp_cmd,   {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "NTPKP",    "fixes - timebase rate loop time constant", ntpkp_cmd,  {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "WIFI",     "ssid password  (quote if spaces)",                          wifi_cmd,     {TINYCL_PARM_STR, TINYCL_PARM_STR, TINYCL_PARM_END} },
  { "NTP",      "hostname",                               ntp_cmd,      {TINYCL_PARM_STR, TINYCL_PARM_END} },
  { "SYNC",     "ask for an NTP exchange now",            sync_cmd,     {TINYCL_PARM_END} },
  { "NET",      "interfaces, servers and their counters",  net_cmd,      {TINYCL_PARM_END} },
  { "CHIME",    "hour minute second - heard it strike, now", chime_cmd, {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "CHIMESET", "interval_min press_allowance_ms strike_offset_s", chimeset_cmd,
                {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "TZ",       "minutes from UTC (-300 = UTC-5)",        tz_cmd,       {TINYCL_PARM_INT, TINYCL_PARM_END} },
  { "AP",       "y|n - raise the setup access point",     ap_cmd,       {TINYCL_PARM_BOOL, TINYCL_PARM_END} },
  { "APKEY",    "password for the setup access point",    apkey_cmd,    {TINYCL_PARM_STR, TINYCL_PARM_END} },
  { "HOSTNAME", "name advertised over mdns",              hostname_cmd, {TINYCL_PARM_STR, TINYCL_PARM_END} },
  { "SAVE",     "write configuration to flash",           save_cmd,     {TINYCL_PARM_END} },
  { "RECREATE", "print the commands that rebuild this config", recreate_cmd, {TINYCL_PARM_END} },
  { "DEFAULTS", "load defaults into ram",                 defaults_cmd, {TINYCL_PARM_END} },
  { "BOOTSEL",  "reboot into the uf2 bootloader",         bootsel_cmd,  {TINYCL_PARM_END} },
};

static int help_cmd(int args, tinycl_parameter *tp, void *v)
{
  (void)args; (void)tp; (void)v;
  tinycl_print_commands(sizeof(tcmds) / sizeof(tinycl_command), tcmds);
  return 1;
}

uint32_t cli_log_seq(void) { return log_seq; }

uint32_t cli_log_read(uint32_t from, char *out, uint32_t outsz, uint32_t *next)
{
  uint32_t avail, want, start, n = 0u;

  *next = log_seq;
  if (outsz == 0u) return 0u;
  if (from > log_seq) from = log_seq;            /* a restarted device */

  avail = log_seq - from;
  if (avail > LOG_SIZE) avail = LOG_SIZE;        /* the rest has scrolled away */
  want  = (avail < outsz - 1u) ? avail : (outsz - 1u);

  start = (log_head + LOG_SIZE - want) % LOG_SIZE;
  while (n < want)
  {
    out[n] = log_buf[(start + n) % LOG_SIZE];
    n++;
  }
  out[n] = '\0';
  return n;
}

bool cli_run_captured(const char *cmd, char *out, uint32_t outsz, uint32_t *outlen)
{
  char line[TINYCL_COMMAND_BUFFER];
  bool echo_was = tinycl_do_echo;
  int  guard;

  *outlen = 0;
  if (!cmd || !out || outsz < 2u) return false;

  snprintf(line, sizeof(line), "%s\r", cmd);

  cap_buf = out; cap_sz = outsz; cap_len = 0; cap_trunc = false;
  feed_p = line;
  tinycl_do_echo = 0;                    /* or the command echoes into itself */
  tinycl_set_getchar(feed_getchar, NULL);
  cap_on = true;

  /* One call should consume the whole line; the guard is only in case a
     command is split across reads. */
  for (guard = 0; guard < 4 && *feed_p != '\0'; guard++)
    tinycl_task(sizeof(tcmds) / sizeof(tinycl_command), tcmds, NULL);
  tinycl_task(sizeof(tcmds) / sizeof(tinycl_command), tcmds, NULL);

  cap_on = false;
  tinycl_set_getchar(tinycl_rppico_getchar, NULL);
  tinycl_do_echo = echo_was;

  if (cap_trunc && cap_len + 24u < cap_sz)
    cap_len += (uint32_t)snprintf(out + cap_len, cap_sz - cap_len,
                                  "\r\n[output truncated]\r\n");
  out[cap_len] = '\0';
  *outlen = cap_len;
  cap_buf = NULL;
  return true;
}

void cli_init(void)
{
  stdio_set_driver_enabled(&cap_driver, true);
  tinycl_do_echo = 1;
  printf("\r\nSynchronizer - pendulum clock discipline\r\n");
  printf("type HELP for commands.  Arguments are separated by spaces;\r\n"
         "quote any that contain one, as in: WIFI \"my net\" \"my key\"\r\n> ");
}

void cli_poll(void)
{
  if (tinycl_task(sizeof(tcmds) / sizeof(tinycl_command), tcmds, NULL))
  {
    tinycl_do_echo = 1;
    tinycl_put_string("> ");
  }
}
