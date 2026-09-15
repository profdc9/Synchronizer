/* sense.c */

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
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "board.h"
#include "config.h"
#include "sense.h"
#include "timebase.h"

/* Baseline tracking.  At 1 kHz a shift of 12 is a time constant of about
   four seconds - long compared with the 0.2 s bump, short enough to follow
   supply and thermal drift.  It is frozen while an event is in progress so
   the event cannot drag the baseline after itself. */
#define BASELINE_SHIFT      12

/* The detector's three timing windows used to be fixed microsecond
   constants, which quietly assumed a fast pendulum: a seconds pendulum
   would have had every event rejected.  They are now percentages of the
   expected interval between sense events, resolved here whenever the
   configuration changes. */
static uint32_t win_rearm_us     = 300000u;
static uint32_t win_min_event_us = 15000u;
static uint32_t win_max_event_us = 500000u;

static volatile sense_event ring[SENSE_RING];
static volatile uint32_t    ring_head, ring_tail;
static volatile uint32_t    ev_count, overruns;
static volatile uint16_t    last_sample;
static volatile int32_t     baseline_q;         /* baseline << BASELINE_SHIFT */
static volatile bool        running;
static volatile bool        adc_busy;           /* a capture owns the ADC */
static volatile uint64_t    last_event_us;

static repeating_timer_t    samp_timer;
static uint32_t             tank_hz;
static uint32_t             tank_div = 1u;   /* PWM clock divider in use */
static uint                 pwm_slice, pwm_chan;
static uint32_t             tank_level;      /* PWM counts actually set */

/* in-progress event */
static bool     in_event;
static uint64_t rise_us, prev_us;
static uint16_t prev_sample;
static uint16_t ev_peak;
static uint16_t ev_baseline;

/* interval history for sense_mean_interval_us */
#define IVAL_RING 64
static volatile uint32_t ivals[IVAL_RING];
static volatile uint32_t ival_n, ival_head;

void sense_refresh_timing(void)
{
  uint64_t iv_us = cfg_event_interval_ns() / 1000ull;
  uint32_t r, mn, mx;

  if (iv_us < 1000ull) iv_us = 1000ull;      /* refuse absurd geometry */

  r  = (uint32_t)((iv_us * (cfg.rearm_pct     ? cfg.rearm_pct     : 35u)) / 100u);
  mn = (uint32_t)((iv_us * (cfg.min_event_pct ? cfg.min_event_pct : 2u))  / 100u);
  mx = (uint32_t)((iv_us * (cfg.max_event_pct ? cfg.max_event_pct : 60u)) / 100u);

  if (mn < 1000u) mn = 1000u;
  if (mx <= mn)   mx = mn * 4u;

  win_rearm_us     = r;
  win_min_event_us = mn;
  win_max_event_us = mx;
}

uint32_t sense_rearm_us(void)     { return win_rearm_us; }
uint32_t sense_min_event_us(void) { return win_min_event_us; }
uint32_t sense_max_event_us(void) { return win_max_event_us; }

static void tank_apply(uint32_t hz)
{
  uint32_t sysclk = clock_get_hz(clk_sys);
  uint32_t div    = 1;
  uint32_t wrap;

  if (hz < 100u) hz = 100u;
  while ((sysclk / (div * hz)) > 65535u) div++;
  wrap = (sysclk / (div * hz));
  if (wrap < 2u) wrap = 2u;

  pwm_set_clkdiv_int_frac(pwm_slice, (uint8_t)(div > 255 ? 255 : div), 0);
  pwm_set_wrap(pwm_slice, (uint16_t)(wrap - 1u));
  {
    /* A width in nanoseconds, rounded - see config.h.  Zero parks the
       output low, which is the only way to see what the amplifier does
       with no signal at all. */
    uint32_t lvl = 0u;
    tank_div = div;
    if (cfg.tank_drive_ns > 0u)
    {
      uint64_t counts = ((uint64_t)cfg.tank_drive_ns * (uint64_t)sysclk
                         + 500000000ull) / 1000000000ull;
      lvl = (uint32_t)(counts / div);
      if (lvl < 1u)        lvl = 1u;         /* ask for drive, get a count  */
      if (lvl > wrap / 2u) lvl = wrap / 2u;  /* half a period is full drive */
    }
    pwm_set_chan_level(pwm_slice, pwm_chan, (uint16_t)lvl);
    tank_level = lvl;
  }
  tank_hz = sysclk / (div * wrap);
}

static void push_event(uint64_t t_us, uint16_t peak, uint16_t base, uint32_t width)
{
  uint32_t head = ring_head;
  uint32_t next = (head + 1u) % SENSE_RING;

  if (next == ring_tail) { overruns++; ring_tail = (ring_tail + 1u) % SENSE_RING; }

  ring[head].seq      = ++ev_count;
  ring[head].t_us     = t_us;
  ring[head].utc_ns   = tb_have_time() ? tb_utc_ns() - (time_us_64() - t_us) * 1000ull : 0ull;
  ring[head].peak     = peak;
  ring[head].baseline = base;
  ring[head].width_us = width;
  ring_head = next;

  if (last_event_us != 0)
  {
    uint32_t d = (uint32_t)(t_us - last_event_us);
    ivals[ival_head] = d;
    ival_head = (ival_head + 1u) % IVAL_RING;
    if (ival_n < IVAL_RING) ival_n++;
  }
  last_event_us = t_us;
}

/* Linear interpolation of the instant a threshold was crossed between two
   consecutive samples.  Returns a time between prev_us and now_us. */
static uint64_t cross_time(uint64_t t0, uint64_t t1, int32_t v0, int32_t v1, int32_t thr)
{
  int32_t den = v1 - v0;
  if (den == 0) return t1;
  int64_t num = (int64_t)(thr - v0) * (int64_t)(t1 - t0);
  int64_t off = num / den;
  if (off < 0) off = 0;
  if (off > (int64_t)(t1 - t0)) off = (int64_t)(t1 - t0);
  return t0 + (uint64_t)off;
}

static bool sample_cb(repeating_timer_t *rt)
{
  uint64_t now;
  uint16_t s;
  int32_t  base, dev, thr;

  (void)rt;
  if (!running || adc_busy)
  {
    prev_us = time_us_64();     /* keep the interpolation interval honest */
    in_event = false;
    return true;
  }

  adc_select_input(ADC_CH_AMPLITUDE);
  s   = (uint16_t)adc_read();
  now = time_us_64();
  last_sample = s;

  if (baseline_q == 0) baseline_q = ((int32_t)s) << BASELINE_SHIFT;
  base = baseline_q >> BASELINE_SHIFT;

  /* Signed excursion in the direction the bob is expected to push it. */
  dev = cfg.detect_falling ? (base - (int32_t)s) : ((int32_t)s - base);
  thr = (int32_t)cfg.detect_threshold;

  if (!in_event)
  {
    {
      /* Signed, or a sample below the baseline wraps and the filter blows up. */
      int32_t e = (((int32_t)s) << BASELINE_SHIFT) - baseline_q;
      baseline_q += e >> BASELINE_SHIFT;
    }

    if (dev >= thr && (last_event_us == 0u || (now - last_event_us) > win_rearm_us))
    {
      int32_t pdev = cfg.detect_falling ? (base - (int32_t)prev_sample)
                                        : ((int32_t)prev_sample - base);
      in_event    = true;
      ev_peak     = (uint16_t)dev;
      ev_baseline = (uint16_t)base;
      rise_us     = cross_time(prev_us, now, pdev, dev, thr);
    }
  }
  else
  {
    if (dev > (int32_t)ev_peak) ev_peak = (uint16_t)dev;

    if (dev < thr)
    {
      int32_t pdev = cfg.detect_falling ? (ev_baseline - (int32_t)prev_sample)
                                        : ((int32_t)prev_sample - ev_baseline);
      uint64_t fall_us = cross_time(prev_us, now, pdev, dev, thr);
      uint32_t width   = (uint32_t)(fall_us - rise_us);

      in_event = false;
      if (width >= win_min_event_us && width <= win_max_event_us)
        push_event(rise_us + width / 2u, ev_peak, ev_baseline, width);
    }
    else if ((now - rise_us) > win_max_event_us)
    {
      /* Stuck high - abandon the event and let the baseline re-acquire. */
      in_event   = false;
      baseline_q = ((int32_t)s) << BASELINE_SHIFT;
    }
  }

  prev_sample = s;
  prev_us     = now;
  return true;
}

void sense_init(void)
{
  gpio_set_function(GPIO_OSCIL, GPIO_FUNC_PWM);
  pwm_slice = pwm_gpio_to_slice_num(GPIO_OSCIL);
  pwm_chan  = pwm_gpio_to_channel(GPIO_OSCIL);
  pwm_set_phase_correct(pwm_slice, false);
  tank_apply(cfg.tank_hz);
  pwm_set_enabled(pwm_slice, true);

  adc_init();
  adc_gpio_init(GPIO_AMPLITUDE);
  adc_gpio_init(GPIO_OSC_SIGNAL);

  ring_head = ring_tail = ev_count = overruns = 0;
  baseline_q = 0; in_event = false; last_event_us = 0;
  ival_n = ival_head = 0;
  prev_us = time_us_64(); prev_sample = 0;
  running = cfg.sense_enabled != 0;

  sense_refresh_timing();

  {
    uint32_t hz = cfg.sample_hz ? cfg.sample_hz : SENSE_SAMPLE_HZ;
    if (hz < 100u)   hz = 100u;
    if (hz > 20000u) hz = 20000u;
    add_repeating_timer_us(-(int64_t)(1000000u / hz), sample_cb, NULL, &samp_timer);
  }
}

void sense_set_tank_hz(uint32_t hz) { tank_apply(hz); cfg.tank_hz = tank_hz; }
uint32_t sense_tank_hz(void) { return tank_hz; }

void sense_set_drive_ns(uint32_t ns)
{
  cfg.tank_drive_ns = ns;
  tank_apply(tank_hz);            /* re-arm the slice with the new level */
}

uint32_t sense_drive_ns(void)    { return cfg.tank_drive_ns; }
uint32_t sense_drive_level(void) { return tank_level; }

/* What the timer will actually produce, which is not what was asked for
   once the request is down to a few counts. */
uint32_t sense_drive_actual_ns(void)
{
  uint32_t sysclk = clock_get_hz(clk_sys);
  return (uint32_t)(((uint64_t)tank_level * tank_div * 1000000000ull)
                    / (uint64_t)sysclk);
}
void sense_enable(bool on) { running = on; if (!on) in_event = false; }
bool sense_enabled(void) { return running; }

bool sense_next_event(sense_event *out)
{
  if (ring_tail == ring_head) return false;
  memcpy(out, (const void *)&ring[ring_tail], sizeof(sense_event));
  ring_tail = (ring_tail + 1u) % SENSE_RING;
  return true;
}

uint16_t sense_baseline(void)  { return (uint16_t)(baseline_q >> BASELINE_SHIFT); }
uint16_t sense_last_sample(void) { return last_sample; }
uint32_t sense_event_count(void) { return ev_count; }
uint32_t sense_overrun_count(void) { return overruns; }
uint64_t sense_last_event_us(void) { return last_event_us; }

uint64_t sense_mean_interval_us(uint32_t n)
{
  uint64_t sum = 0;
  uint32_t have = ival_n, i, idx;

  if (n > have) n = have;
  if (n == 0u) return 0;
  for (i = 0; i < n; i++)
  {
    idx = (ival_head + IVAL_RING - 1u - i) % IVAL_RING;
    sum += ivals[idx];
  }
  return sum / n;
}

/* --- bring-up tools ---------------------------------------------------- */

void sense_sweep(uint32_t from_hz, uint32_t to_hz, uint32_t step_hz, uint32_t dwell_ms)
{
  uint32_t saved = tank_hz;
  bool     was   = running;
  uint32_t f;
  uint32_t best_f = from_hz, best_v = 0, worst_v = 0xFFFFFFFFu, worst_f = from_hz;

  if (step_hz == 0u) step_hz = 100u;
  if (dwell_ms == 0u) dwell_ms = 20u;

  running = false;
  printf("     hz    adc   mV\r\n");
  for (f = from_hz; f <= to_hz; f += step_hz)
  {
    uint32_t acc = 0, i;
    watchdog_update();          /* a wide sweep outruns the eight seconds */
    tank_apply(f);
    sleep_ms(dwell_ms);
    adc_select_input(ADC_CH_AMPLITUDE);
    for (i = 0; i < 64u; i++) { acc += adc_read(); sleep_us(50); }
    acc /= 64u;
    printf("%7lu %6lu %5lu\r\n", (unsigned long)tank_hz, (unsigned long)acc,
           (unsigned long)((acc * 3300u) / 4095u));
    if (acc > best_v)  { best_v = acc;  best_f = tank_hz; }
    if (acc < worst_v) { worst_v = acc; worst_f = tank_hz; }
  }
  printf("peak %lu at %lu hz, trough %lu at %lu hz\r\n",
         (unsigned long)best_v, (unsigned long)best_f,
         (unsigned long)worst_v, (unsigned long)worst_f);

  tank_apply(saved);
  baseline_q = 0;
  running = was;
}

void sense_envelope(uint32_t ms, sense_env_stats *out)
{
  uint64_t acc = 0ull, end;
  uint32_t n = 0u;
  uint16_t mn = 0xffffu, mx = 0u;

  memset(out, '\000', sizeof(*out));
  if (ms == 0u)   ms = 200u;
  if (ms > 2000u) ms = 2000u;      /* the watchdog is not that patient */

  /* Take the ADC off the detector for the window, the same way a scan
     does, so its timer callback does not interleave conversions. */
  adc_busy = true;
  adc_select_input(ADC_CH_AMPLITUDE);
  end = time_us_64() + (uint64_t)ms * 1000ull;
  while (time_us_64() < end)
  {
    uint16_t s = (uint16_t)adc_read();
    if (s < mn) mn = s;
    if (s > mx) mx = s;
    acc += s;
    n++;
  }
  adc_busy = false;

  out->ms      = ms;
  out->samples = n;
  out->min     = n ? mn : 0u;
  out->max     = mx;
  out->mean    = n ? (uint16_t)(acc / n) : 0u;
}

void sense_capture(uint32_t rate_hz, uint32_t count)
{
  static uint16_t buf[2048];
  int chan;
  dma_channel_config c;
  float div;
  uint32_t i;

  if (count > 2048u) count = 2048u;
  if (count == 0u)   count = 512u;
  if (rate_hz == 0u) rate_hz = 200000u;

  adc_busy = true;
  adc_run(false);
  adc_fifo_drain();
  adc_select_input(ADC_CH_OSC_SIGNAL);
  adc_fifo_setup(true, true, 1, false, false);

  /* The ADC clock is 48 MHz and one conversion takes 96 cycles, so the
     fastest it will go is 500 ksps. */
  div = 48000000.0f / (float)rate_hz;
  if (div < 96.0f) div = 96.0f;
  adc_set_clkdiv(div - 1.0f);

  chan = dma_claim_unused_channel(true);
  c = dma_channel_get_default_config(chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_ADC);
  dma_channel_configure(chan, &c, buf, &adc_hw->fifo, count, true);

  adc_run(true);
  dma_channel_wait_for_finish_blocking(chan);
  adc_run(false);
  adc_fifo_drain();
  dma_channel_unclaim(chan);

  adc_set_clkdiv(0);
  adc_busy = false;
  baseline_q = 0;

  printf("captured %lu samples at %lu hz\r\n",
         (unsigned long)count, (unsigned long)(48000000.0f / div));
  for (i = 0; i < count; i++)
    printf("%lu%s", (unsigned long)(buf[i] & 0x0FFFu), ((i % 16u) == 15u) ? "\r\n" : " ");
  if ((count % 16u) != 0u) printf("\r\n");
}

/* --- resonance calibration --------------------------------------------- */

#define SCAN_MAX      96u
#define COARSE_STEPS  96u
#define FINE_STEPS    48u
#define ADC_FULL      4095u
#define ADC_SAT       4000u       /* above this the envelope is clipping */

static uint32_t scan_hz[SCAN_MAX];
static uint16_t scan_adc[SCAN_MAX];
static uint32_t scan_n;
static sense_resonance last_res;

uint32_t sense_scan_count(void) { return scan_n; }

bool sense_scan_point(uint32_t i, uint32_t *hz, uint16_t *adc)
{
  if (i >= scan_n) return false;
  *hz = scan_hz[i]; *adc = scan_adc[i];
  return true;
}

const sense_resonance *sense_last_resonance(void) { return &last_res; }

/* Park the drive at hz, let the envelope detector settle, and average. */
static uint16_t measure_at(uint32_t hz, uint32_t dwell_ms)
{
  uint32_t acc = 0, i;
  watchdog_update();
  tank_apply(hz);
  sleep_ms(dwell_ms);
  adc_select_input(ADC_CH_AMPLITUDE);
  for (i = 0; i < 64u; i++) { acc += adc_read(); sleep_us(40); }
  return (uint16_t)(acc / 64u);
}

static uint32_t run_scan(uint32_t lo, uint32_t hi, uint32_t steps,
                         uint32_t dwell_ms, uint32_t *peak_idx)
{
  uint32_t i, best = 0;
  if (steps > SCAN_MAX) steps = SCAN_MAX;
  if (steps < 3u) steps = 3u;
  for (i = 0; i < steps; i++)
  {
    scan_hz[i]  = lo + ((hi - lo) * i) / (steps - 1u);
    scan_adc[i] = measure_at(scan_hz[i], dwell_ms);
    if (scan_adc[i] > scan_adc[best]) best = i;
  }
  *peak_idx = best;
  scan_n = steps;
  return steps;
}

/* Linear interpolation of the frequency at which the curve crosses `level`,
   walking outward from the peak.  Returns 0 if it never crosses. */
static uint32_t cross_freq(uint32_t n, uint32_t peak, uint16_t level, int dir)
{
  int32_t i = (int32_t)peak;
  for (;;)
  {
    int32_t j = i + dir;
    if (j < 0 || j >= (int32_t)n) return 0u;
    if (scan_adc[j] <= level)
    {
      int32_t num = (int32_t)scan_adc[i] - (int32_t)level;
      int32_t den = (int32_t)scan_adc[i] - (int32_t)scan_adc[j];
      int32_t df  = (int32_t)scan_hz[j] - (int32_t)scan_hz[i];
      if (den <= 0) return scan_hz[j];
      return (uint32_t)((int32_t)scan_hz[i] + (df * num) / den);
    }
    i = j;
  }
}

static void plot_scan(uint32_t n, uint16_t floor_adc, uint16_t peak_adc)
{
  uint32_t i;
  int32_t  span = (int32_t)peak_adc - (int32_t)floor_adc;

  if (span < 1) span = 1;
  printf("      hz    adc\r\n");
  for (i = 0; i < n; i++)
  {
    int32_t v = ((int32_t)scan_adc[i] - (int32_t)floor_adc) * 46 / span;
    int32_t k;
    if (v < 0) v = 0;
    if (v > 46) v = 46;
    printf("%8lu %6u |", (unsigned long)scan_hz[i], scan_adc[i]);
    for (k = 0; k < v; k++) putchar('#');
    printf("\r\n");
  }
}

bool sense_find_resonance(uint32_t lo, uint32_t hi, bool plot, sense_resonance *out)
{
  uint32_t n, peak, i;
  uint16_t floor_adc, half;
  uint32_t w_lo, w_hi, width, flo, fhi;
  bool     was = running;

  memset(out, '\0', sizeof(*out));
  scan_n = 0;
  if (hi <= lo || (hi - lo) < 100u) { printf("bad range\r\n"); return false; }

  running = false;              /* the detector must not run during a scan */

  /* --- coarse: where is it, and roughly how wide? --- */
  printf("coarse scan %lu..%lu hz\r\n", (unsigned long)lo, (unsigned long)hi);
  n = run_scan(lo, hi, COARSE_STEPS, 20u, &peak);

  floor_adc = scan_adc[0];
  for (i = 0; i < n; i++) if (scan_adc[i] < floor_adc) floor_adc = scan_adc[i];

  if (peak == 0u || peak == n - 1u) out->edge = true;

  half  = (uint16_t)(floor_adc + (((uint32_t)scan_adc[peak] - floor_adc) * 707u) / 1000u);
  w_lo  = cross_freq(n, peak, half, -1);
  w_hi  = cross_freq(n, peak, half, +1);
  width = (w_lo && w_hi) ? (w_hi - w_lo) : ((hi - lo) / 8u);
  if (width < 100u) width = 100u;

  /* --- fine: three linewidths centred on the coarse peak --- */
  {
    uint32_t c  = scan_hz[peak];
    uint32_t fl = (c > width + width / 2u) ? (c - width - width / 2u) : 100u;
    uint32_t fh = c + width + width / 2u;
    if (fl < lo) fl = lo;
    if (fh > hi) fh = hi;
    if (fh <= fl + 100u) { fl = lo; fh = hi; }
    printf("fine scan %lu..%lu hz\r\n", (unsigned long)fl, (unsigned long)fh);
    n = run_scan(fl, fh, FINE_STEPS, 30u, &peak);
  }

  for (i = 0; i < n; i++) if (scan_adc[i] < floor_adc) floor_adc = scan_adc[i];
  out->floor_adc = floor_adc;
  out->peak_adc  = scan_adc[peak];
  out->saturated = scan_adc[peak] >= ADC_SAT;

  if (plot) plot_scan(n, floor_adc, scan_adc[peak]);

  /* --- parabolic interpolation of the peak --- */
  out->f0_hz = scan_hz[peak];
  if (peak > 0u && peak < n - 1u && !out->saturated)
  {
    int32_t y0 = scan_adc[peak - 1], y1 = scan_adc[peak], y2 = scan_adc[peak + 1];
    int32_t den = y0 - 2 * y1 + y2;
    if (den != 0)
    {
      int32_t step = (int32_t)scan_hz[peak + 1] - (int32_t)scan_hz[peak];
      /* offset = 0.5 * (y0 - y2) / den, in units of one step */
      int32_t off = (step * (y0 - y2)) / (2 * den);
      if (off > step)  off =  step;
      if (off < -step) off = -step;
      out->f0_hz = (uint32_t)((int32_t)scan_hz[peak] + off);
    }
  }

  /* --- half-power points and Q --- */
  half = (uint16_t)(floor_adc + (((uint32_t)scan_adc[peak] - floor_adc) * 707u) / 1000u);
  flo  = cross_freq(n, peak, half, -1);
  fhi  = cross_freq(n, peak, half, +1);
  out->f_lo_hz = flo;
  out->f_hi_hz = fhi;
  if (flo && fhi && fhi > flo)
    out->q_x10 = (uint32_t)(((uint64_t)out->f0_hz * 10ull) / (uint64_t)(fhi - flo));

  out->valid = (scan_adc[peak] > floor_adc + 40u);

  /* A coarse grid much coarser than the resonance can step over the peak,
     and the fine window, the parabola and Q are then all built on whichever
     shoulder happened to be sampled.  Being merely comparable to the
     linewidth is fine - the response falls off smoothly either side, so the
     nearest coarse sample is still the nearest one to f0.  It goes wrong
     when the step is several linewidths and the nearest sample sits down in
     the floor.  The final width is the first honest measurement of how wide
     the peak is, so audit the coarse pass with it after the fact. */
  if (out->valid && flo && fhi && fhi > flo)
  {
    uint32_t coarse_step = (hi - lo) / (COARSE_STEPS - 1u);
    if (2u * (fhi - flo) < coarse_step)
      printf("warning: the peak is %lu hz wide but the coarse scan stepped\r\n"
             "         %lu hz - it may have missed it.  Narrow the range and\r\n"
             "         run it again to be sure.\r\n",
             (unsigned long)(fhi - flo), (unsigned long)coarse_step);
  }

  /* --- adopt it, but only if the result is worth believing ---

     A clipped peak is flat on top, so the parabolic fit slides off it and
     the half-power points sit on the wrong part of the curve.  On the
     synthetic test a railed scan came back 302 Hz off with Q wrong by half.
     Better to keep the old frequency and say so than to quietly adopt a
     bad one and save it. */
  if (out->valid && !out->saturated)
  {
    tank_apply(out->f0_hz);
    cfg.tank_hz        = tank_hz;
    cfg.tank_f0_hz     = out->f0_hz;
    cfg.tank_q_x10     = out->q_x10;
    cfg.tank_peak_adc  = out->peak_adc;
    cfg.tank_floor_adc = out->floor_adc;
  }
  else
    tank_apply(cfg.tank_hz);        /* put it back where it was */

  baseline_q = 0;
  running    = was;
  last_res   = *out;
  return out->valid && !out->saturated;
}
