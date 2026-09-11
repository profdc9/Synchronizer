# Synchronizer firmware

Disciplines a pendulum clock to NTP time (GPS later), without modifying the
clock. Two coils sit behind the case on the pendulum's swing path, acting
through the back wall.

Nothing here assumes a particular movement. Pendulum period, escapement,
gear ratio, coil placement and detector timing are all configuration held in
flash. The defaults describe the 31-day clock it was developed against —
they are a starting point, not a specification.

Target: **Raspberry Pi Pico W**, SDK 2.x.

## Building

```sh
export PICO_SDK_PATH=/home/dmarks/pico/pico-sdk
cmake -S . -B build -G Ninja -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=RelWithDebInfo
ninja -C build
```

`build/synchronizer.uf2` is the image. The console is USB CDC at any baud
rate; the UART pins stay free for a GPS receiver on J2.

## What each piece does

| File | Role |
|---|---|
| `board.h` | Pin map, taken from the schematic. Nothing else hard-codes a pin. |
| `sense.c` | Drives the tank at resonance, samples the envelope at 1 kHz, and times each swing. |
| `drive.c` | Fires the impulse coil, with the safety limits described below. |
| `timebase.c` | A UTC model disciplined by NTP in both phase and rate. |
| `netclock.c` | WiFi association and the NTP client. |
| `control.c` | The phase-locked loop, and the sigma-delta that turns its demand into whole pulses. |
| `cli.c` | The serial command line. `HELP` lists everything. |
| `config.c` | Settings in the last flash sector, CRC-checked. |
| `tinycl.c` | Your command-line parser, reused unchanged. |

## Two things worth knowing before powering it up

**The drive coil can cook R6.** `GPIO4` high energises the coil from +12 V
through R6, a 10R resistor. Until the coil's DC resistance is measured, the
current is unknown, and a stuck pulse could put several watts into that
resistor. So the firmware bounds every pulse three independent ways: a
ceiling of 50 ms on any single pulse, a separate 5 ms watchdog timer that
forces the pin low regardless of what the rest of the code believes, and a
token bucket that caps the long-run average duty at 2%. The pin is driven low
in `drive_init()` before anything else runs. Default pulse width is a timid
2 ms — widen it once you know what the coil draws.

**Nothing acts until you say so.** `control_enabled` defaults off, and the
loop refuses to fire even when enabled until `pulse_authority_ns` has been
measured. A fresh board will sit there sensing and telling you what it sees.

## Bring-up, in order

**1. Calibrate the tank.** With the sense coil on J3 and **nothing metallic
near it** — not the bob, not your hand, not a steel rule on the bench:

```
RESONANCE 2000,80000,Y
```

Two passes. A coarse scan locates the peak and gauges its width, then a fine
scan covers about three linewidths centred on it. The peak frequency comes
from a parabolic fit to the three points around the fine maximum, so it is
not limited to the step size; the half-power points come from linear
interpolation across the fine scan. The trailing `Y` prints the curve as a
bar chart.

It reports the peak, the −3 dB points, Q, and the two steepest flanks, then
adopts the peak. `SAVE` keeps it.

On a synthetic resonance the peak comes back within a few hertz — well under
one scan step — for Q anywhere from 12 to 120 and across the whole band. Q
itself is good to about 1% at moderate Q and drifts low by roughly 10% on a
very sharp tank, where the fine scan's step size limits how precisely the
flanks can be located.

Two things to watch for, both of which the command warns about:

- **Clipping.** If the envelope rails, the peak goes flat, the parabola
  slides off it and Q collapses. On the synthetic test a railed scan came
  back 302 Hz out with Q wrong by half. The command refuses to adopt a
  clipped result and leaves the drive where it was.
- **Harmonics.** The drive is a square wave, so a scan that passes through
  f0/3 and f0/5 will show smaller responses there too. The fundamental
  always wins, but do not be surprised by the subpeaks in the plot.

**Should you sit exactly on the peak?** Not necessarily. At the peak the
amplitude is stationary, so pure detuning by the bob only moves it to
*second* order. On the steepest flank — about 0.354 bandwidths either side of
centre, which is what the command prints — detuning moves it to first order,
which can be far more sensitive. Against that, the bob also *loads* the tank
through eddy-current loss, and that lowers the peak to first order even at
centre. Which mechanism dominates depends on your coil, your frequency and
how much brass versus steel the bob presents, so it is worth trying both:
run `RESONANCE`, note the peak amplitude, then hold the bob at its closest
approach and read `STATUS` at the peak and at each flank. Take whichever
gives the biggest swing.

`SWEEP 5000,60000,250` still prints a raw table if you want to look at the
whole band by hand, and `CAPTURE 200000,512` dumps the amplified waveform
from ADC1 so you can see what the LM358 is actually producing.

**2. Watch the bob.** Put the coil behind the clock and:

```
WATCH Y
```

Every detected swing prints. You want one event per full period — about
every 857 ms — with a stable `peak` well above the threshold and a `width`
of a few tens of milliseconds. Adjust with `THRESH`, and use `DIR` if the
bob makes the amplitude rise rather than fall. `STATUS` shows the measured
swing interval against nominal.

**3. Get time.** 

```
WIFI myssid,mypassword
SAVE
```

Credentials live in flash, not in the source. `STATUS` shows the association
and the NTP fix count. The timebase needs a few minutes and at least two
fixes before it starts correcting the RP2040 crystal's own error, which is
around 30 ppm — 2.6 s/day, a quarter of what we are trying to remove.

**4. Let the loop lock without acting.**

```
CONTROL Y
```

With `pulse_authority_ns` still zero the loop tracks but never fires. Let it
sit and watch `phase error` in `STATUS` walk at the clock's natural rate —
about 11 s/day fast, from the audio measurement.

**5. Measure the loop gain.** This is the one parameter the project does not
already know:

```
MEASURE 20,Y
```

It fires one retard pulse per swing for 20 swings, subtracts the natural
drift over that interval, and reports nanoseconds of phase per pulse. Start
with a narrow `PW` and work up — an 18 cm pendulum stores very little
energy, so the coil has more authority than you might expect, and it will
disturb the swing amplitude as readily as the phase.

```
AUTH 4200000
SAVE
```

**6. Close the loop.** With authority set, the loop starts spending its
demand in whole pulses. `STATUS` shows `undelivered credit` — the correction
asked for but not yet paid out — and `pulses` counting up. At 11 s/day it
needs 113 µs of retard per swing, so expect one pulse every few dozen swings,
not one per swing.

**7. Set the hands.** `OFFSET 30000` tells the loop that "on time" is 30 s
later than it currently thinks, and it will walk the hands there at the
`SLEW` rate rather than jumping. Nobody touches the clock.

## The web interface

Once the board associates, it serves a page on port 80 at the address
`STATUS` prints. Everything the serial console does is there: live status,
the clock and coil geometry, the detector, the drive coil, the loop, and a
resonance scan that draws its own curve.

The serial CLI keeps working alongside it, and remains the only way to set
Wi-Fi credentials — they are never served or accepted over the network.

Two things shape the design:

- **A resonance scan blocks for about three seconds**, which an HTTP handler
  must not do. Requesting one only queues it; `web_poll()` runs it from the
  main loop and the page watches the `job` field until it clears, then
  fetches the curve. The same applies to writing flash.
- **`http_dispatch()` runs in lwIP callback context**, so it never
  allocates, never touches the ADC or flash, and never blocks. Each of the
  three connection slots owns its own request and response buffers.

The page lives in flash as a C string. **Edit `web/page.html`, not
`src/webpage.c`**, and regenerate:

```sh
python3 web/genpage.py          # rewrite src/webpage.c
python3 web/genpage.py --check  # fail if it is stale
```

The generator refuses to write unless it can parse its own output back into
the exact bytes of the source, so a mistake in the escaping cannot reach the
firmware silently.

**There is no authentication.** Anyone who can reach the device on your
network can energise the drive coil. The pulse ceiling, the watchdog and the
duty bucket in `drive.c` bound what that can do to the hardware, but they
are not a security boundary — put this on a network you trust, or keep it to
the serial console.

## Setting it up for a different clock

Six commands cover everything clock-specific. Each one takes effect
immediately and `SAVE` keeps it; the config carries a magic number and a
version, so changing the structure in a future build restores defaults
rather than reading a stale layout.

```
BPH      8400,2     beats per hour, and beats per full swing
GEOMETRY 1,500      sense events per swing, drive coil offset
WINDOWS  35,2,60    detector windows, as % of the event interval
LOCK     12,4       events needed to lock, gap tolerance %
SAMPLE   1000       envelope sampling rate
TANK     23400      tank drive frequency (RESONANCE finds this)
```

**`BPH`** is the gear ratio. `beats_per_swing` is 2 for an anchor, deadbeat
or pin-pallet escapement — two pallets, one tooth released per half period.

**`GEOMETRY`** is where you put the coils, and it is the one most likely to
differ from this build:

| Sense coil | Drive coil | Command |
|---|---|---|
| at a swing extreme | at the other extreme | `GEOMETRY 1,500` |
| at a swing extreme | at the same extreme | `GEOMETRY 1,0` |
| at the swing centre | at an extreme | `GEOMETRY 2,250` |

A coil at an extreme sees the bob once per full period; one at the centre
sees it twice. The offset is how long after a sense event the bob reaches
the *drive* coil, in parts per thousand of a full period.

**`WINDOWS`** are percentages of the expected interval between sense events,
not fixed microseconds, so they follow the pendulum. On a 0.857 s clock the
defaults resolve to a 300 ms rearm and a bump between 17 ms and 514 ms; on a
seconds pendulum, to 700 ms and 40 ms–1.2 s. `STATUS` shows the resolved
values.

The event schedule is exact integer arithmetic in all of these. Checked on
the host across 8400/3600/7200/5400/14400/9000 bph, one and two events per
period, and a single-beat escapement: **zero nanoseconds of error over a
year** in every case.

## The clock this was developed against

Measured from a three-minute recording of the escapement on 2026-09-10:

- full swing period **0.857030 s**, nominal 6/7 s
- **8400 bph**, 140 beats/min, **4200 full swings/hour, 100,800/day**
- pendulum about **18.3 cm**
- runs **~11 s/day fast**
- **203 ms out of beat** — stable, and mechanically fine; the escapement
  releases at 0.678 of peak swing where the bob still has 73% of its top
  speed. Curable by shimming the case if you want, but the project does not
  need it.

100,800 swings/day is the gear-ratio constant, so the clock-face calibration
is now a confirmation step rather than a prerequisite. `BPH` changes it if
this movement turns out not to be standard.
