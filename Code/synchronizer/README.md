# Synchronizer firmware

Disciplines a wind-up 31-day pendulum clock to NTP time (GPS later), without
modifying the clock. Two coils sit behind the case, on the pendulum's swing
path, at opposite extremes, acting through the wooden back wall.

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

**1. Find the tank's resonance.** With the sense coil connected to J3:

```
SWEEP 5000,60000,250
```

It steps the drive frequency and prints the envelope reading at each step.
The tank's resonance is where the amplitude peaks. Narrow in, then:

```
TANK 23400
SAVE
```

`CAPTURE 200000,512` dumps the raw amplified waveform from ADC1 if you want
to see what the LM358 is actually producing.

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

## The numbers this clock actually has

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
