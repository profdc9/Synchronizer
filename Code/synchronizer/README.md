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
loop refuses to fire even when enabled until the pulse authority has been
measured, and it can only act in a direction that has been measured. A fresh board will sit there sensing and telling you what it sees.

## Bring-up, in order

**1. Calibrate the tank.** With the sense coil on J3 and **nothing metallic
near it** — not the bob, not your hand, not a steel rule on the bench:

```
RESONANCE 2000 80000 Y
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

`SWEEP 5000 60000 250` still prints a raw table if you want to look at the
whole band by hand, and `CAPTURE 200000 512` dumps the amplified waveform
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
WIFI myssid mypassword
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

With both authorities still zero the loop tracks but never fires. Let it
sit and watch `phase error` in `STATUS` walk at the clock's natural rate —
about 11 s/day fast, from the audio measurement.

**5. Measure the loop gain.** This is the one parameter the project does not
already know:

```
MEASURE 20 Y
```

It fires one retard pulse per swing for 20 swings, subtracts the natural
drift over that interval, and reports nanoseconds of phase per pulse. Start
with a narrow `PW` and work up — an 18 cm pendulum stores very little
energy, so the coil has more authority than you might expect, and it will
disturb the swing amplitude as readily as the phase.

Advance and retard are measured separately, because they are not the same
number. An attract-only coil retards when the bob is on its side of centre
and advances only when the bob is at the far extreme, where the field is far
weaker — depending on where the coil sits the two can differ by more than an
order of magnitude. Run it both ways:

```
MEASURE 20 N        advance
MEASURE 20 Y        retard
AUTH <advance_ns> <retard_ns>
SAVE
```

Leaving one of them at zero is legitimate: the loop then corrects in one
direction only, which is all a clock that consistently gains ever needs.

**6. Close the loop.** With authority measured, the loop starts spending its
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

The serial CLI keeps working alongside it — and is also *on* the page: the
Console card runs the same command line, through the same parser and the
same command table. There is no second syntax to keep in step.

```sh
curl -s -X POST 'http://synchronizer.local/api/cli?cmd=STATUS'   # queue it
curl -s        'http://synchronizer.local/api/cli'               # read the output
```

The output comes back as plain text, so this is as pleasant from `curl` or a
script as it is from the browser. It works because the SDK's stdio layer
fans every `printf` out to all registered drivers: a capture driver sits
alongside USB and is switched on around the command, so **not one of the
firmware's `printf` calls had to change**. Output still reaches the serial
port at the same time, so a command run from a browser is visible to anyone
watching the wire.

Three things to know:

- **A command runs from the main loop, never in the HTTP handler.** The POST
  queues it and returns `202`; the page notices the result counter change in
  `/api/status` and fetches the output. `RESONANCE` alone blocks for three
  seconds, which an HTTP handler must not do.
- **Output is capped at about 4 kB** and marked `[output truncated]` past
  that. `CAPTURE 200000 256` and a full `SWEEP` both fit; larger dumps want
  the serial console.
- **`WIFI` and `APKEY` are refused here.** Credentials are deliberately
  settable only over the setup access point, and letting the console set
  them from your LAN would quietly undo that. They still work on the serial
  console.

The USB and web consoles share one `tinycl` command buffer, so typing on the
serial port at the exact moment a web command runs can garble one of them.
The result is a rejected command, not a wrong one.

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

## First-time setup, with no serial terminal

A board that has never been told a network raises **its own access point**
instead, and so does one that has failed to connect three times running —
which is what a mistyped password looks like from the inside. There is no
way to get locked out.

1. Join the Wi-Fi network **`Synchronizer-XXXX`** (the last two bytes of the
   board's unique ID). The key is **`synchronizer`**, changeable with
   `APKEY`.
2. Open any page. The device runs a DHCP server so your phone gets an
   address, and answers every DNS lookup with its own, so the captive-portal
   check fails and the setup form opens by itself. If it does not, go to
   **192.168.4.1**.
3. Pick your network from the list, or type it — scanning is best effort
   while the AP is up, so an empty list is normal and the text field always
   works.
4. Press Connect. Credentials are saved to flash, the AP drops and the
   device joins your network, where it answers at
   **http://synchronizer.local**.

`AP Y` raises the access point by hand, `AP N` drops it, and the main page
has a button for it too.

### Finding it afterwards

An mDNS responder answers for **`<hostname>.local`** — `synchronizer.local`
by default, changed with `HOSTNAME` or from the page — and advertises the
web interface as an **`_http._tcp`** service, so it also shows up in service
browsers (`avahi-browse -rt _http._tcp`, Safari's Bonjour list, Android's
NSD) without anyone knowing an address. It runs on the setup access point
too, so even provisioning works by name.

This is a convenience, not a guarantee. `.local` resolves reliably on macOS,
iOS, Windows 10 and later, and Linux with Avahi; **typing it into a browser
on Android is unreliable**, though service discovery from an app works
there. `STATUS` always prints the plain address, and so does your router's
client list.

Anything typed as a hostname is reduced to a legal DNS label — lower-cased,
with everything but letters, digits and interior hyphens dropped. The name
it settled on is echoed back, so `Papa's clock` becoming `papasclock` is
visible rather than silent.

### A note on ordering

Provisioning and raising or dropping the access point are **deferred to the
main loop**, not done in the HTTP handler that asked for them. Saving
credentials writes flash with interrupts masked, and tearing down an
interface from inside an lwIP callback would take away the very network the
reply still has to travel over. So the handler records the request, answers
the browser, and the change happens a moment later.

**Credentials are only accepted over that access point.** `net_provision()`
refuses otherwise, so nobody on your LAN can repoint the device at their own
network — they would have to be in radio range and raise the AP first.

The key on the setup AP is not much of a secret; it is written above. Its
job is to stop your home Wi-Fi password crossing an open link in the clear
while you type it in. Anyone within radio range who knows the default could
provision an unconfigured board, so change it with `APKEY` if that matters
where the clock lives.

## Setting it up for a different clock

Six commands cover everything clock-specific. Each one takes effect
immediately and `SAVE` keeps it; the config carries a magic number and a
version, so changing the structure in a future build restores defaults
rather than reading a stale layout.

```
BPH      8400 2     beats per hour, and beats per full swing
GEOMETRY 1 500      sense events per swing, drive coil offset
WINDOWS  35 2 60    detector windows, as % of the event interval
LOCK     12 4       events needed to lock, gap tolerance %
SAMPLE   1000       envelope sampling rate
TANK     23400      tank drive frequency (RESONANCE finds this)
```

**`BPH`** is the gear ratio. `beats_per_swing` is 2 for an anchor, deadbeat
or pin-pallet escapement — two pallets, one tooth released per half period.

**`GEOMETRY`** is where you put the coils, and it is the one most likely to
differ from this build:

| Sense coil | Drive coil | Command |
|---|---|---|
| at a swing extreme | at the other extreme | `GEOMETRY 1 500` |
| at a swing extreme | at the same extreme | `GEOMETRY 1 0` |
| at the swing centre | at an extreme | `GEOMETRY 2 250` |

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
