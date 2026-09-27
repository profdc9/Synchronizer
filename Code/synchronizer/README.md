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

### Host tests

The part of the firmware most likely to be quietly wrong is the arithmetic
in `control.c`, and it is also the part hardest to check on the bench — a
swinging pendulum takes minutes per measurement and never repeats exactly.
So `test/` compiles `src/control.c` *unmodified* against stub headers and
feeds it synthetic swings with a known answer:

```sh
make -C test check
```

It checks that a persistently fast clock trips a retard and a persistently
slow one trips an advance, each releasing again once the pulses pull the
error back past zero; that a schedule slip forces a hold and the loop
reacquires afterward; and that a direction left at zero width trips but
never actually fires or releases. Run it after touching the loop; it takes
a second.

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
through R6. Its job is damping the drive circuit's oscillations, not setting
a current limit by design, so its value has to be picked for the coil
actually wound: a lower-resistance coil wants a smaller R6, and in many
cases R6 can be 0 Ω outright, while a higher-resistance coil can tolerate,
and may need, more damping. Check the current schematic rather than
assuming a number here. Until the coil's DC resistance is measured and R6
sized for it, the current is unknown, and a stuck pulse could put several
watts into that resistor. So the firmware bounds every pulse three
independent ways: a ceiling of 100 ms on
any single pulse, a separate watchdog checked every 5 ms that forces the pin
low regardless of what the rest of the code believes, and a token bucket
that caps the long-run average duty at 2%. The pin is driven low
in `drive_init()` before anything else runs. Default pulse width is a timid
2 ms — widen it once you know what the coil draws.

**Nothing acts until you say so.** `control_enabled` defaults off, and even
once it is on, `PW`'s two widths gate whether either direction can ever
fire - 0 refuses that direction outright rather than falling back to a
guessed width. A fresh board will sit there sensing and telling you what it
sees; KICK needs no calibration step before it can act, just a nonzero
width for whichever direction(s) you want it to use.

## Bring-up, in order

**A firmware update can silently reset everything below.** The config sector
carries a magic number and a version; if a build changes the struct layout,
the device restores defaults on its next boot rather than reading a stale
layout — deliberately, since the alternative is misinterpreting bytes that
used to mean something else. That is the right call for tuning values, which
are guesses anyway, but it means `THRESH`, `WINDOWS`, `PW`, `PTIME`, `KICK`
and everything else below can vanish after a routine reflash with no warning
beyond `STATUS` quietly showing different numbers than you left it with. If a
clock that was working stops acquiring or measuring right after an update,
suspect this before suspecting the update's actual change — walk back through
the steps below rather than assuming the new code broke something.

**Run `RECREATE` before flashing new firmware.** It prints the sequence of
commands that would rebuild the current configuration from defaults —
`GEOMETRY`, `WINDOWS`, `PW`, `KICK`, and everything else that has been
tuned, `SAVE` included. It does not print `WIFI`; credentials are
deliberately left out of it. Save the output before a version bump wipes
it, and replaying it is a one-paste recovery instead of redoing bring-up
from scratch.

**0. Pick the tank components.** The inductive sense works best somewhere in
30-100 kHz. If you don't already know the sense coil's inductance, measure it
directly with an LCR meter, or estimate one you're winding yourself from
`L = mu_0 N^2 A / l` (turns squared, cross-sectional area, over coil length).
Either way, solve for C3 to land on a target frequency f in that band:

```
C = 1 / (4 pi^2 f^2 L)
```

A typical C3 lands somewhere between 1000 pF and 10000 pF for coils in the
few-mH range tuned into 30-100 kHz — worth a sanity check against the
calculation before ordering parts; a result far outside that band usually
means the wrong L or the wrong target frequency, not an unusual coil.

C3 is not the only capacitance across the tank, though: C6 taps the tank to
feed the amplifier — lightly enough that it does not load it down — but it
still adds its own value in parallel with C3, currently 30 pF on the
development board, so account for it (`C3 + C6`) when solving for a target
frequency, not C3 alone.

Pick the nearest standard capacitor value - an exact match is not needed,
because the next step measures the tank's *actual* resonance rather than
trusting the calculation. The development board's own sense coil is 2.5 mH on
a square 25x25 mm form, for reference.

**Use a film capacitor for C3, or a ceramic NP0/C0G type - not a typical
X7R.** X7R's capacitance shifts substantially with temperature and applied
voltage, which is exactly what a frequency-determining tank component cannot
tolerate; the tank frequency will drift and the whole detector chain rides on
that frequency staying put.

**1. Calibrate the tank.** With the sense coil on J3 and **nothing metallic
near it** — not the bob, not your hand, not a steel rule on the bench:

```
RESONANCE 20000 120000 Y
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
whole band by hand, and `CAPTURE 500000 512` dumps the amplified waveform
from ADC1 so you can see what the LM358 is actually producing.

**2. Place the coil.** The tank peak found in step 1 is *not* where the
detector should sit. The bob's eddy losses drag the loaded resonance
downward, so the drive wants to be below the unloaded peak — on the
development clock, about 270 Hz below, worth 20% more signal. That optimum
moves whenever the coil moves, so let the board find it:

```
MODSCAN 0 0 0
```

It sweeps the drive, watches each frequency for longer than a full swing,
and reports how far the bob moves the envelope. **The peak-to-peak figure is
the entire signal budget** — maximise it by moving the coil, re-running after
each move. It leaves the drive on the winner.

**The recommended default geometry is opposite extremes**, and it is worth
starting there rather than exploring alternatives: sense coil at one extreme,
drive coil at the other. The photos in the top-level
[README](../../README.md#physical-setup) show the two coils placed exactly
this way. Place each coil about a quarter of the bob's own
diameter *beyond* where the bob's metal actually reaches at that extreme —
not centred on the point of deepest penetration. At the true turning point
the bob is nearly stationary, which is what produces a wide, poorly-localised
dip; offsetting the coil means it only sees the bob while it is still moving,
which gives a narrower, more sharply-timed crossing for the sense coil, and
lets the drive coil work through its *lateral fringe field* rather than a
direct, on-axis pull. "Bob diameter" here means the extent of the metal, not
any outer decorative dimension — the development clock's bob is a steel core
inside a brass envelope, so it is the full brass-covered diameter that
matters, since eddy-current sensing responds to any conductor and the brass
alone would still register even though it is not itself magnetic.

The drive coil benefits in particular from a design that leans into the
fringe field rather than fighting it: a commercial electromagnet meant for
attaching to metal, with a steel bolt glued to its own centre core so the
field extends past the coil's physical edge, measurably outperformed direct
placement attempts on the development clock — an evening of no measurable
authority at all became a clearly significant one purely from this change,
nothing else. Start here rather than rediscovering it by trial and error.

Geometry matters more than fine positioning. The bob's distance from a coil
at the swing *extreme* varies by twice the amplitude; at the *centre of
travel*, only by one amplitude. On the development clock that was 206 counts
against 75 — nearly 3:1 in favour of the extreme. Against that, the centre
gives two passes per period and the bob is moving fastest there. Try both;
`TRACE` tells you which you have, since a coil at the centre shows two
cycles per period and one at an extreme shows one.

Coil diameter is a genuine compromise: sensing depth grows with coil radius,
so too small cannot reach, while spatial resolution is also set by diameter,
so too large smears the pass into a slow sinusoid. Comparable to the standoff
is a good rule. Note that if the coil, the standoff and the bob's total
travel are all of similar size — 25 mm each on the development clock — the
bob never leaves the coil's field and the envelope is a smooth sinusoid at
the swing frequency rather than a sharp pass. That is geometry, not
misplacement, and it times perfectly well.

**3. Set the threshold.** `THRESH` is in ADC counts of envelope excursion
away from the tracked baseline, in the direction `DIR` selects. It decides
three things at once, which is why the extremes are both bad:

* whether a swing triggers at all,
* how wide the event is — a low threshold means crossing near the top of the
  dip and staying below it for most of the period,
* and the timing precision, since both edges are timed at exactly this level
  and precision is noise divided by the slope where they cross.

Run `WATCH Y` and look at the printed `peak`, which is the excursion the bob
actually produces. **Half of it is about right.** On the development clock,
peak ≈ 160 counts:

```
 THRESH  events  gap median   gap sd   width   misses
     25      17     857.3 ms     2.68   477 ms        2
     50      19     858.3        5.19   447          0
     90      19     856.6        2.24   324          0
    110      19     857.4        2.53   247          0
```

At 25 the events ran 477 ms wide against the 514 ms `max_event_pct` limit,
so one swing in ten was abandoned as stuck-high. At 90 — 56% of peak — no
misses and the lowest jitter. Too high is bad too: near the dip's floor the
slope flattens again and normal amplitude variation starts causing misses.

**If it stops acquiring entirely — not one swing in ten, every one of
them — the loop can look dead while the signal underneath is perfectly
healthy.** `STATUS` still shows `events` frozen at whatever it was, `chatter`
climbing steadily, and `baseline`/`last sample` both clearly alive and
moving. That combination — chatter increasing, `events` not — means events
are opening and never closing: every single one is running past
`max_event_pct` and getting abandoned as stuck-high before it can complete,
so none of them ever reach the width gate to be counted, rejected, or even
logged by `WATCH` (its echo only fires on events `sense.c` actually reports).
This is the same failure as the table above, just at 100% instead of 10% —
most often seen right after a reflash wiped `WINDOWS` back to its defaults
while the coil geometry underneath had since changed enough that the old
percentages no longer fit.

Diagnose it with `TRACE`, not by guessing new `WINDOWS` numbers: it plots the
raw envelope against time, bypassing the threshold/event state machine
entirely, so it shows the true dip regardless of whether detection is
currently working at all. Read the actual event width and gap straight off
the printed trace — where the excursion crosses `THRESH` on the way in to
where it recrosses `THRESH` minus `HYST` on the way out is the width; from
that crossing to the next event's crossing is the gap — and set:

```
WINDOWS <rearm%> <min%> <max%>
```

so `max%` clears the real width with margin and `rearm%` sits comfortably
*under* the real gap, not just under the nominal interval. A width that now
runs 60-70% of the period rather than the ~30-40% the defaults assume is not
a sign of a broken swing; it is what the recommended opposite-extremes-plus-
offset geometry in step 2 produces once the coil is close enough to have real
authority, and `WINDOWS` exists precisely so this is a one-line fix rather
than a rebuild.

`HYST` sets how far *below* the threshold the excursion must fall before the
event is declared over. It costs nothing in timing — both edges are still
timed at `THRESH` — and it stops ripple near the threshold from ending an
event early, which the width gate would then throw away entirely. 30% is a
good default; `STATUS` counts how often it saves a swing.

**A warning about the diagnostics.** `ENV`, `TRACE` and `MODSCAN` take the
ADC away from the detector while they run, so the detector is blind for
their duration and the loop will miss those swings. That is fine when you
are setting up, but do not leave one running and expect the loop to hold
lock, and do not trust `measured interval` until they have aged out.

**4. Get time.** 

```
WIFI myssid mypassword
SAVE
```

Credentials live in flash, not in the source. `STATUS` shows the association
and the NTP fix count. The timebase needs a few minutes and at least two
fixes before it starts correcting the RP2040 crystal's own error, which is
around 30 ppm — 2.6 s/day, a quarter of what we are trying to remove.

**5. Let the loop lock without acting.**

```
PW 0 0
CONTROL Y
```

With both pulse widths zero the loop tracks but never fires — KICK still
updates its active/idle state and `STATUS`'s `uncorrected error`, it just
never reaches `fire_for()`. Let it sit and watch `uncorrected error` walk at
the clock's natural rate — about 11 s/day fast, from the audio measurement —
before turning either direction on.

**6. Place the drive pulse.** `GEOMETRY events_per_swing drive_offset_ppt`
sets how long after a sense event the bob reaches the drive coil, as parts
per thousand of a full period - 500 for opposite extremes, the recommended
default. Treat that as a starting guess, not a fact: get a rough number by
timing the onboard LED (it blinks once per sense event) against the drive
coil by eye or stopwatch, convert to ppt, and set it with `GEOMETRY`.

`PTIME advance_us retard_us` nudges the placement by hand if that
rough centre needs adjusting. Both parameters are the same signed offset
from that centre - a given number means the same physical instant whether it
is stored as the advance placement or the retard one. What matters here is
the *sign*: a pulse fired on the wrong side of the bob's turning point
advances when it should retard, or the reverse - not the exact instant
within the half-period, since KICK's hysteresis works with whatever
authority a reasonably-placed pulse actually has, rather than needing that
authority measured in advance.

**Anything that physically disturbs the pendulum invalidates the rate
tracker**, not just the count of swings it has seen — touching the coil,
running `COILTEST`, adjusting the drive placement by hand. A disturbance
leaves a contaminated `rate_q` that only washes out over roughly `2 * RATEKP`
swings on its own; reissuing the same `RATEKP` value forces a clean restart
instead of waiting it out, and is worth doing on reflex after any physical
change.

**7. Turn a direction on and watch it run.**

```
PW 2000 2000
KICK 5 25
CONTROL Y
```

`PW advance_us retard_us` sets how wide each pulse is; 0 disables that
direction outright rather than falling back to a guessed width - a
direction the coil cannot usefully drive should simply not fire. `KICK
min_swings threshold_pct` sets how many swings must pass between pulses,
and how far past zero - as a percentage of one swing period - the
uncorrected error has to go before a direction is judged to need
correcting; it releases again as soon as the error crosses back past zero,
not out to the opposite threshold. There is no separate advance-mode or
retard-mode to pick - KICK reacts to whichever side the error is actually
on, and can run both directions at once if both widths are nonzero.

There is no bench measurement left to run first, and nothing to `SAVE` a
number from - watch it over real time instead. `STATUS`'s `uncorrected
error` should walk toward the threshold, trip a `kick`, and walk back
toward zero over the next few pulses; `locked for` counts how long that has
been happening without an interruption. `FORCEERR <us>` can confirm KICK
trips and releases in the right direction on demand, but it cannot hold an
artificial error in place long enough to judge a pulse width by - the real
measurement against NTP overwrites it within about one swing (see the doc
comment above `forceerr_cmd` in `cli.c`).

Leaving one width at zero is legitimate: the loop then corrects in one
direction only, which is all a clock that consistently gains (or
consistently loses) ever needs.

**A sensor artifact worth knowing about, if a bench-measurement tool is
ever rebuilt.** An earlier version of this loop *did* measure how many
nanoseconds of phase one pulse was worth, by firing pulses for a fixed
run and fitting the phase shift against the pendulum's own rate tracker.
That measurement turned up a real trap, and the trap outlives the tool
that found it:

A placement sweep that comes back with a smooth, coherent
placement-vs-kick curve - one sign across half the range, the other sign
across the rest, exactly the shape a real coil authority curve should
have - is not on its own evidence that the coil is doing anything. Run
the same sweep with the magnet moved away from the clock entirely; if a
similar curve still comes back, none of it is mechanical.

The suspected cause was that firing the drive coil disturbs `sense.c`'s
demod directly (supply sag, ground bounce, or coupling straight into the
tank coil), and since that demod is a delay-and-multiply phase detector
built on an I/Q accumulator that runs every ADC tick and is never reset, a
glitch on even a few samples shows up as a step that decays only over the
accumulator's own multi-swing time constant - indistinguishable from a
genuine kick. The fix tried was a guard window (`SENSEDEAD <us>`) that
stopped feeding samples into the accumulator for that long on either side
of every pulse.

It backfired. Measuring the same sweep at three guard widths -

```
SENSEDEAD 30000    ->  best-magnitude points around +-25000 ns/pulse
SENSEDEAD 10000    ->  the same shape, roughly a third the size
SENSEDEAD 0        ->  every point within about 1 sigma of zero, no shape at all
```

- showed the "fix" was the dominant source of the curve, scaling with the
guard width rather than shrinking it. The reason is structural, not specific
to this hardware: `dm_I`/`dm_Q` is a single-bin correlator estimating the
Fourier coefficient of the envelope at one cycle per swing, and removing a
fixed window from that correlation sum every cycle does not just lower its
gain — the product of two sinusoids splits into a DC term and a
second-harmonic term, and the second-harmonic part integrated over just the
missing window does not cancel. Its phase depends on *where* the window
sits, not on the true phase being measured, so blanking manufactures a bias
that tracks pulse placement directly — worse for a sharp, non-sinusoidal dip
(real harmonic content to feed the bias) than for a clean sinusoid, and
worse the wider the guard window. `SENSEDEAD` was removed rather than
defaulted to 0 and left in the config, since a knob whose only well-tested
setting is "off" is a trap for the next person tuning this board.

With no blanking, the magnet-away curve did not clearly survive above noise
either. So whatever the original electrical effect is, if it exists at all
it is smaller than that measurement could resolve, and it was very unlikely
to matter next to a much bigger problem: an attract-only coil at a
fringe-field placement measured *microseconds* of authority per pulse
against a clock that needs *~100 microseconds per swing* to correct - over
an order of magnitude short regardless of how cleanly it was measured. KICK
sidesteps the whole question by never needing that number in the first
place; this is preserved for whoever next builds a tool that does.

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
  that. `CAPTURE 500000 256` and a full `SWEEP` both fit; larger dumps want
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
TANK     55000      tank drive frequency (RESONANCE finds this)
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
