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

It checks that `MEASURE` recovers the kick it was given, that the error bar
it prints matches the spread it actually has, that it refuses to call noise
a measurement, and that `PTIMESCAN` ranks placements by sign rather than by
magnitude. Run it after touching the loop; it takes a second.

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
through R6. Its value is still being tuned on the development board (it may
end up as low as 0R), so check the current schematic rather than assuming a
number here. Until the coil's DC resistance is measured, the current is
unknown, and a stuck pulse could put several watts into that resistor. So the
firmware bounds every pulse three independent ways: a ceiling of 100 ms on
any single pulse, a separate watchdog checked every 5 ms that forces the pin
low regardless of what the rest of the code believes, and a token bucket
that caps the long-run average duty at 2%. The pin is driven low
in `drive_init()` before anything else runs. Default pulse width is a timid
2 ms — widen it once you know what the coil draws.

**Nothing acts until you say so.** `control_enabled` defaults off, and the
loop refuses to fire even when enabled until the pulse authority has been
measured, and it can only act in a direction that has been measured. A fresh board will sit there sensing and telling you what it sees.

## Bring-up, in order

**A firmware update can silently reset everything below.** The config sector
carries a magic number and a version; if a build changes the struct layout,
the device restores defaults on its next boot rather than reading a stale
layout — deliberately, since the alternative is misinterpreting bytes that
used to mean something else. That is the right call for tuning values, which
are guesses anyway, but it means `THRESH`, `WINDOWS`, `PW`, `PTIME`, `AUTH`
and everything else below can vanish after a routine reflash with no warning
beyond `STATUS` quietly showing different numbers than you left it with. If a
clock that was working stops acquiring or measuring right after an update,
suspect this before suspecting the update's actual change — walk back through
the steps below rather than assuming the new code broke something.

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
drive coil at the other. Place each coil about a quarter of the bob's own
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
CONTROL Y
```

With both authorities still zero the loop tracks but never fires. Let it
sit and watch `phase error` in `STATUS` walk at the clock's natural rate —
about 11 s/day fast, from the audio measurement.

**6. Find where the drive coil actually works.** `GEOMETRY events_per_swing
drive_offset_ppt` sets how long after a sense event the bob reaches the drive
coil, as parts per thousand of a full period - 500 for opposite extremes, the
recommended default. Treat that as a starting guess, not a fact: get a rough
number by timing the onboard LED (it blinks once per sense event) against the
drive coil by eye or stopwatch, convert to ppt, and set it with `GEOMETRY`.

Then let `PTIMESCAN` do the precise search. Both `pulse_advance_us` and
`pulse_retard_us` are the *same* signed offset from that same centre - a
given number means the same physical instant whether it is stored as the
advance placement or the retard one - so either can reach a full half period
in either direction, and a sweep across the *whole* range is a reasonable
first move rather than a guess at a narrow window:

```
PTIMESCAN N -400000 400000 10 60
```

```
advance placement sweep
        us     ns/pulse       +/-  rate ppb   wrong <-- | --> working
   -400000          -86      1441      2157   [               |               ]
   -311112        -2850      1402      -658   [               |######         ]
   -222223        -3015      1587      7010   [               |######         ]
   -133334        -1801      1475      3161   [               |###            ]
    -44445        -6335      1658      6157   [               |############## ]  <-- best
     44444        -1018      1579      2405   [               |##             ]
    133333         2046      1482     -1753   [           ####|               ]
    222222         1187      1711     -3747   [             ##|               ]
    311111         3884      1513        76   [       ########|               ]
    400000         2189      1408     -3335   [           ####|               ]
placement restored to 40000 us.
'PTIME -44445 47500' then SAVE to keep the best
```

(This particular capture predates advance and retard being put on the same
sign convention - at the time, advance subtracted from centre instead of
adding, so the offsets shown here are negated relative to what today's
firmware would report for the same physical instants; the best point above
would be reached as `PTIME 44445 47500` now, not `-44445`. The method and the
fluke warning below it are unaffected.)

**Do not trust the single biggest number on its own.** A follow-up sweep
across a narrower window around -44445, with more swings per point, found the
true value there was closer to -2200 - consistent with its *neighbours* in
the wide sweep, not with the outlier reading that made it look so much
stronger. The wide sweep is for finding which *region* has real, working
authority; treat any one point in it as noisy until a second, independent
measurement near it agrees. `PW` also matters here: a short pulse may not
give the coil's current time to build up before it switches off again, so if
a region shows a real but weak effect, try lengthening `PW` before concluding
the geometry itself is at fault.

**Anything that physically disturbs the pendulum invalidates the rate
tracker**, not just the count of swings it has seen — touching the coil,
running `COILTEST`, adjusting the drive placement by hand. A disturbance
leaves a contaminated `rate_q` that only washes out over roughly `2 * RATEKP`
swings on its own; reissuing the same `RATEKP` value forces a clean restart
instead of waiting it out, and is worth doing on reflex after any physical
change, before trusting `MEASURE` or `PTIMESCAN` again.

**7. Measure the loop gain.** This is the one parameter the project does not
already know:

```
MEASURE 60 N
```

It watches for 60 swings, fires one pulse per swing for 60 swings (`N` for
advance, `Y` for retard), watches for 60 more, and reports nanoseconds of
phase per pulse — three times the pulse count in swings, and it prints the
duration before it starts. Start with a narrow `PW` and work up: a light
pendulum stores very little energy, so the coil has more authority than you
might expect, and it will disturb the swing amplitude as readily as the
phase.

The measurement rides on the pendulum rate tracker, so the loop has to have
been tracking for at least `RATEKP` swings — about five minutes — before it
will run. Through the measurement the tracker stops learning and free-runs
on the rate it had, which makes it a flywheel the pulses cannot move; what
shows up against it is what the pulses did. Each window is fitted by least
squares rather than read off its endpoints, which matters more than it
sounds: per-swing timing noise used to be a couple of milliseconds against a
kick of tens of microseconds, which is why endpoint differencing was
hopeless. The delay-and-multiply phase detector (`sense.c`) cut that noise by
roughly 9x on the development clock, so read whatever `event noise` reports
for your own hardware rather than assuming either number:

```
advance authority over 60 pulses
   event noise 151 us; windows of 60 swings either side
   rate before 170586, after 5448473 ns per 1000 swings  (moved 6157 +/- 1860 ppb)
   phase across pulsing -211 us, of which 168 us was rate
   kick -380 us -> -6335 +/- 1658 ns per pulse
```

The uncertainty falls as the pulse count to the three-halves power, so if
the answer is inside its own error bar the fix is simply `MEASURE 200 N` and
a longer wait — and see the fluke warning in step 6 before trusting a single
run regardless of how significant it looks. `rate before`/`after` is the
other half of the story: a pulse that changes the swing *amplitude* changes
the rate through circular error, and that shows up as the slope change
rather than the step. The two are in quadrature — a placement with all kick
and no rate change is the one to want.

**A second kind of fluke, and it will not go away with more swings.** A
`PTIMESCAN` that comes back with a smooth, coherent placement-vs-kick curve —
one sign across half the range, the other sign across the rest, exactly the
shape a real coil authority curve should have — is not on its own evidence
that the coil is doing anything. Run the same sweep with the magnet moved
away from the clock entirely. If a similar curve still comes back, none of it
is mechanical.

**A blanking "fix" for this was tried and made things worse, not better —
worth recording so it is not reinvented.** The suspicion was that firing the
drive coil disturbs `sense.c`'s demod directly (supply sag, ground bounce, or
coupling straight into the tank coil), and since that demod is a delay-and-
multiply phase detector built on an I/Q accumulator that runs every ADC tick
and is never reset, a glitch on even a few samples would show up as a step
that decays only over the accumulator's own multi-swing time constant —
indistinguishable from a genuine kick. The fix tried was a guard window
(`SENSEDEAD <us>`) that stopped feeding samples into the accumulator for that
long on either side of every pulse.

It backfired. Measuring the same sweep at three guard widths —

```
SENSEDEAD 30000    ->  best-magnitude points around +-25000 ns/pulse
SENSEDEAD 10000    ->  the same shape, roughly a third the size
SENSEDEAD 0        ->  every point within about 1 sigma of zero, no shape at all
```

— showed the "fix" was the dominant source of the curve, scaling with the
guard width rather than shrinking it. The reason is structural, not specific
to this hardware: `dm_I`/`dm_Q` is a single-bin correlator estimating the
Fourier coefficient of the envelope at one cycle per swing, and removing a
fixed window from that correlation sum every cycle does not just lower its
gain — the product of two sinusoids splits into a DC term and a second-
harmonic term, and the second-harmonic part integrated over just the missing
window does not cancel. Its phase depends on *where* the window sits, not on
the true phase being measured, so blanking manufactures a bias that tracks
pulse placement directly — worse for a sharp, non-sinusoidal dip (real
harmonic content to feed the bias) than for a clean sinusoid, and worse the
wider the guard window. `SENSEDEAD` was removed rather than defaulted to 0
and left in the config, since a knob whose only well-tested setting is "off"
is a trap for the next person tuning this board.

With no blanking, the magnet-away curve does not clearly survive above noise
either — every point of that same sweep landed within about one sigma of
zero. So whatever the original electrical effect is, if it exists at all it
is smaller than this measurement can currently resolve, and it is very
unlikely to matter next to a much bigger problem: an attract-only
coil at a fringe-field placement measured *microseconds* of authority per
pulse against a clock that needs *~100 microseconds per swing* to correct —
over an order of magnitude short regardless of how cleanly it is measured.
Chasing this artifact further is a low priority until that gap is closed by
some combination of a much wider `PW`, closer or more direct coupling, or
more drive current.

Advance and retard are measured separately, because they are not the same
number. An attract-only coil retards when the bob is on its side of centre
and advances only when the bob is at the far extreme, where the field is far
weaker — depending on where the coil sits the two can differ by more than an
order of magnitude. Run it both ways:

```
MEASURE 60 N        advance
MEASURE 60 Y        retard
AUTH <advance_ns> <retard_ns>
SAVE
```

If a direction comes back with the wrong sign the measurement says so and
refuses to suggest an `AUTH` for it — the pulse is landing on the wrong side
of the bob's turning point, and `PTIMESCAN` is the tool for finding where it
should go. That sweep ranks placements by what they were *asked* to do, not
by how large the number came out, and draws each one as a bar either side of
a zero column so a sign flip is visible at a glance.

Leaving one of them at zero is legitimate: the loop then corrects in one
direction only, which is all a clock that consistently gains ever needs.

**8. Close the loop.** With authority measured, the loop starts spending its
demand in whole pulses. `STATUS` shows `undelivered credit` — the correction
asked for but not yet paid out — and `pulses` counting up. At 11 s/day it
needs 113 µs of retard per swing, so expect one pulse every few dozen swings,
not one per swing.

**9. Set the hands.** `OFFSET 30000` tells the loop that "on time" is 30 s
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
