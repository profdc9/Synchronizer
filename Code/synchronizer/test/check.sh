#!/bin/sh
# Feed synthetic swings through the real control loop and check that MEASURE
# recovers what was put in, and that the error bar it quotes is honest.
#
# The numbers are the development clock's: 2.27 ms of per-swing timing noise
# against a per-pulse kick of tens of microseconds, which is the regime that
# made endpoint differencing produce a different answer every run.
set -e
cd "$(dirname "$0")"

say() { printf '\n== %s ==\n' "$1"; }

say "60 pulses of 89100 ns, plus an amplitude-driven rate step"
./authority 89100 400 2270000 60 12345 | tail -6

say "300 pulses - the error bar should fall about tenfold"
./authority 89100 400 2270000 300 99 | tail -6

say "no authority at all - must refuse to call noise a measurement"
./authority 0 0 2270000 60 7 | tail -4

say "a pulse that does the opposite of what was asked"
./authority -89100 400 2270000 60 42 | tail -4

say "PTIMESCAN: the largest magnitude is on the WRONG side of zero"
say "  (ranking by magnitude picks 60000 us; ranking by sign picks 35000)"
SWEEP=1 ./authority 0 0 300000 40 5 | sed -n '/placement sweep/,/SAVE/p'

say "PTIMESCAN: no placement in range works - must say so, not pick one"
SWEEP=1 WRONGSIDE=1 ./authority 0 0 300000 40 5 | sed -n '/placement sweep/,/instead/p'
