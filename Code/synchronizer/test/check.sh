#!/bin/sh
# Feed synthetic swings through the real control loop and check that KICK
# trips the right direction, keeps pulsing until corrected, releases, and
# that a genuine schedule slip still drops the loop into hold.
set -e
cd "$(dirname "$0")"

./kick
