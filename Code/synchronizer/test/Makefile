# Host tests for the parts of the firmware that are pure arithmetic.
#
# control.c is compiled unmodified against stub headers, so what is under
# test is the code that runs on the board - not a transcription of it.
#
#   make          build
#   make check    the whole battery
#
CFLAGS = -O1 -Wall -Wextra -I. -I../src
all: kick

kick: kick.c stubs.c ../src/control.c board.h pico/stdlib.h
	$(CC) $(CFLAGS) -o $@ kick.c stubs.c ../src/control.c -lm

check: kick
	@./check.sh

clean:
	rm -f kick authority
.PHONY: all check clean
