# Host tests for the parts of the firmware that are pure arithmetic.
#
# control.c is compiled unmodified against stub headers, so what is under
# test is the code that runs on the board - not a transcription of it.
#
#   make          build
#   make check    the whole battery
#
CFLAGS = -O1 -Wall -Wextra -I. -I../src
all: authority

authority: authority.c stubs.c ../src/control.c board.h pico/stdlib.h
	$(CC) $(CFLAGS) -o $@ authority.c stubs.c ../src/control.c -lm

check: authority
	@./check.sh

clean:
	rm -f authority
.PHONY: all check clean
