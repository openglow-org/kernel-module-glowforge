# Host-side tests for the kernel-independent parts of the module.
#
# These build with the host compiler against the source in ../src; nothing
# here needs a kernel tree. Run: make -C tests
#
# Each test is a standalone binary that returns nonzero on failure.

CC      ?= gcc
CFLAGS  ?= -O2
CFLAGS  += -std=gnu11 -Wall -Wextra -Werror -I../src

TESTS = interlock_test backtrack_test pic_stats_test heater_lease_test

.PHONY: all check clean

all: $(TESTS)

interlock_test: interlock_test.c ../src/cnc_interlock.c ../src/cnc_interlock.h
	$(CC) $(CFLAGS) -o $@ interlock_test.c ../src/cnc_interlock.c

backtrack_test: backtrack_test.c ../src/cnc_backtrack.h
	$(CC) $(CFLAGS) -o $@ backtrack_test.c

pic_stats_test: pic_stats_test.c ../src/pic_stats.c ../src/pic_stats.h ../src/pic_sampler_policy.h
	$(CC) $(CFLAGS) -o $@ pic_stats_test.c ../src/pic_stats.c

heater_lease_test: heater_lease_test.c ../src/heater_lease.h
	$(CC) $(CFLAGS) -o $@ heater_lease_test.c

check: all
	@set -e; for t in $(TESTS); do ./$$t; done

clean:
	rm -f $(TESTS)
