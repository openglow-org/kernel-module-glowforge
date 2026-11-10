// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * heater_lease_test.c
 *
 * Host-side test for the loop heater's lease (src/heater_lease.h): a duty
 * above zero lapses at the end of its lease and not before, a write again
 * within the lease carries it on, a heater at zero never lapses, and a
 * lease parameter out of range is held to 2 to 60 s. Also the two steps
 * the driver takes under its lock: a write (a duty above zero starts a new
 * lease, a zero does not), and the timer's look at the lease on every edge
 * (a lapsed heater goes to zero and is counted once).
 *
 * Copyright 2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "heater_lease.h"

#include <stdio.h>

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

#define MS 1000000LL

/* The driver's heater state, and its timer looking at the lease every
 * 10 ms (the heater's PWM period) from @from to @to. Returns how many of
 * the looks ended a lease. */
struct heater {
  unsigned short duty;
  long long until_ns;
  unsigned int lapses;
};

static int run_timer(struct heater *h, long long from, long long to)
{
  long long t;
  int ended = 0;
  for (t = from; t <= to; t += 10 * MS) {
    ended += heater_lease_tick(&h->duty, h->until_ns, t, &h->lapses);
  }
  return ended;
}

/* The write and the timer's look, as the driver strings them together. */
static void test_write_and_tick(void)
{
  struct heater h = { 0, 0, 0 };
  long long t0 = 1000 * MS;
  long long end;

  /* A heater never written has nothing to lapse. */
  CHECK(run_timer(&h, t0, t0 + 10000 * MS) == 0);
  CHECK(h.duty == 0 && h.lapses == 0);

  /* A write holds its duty to the lease's end; the look at the end takes
   * it to zero and counts it, and the looks after that count nothing. */
  heater_lease_write(&h.duty, &h.until_ns, 2162, t0, HEATER_LEASE_MS_DEFAULT);
  end = t0 + 5000 * MS;
  CHECK(h.duty == 2162 && h.until_ns == end);
  CHECK(run_timer(&h, t0, end - 10 * MS) == 0);
  CHECK(h.duty == 2162 && h.lapses == 0);
  CHECK(heater_lease_tick(&h.duty, h.until_ns, end, &h.lapses) == 1);
  CHECK(h.duty == 0 && h.lapses == 1);
  CHECK(run_timer(&h, end, end + 10000 * MS) == 0);
  CHECK(h.duty == 0 && h.lapses == 1);

  /* A write of zero turns it off at once, moves no lease, and is no lapse. */
  heater_lease_write(&h.duty, &h.until_ns, 65535, t0, HEATER_LEASE_MS_DEFAULT);
  heater_lease_write(&h.duty, &h.until_ns, 0, t0 + 1000 * MS, HEATER_LEASE_MS_DEFAULT);
  CHECK(h.duty == 0 && h.until_ns == end);
  CHECK(run_timer(&h, t0 + 1000 * MS, end + 1000 * MS) == 0);
  CHECK(h.lapses == 1);

  /* A lease at the minimum outlasts a 1 Hz writer whose write comes 900 ms
   * late; when the writes stop, the heater is off one lease later. */
  {
    long long t = t0;
    int i;
    h.lapses = 0;
    heater_lease_write(&h.duty, &h.until_ns, 2162, t, 0);
    for (i = 0; i < 60; i++) {
      long long next = t + 1000 * MS + (i == 30 ? 900 * MS : 0);
      CHECK(run_timer(&h, t, next) == 0);
      t = next;
      heater_lease_write(&h.duty, &h.until_ns, 2162, t, 0);
    }
    CHECK(h.duty == 2162 && h.lapses == 0);
    CHECK(run_timer(&h, t, t + HEATER_LEASE_MS_MIN * MS - 10 * MS) == 0);
    CHECK(run_timer(&h, t + HEATER_LEASE_MS_MIN * MS, t + 10000 * MS) == 1);
    CHECK(h.duty == 0 && h.lapses == 1);
  }
}

int main(void)
{
  long long t0 = 1000 * MS;
  long long until = heater_lease_until(t0, HEATER_LEASE_MS_DEFAULT);

  /* A write holds for the lease, and lapses at its end. */
  CHECK(until == t0 + 5000 * MS);
  CHECK(!heater_lease_lapsed(2162, t0, until));
  CHECK(!heater_lease_lapsed(2162, until - 1, until));
  CHECK(heater_lease_lapsed(2162, until, until));
  CHECK(heater_lease_lapsed(65535, until + 3600000 * MS, until));

  /* A writer that writes again every second keeps it on for good. */
  {
    long long t = t0, u = until;
    int lapsed = 0, i;
    for (i = 0; i < 3600; i++) {
      t += 1000 * MS;
      lapsed |= heater_lease_lapsed(2162, t, u);
      u = heater_lease_until(t, HEATER_LEASE_MS_DEFAULT);
    }
    CHECK(!lapsed);
  }

  /* A heater at zero has nothing to lapse. */
  CHECK(!heater_lease_lapsed(0, until + 1, until));

  /* The parameter's range. */
  CHECK(heater_lease_ms_held(0) == 2000);
  CHECK(heater_lease_ms_held(1999) == 2000);
  CHECK(heater_lease_ms_held(2000) == 2000);
  CHECK(heater_lease_ms_held(2001) == 2001);
  CHECK(heater_lease_ms_held(5000) == 5000);
  CHECK(heater_lease_ms_held(60000) == 60000);
  CHECK(heater_lease_ms_held(60001) == 60000);
  CHECK(heater_lease_ms_held(4000000000u) == HEATER_LEASE_MS_MAX);
  CHECK(heater_lease_until(t0, 0) == t0 + 2000 * MS);

  test_write_and_tick();

  if (failures) {
    fprintf(stderr, "heater_lease_test: %d failure(s)\n", failures);
    return 1;
  }
  printf("heater_lease_test: all passed\n");
  return 0;
}
