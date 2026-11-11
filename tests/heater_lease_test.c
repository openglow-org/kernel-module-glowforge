// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * heater_lease_test.c
 *
 * Host-side test for the loop heater's lease (src/heater_lease.h): a duty
 * above zero lapses at the end of its lease and not before, a write again
 * within the lease carries it on, a heater at zero never lapses, and a
 * lease parameter out of range is held to 2 to 60 s and stored back. Also
 * the steps the driver takes under its lock: a write (a duty above zero
 * starts a new lease, a zero does not, a value over full scale is refused
 * and changes nothing), and the PWM timer's edge, which looks at the lease
 * first (a lapsed heater goes to zero and is counted once) and drives the
 * pin from the duty that look leaves. And the orders of the thermal
 * driver's safe state, remove and shutdown (src/thermal_safe.h).
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
#include "thermal_safe.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

#define MS 1000000LL

static const long long t0_ns = 1000 * MS;

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
   * late; when the writes stop, the heater is off one lease later. The
   * lapses count since load: this is the second. */
  {
    long long t = t0;
    int i;
    heater_lease_write(&h.duty, &h.until_ns, 2162, t, 0);
    for (i = 0; i < 60; i++) {
      long long next = t + 1000 * MS + (i == 30 ? 900 * MS : 0);
      CHECK(run_timer(&h, t, next) == 0);
      t = next;
      heater_lease_write(&h.duty, &h.until_ns, 2162, t, 0);
    }
    CHECK(h.duty == 2162 && h.lapses == 1);
    CHECK(run_timer(&h, t, t + HEATER_LEASE_MS_MIN * MS - 10 * MS) == 0);
    CHECK(run_timer(&h, t + HEATER_LEASE_MS_MIN * MS, t + 10000 * MS) == 1);
    CHECK(h.duty == 0 && h.lapses == 2);
  }
}

/* The heater's software PWM, edge by edge as its timer takes them. The pin
 * (1: the heater on) is high for the duty's part of the period and low for
 * the rest, low at 0 and high at full. Every edge looks at the lease, so
 * when the writes stop the first edge at or after the lease's end drives
 * the pin low, within one period of it, and no edge raises it again. */
static void test_pwm_edge(void)
{
  struct heater h = { 2162, 1000000 * MS, 0 };
  long long t, end, next, first_after = -1, longest = 0;
  int level, next_level, high_after = 0;
  unsigned int f;

  CHECK(heater_pwm_edge(&h.duty, h.until_ns, t0_ns, &h.lapses, 0, &next_level, &next) == 0);
  CHECK(next_level == 1 && next == 329895);
  CHECK(heater_pwm_edge(&h.duty, h.until_ns, t0_ns, &h.lapses, 1, &next_level, &next) == 0);
  CHECK(next_level == 0 && next == 9669952);
  h.duty = 0;
  heater_pwm_edge(&h.duty, h.until_ns, t0_ns, &h.lapses, 1, &next_level, &next);
  CHECK(next_level == 0 && next == HEATER_PWM_PERIOD_NS);
  h.duty = HEATER_DUTY_MAX;
  heater_pwm_edge(&h.duty, h.until_ns, t0_ns, &h.lapses, 0, &next_level, &next);
  CHECK(next_level == 1 && next == HEATER_PWM_PERIOD_NS);

  h.duty = 0;
  h.lapses = 0;
  t = t0_ns;
  heater_lease_write(&h.duty, &h.until_ns, 2162, t, HEATER_LEASE_MS_DEFAULT);
  end = h.until_ns;
  level = 0;
  while (t < end + 1000 * MS) {
    heater_pwm_edge(&h.duty, h.until_ns, t, &h.lapses, level, &next_level, &next);
    if (t >= end && first_after < 0) {
      first_after = t;
    }
    if (t >= end && next_level) {
      high_after = 1;
    }
    level = next_level;
    t += next;
  }
  CHECK(first_after >= end && first_after < end + HEATER_PWM_PERIOD_NS);
  CHECK(!high_after && h.duty == 0 && h.lapses == 1);

  /* No part of a period is longer than the period. */
  for (f = 0; f <= HEATER_DUTY_MAX; f++) {
    if (heater_pwm_interval_ns(f) > longest) {
      longest = heater_pwm_interval_ns(f);
    }
  }
  CHECK(longest <= HEATER_PWM_PERIOD_NS);
}

/* A heater_pwm write over the duty's full scale is refused and changes
 * nothing: the duty written before holds, and its lease is not renewed.
 * Full scale itself is taken. */
static void test_store(void)
{
  struct heater h = { 0, 0, 0 };

  CHECK(heater_lease_store(&h.duty, &h.until_ns, 2162, t0_ns, HEATER_LEASE_MS_DEFAULT) == 0);
  CHECK(heater_lease_store(&h.duty, &h.until_ns, 70000, t0_ns + 4000 * MS, HEATER_LEASE_MS_DEFAULT) == -1);
  CHECK(h.duty == 2162 && h.until_ns == t0_ns + 5000 * MS);
  CHECK(heater_lease_store(&h.duty, &h.until_ns, HEATER_DUTY_MAX, t0_ns, HEATER_LEASE_MS_DEFAULT) == 0);
  CHECK(h.duty == HEATER_DUTY_MAX);
}

/* The thermal driver's steps, recorded in the order taken. The heater's
 * duty goes to zero through the same write the driver's step makes. */
static char steps[16];
static int nsteps;
static struct heater safe_heater;

static void step(char c)
{
  if (nsteps < (int)sizeof(steps) - 1) {
    steps[nsteps++] = c;
    steps[nsteps] = '\0';
  }
}

static void steps_reset(void)
{
  nsteps = 0;
  steps[0] = '\0';
}

static void op_zero_heater(void *ctx)
{
  (void)ctx;
  heater_lease_store(&safe_heater.duty, &safe_heater.until_ns, 0, t0_ns + 1000 * MS,
                     HEATER_LEASE_MS_DEFAULT);
  step('Z');
}

static void op_pins_low(void *ctx) { (void)ctx; step('P'); }
static void op_stop_timer(void *ctx) { (void)ctx; step('T'); }
static void op_heater_pin_low(void *ctx) { (void)ctx; step('L'); }
static void op_remove_attrs(void *ctx) { (void)ctx; step('A'); }
static void op_unregister_dms(void *ctx) { (void)ctx; step('D'); }
static void op_cancel_lapse_work(void *ctx) { (void)ctx; step('W'); }

static const struct thermal_safe_ops safe_ops = {
  .zero_heater = op_zero_heater,
  .pins_low = op_pins_low,
  .stop_timer = op_stop_timer,
  .heater_pin_low = op_heater_pin_low,
  .remove_attrs = op_remove_attrs,
  .unregister_dms = op_unregister_dms,
  .cancel_lapse_work = op_cancel_lapse_work,
};

/* The safe state (the dead man's switch) zeroes the heater's duty first,
 * so the running timer holds the pin low, then takes the heater and TEC
 * pins low; the zero is a write, not a lapse, and moves no lease. Remove
 * takes the attributes away first, so no write follows, and stops the
 * timer before it drives the pin low. Shutdown is the safe state and the
 * heater stopped; the device stays bound. */
static void test_safe_orders(void)
{
  safe_heater.duty = 2162;
  safe_heater.until_ns = t0_ns + 5000 * MS;
  steps_reset();
  thermal_safe_state(&safe_ops, NULL);
  CHECK(strcmp(steps, "ZP") == 0);
  CHECK(safe_heater.duty == 0 && safe_heater.until_ns == t0_ns + 5000 * MS && safe_heater.lapses == 0);

  steps_reset();
  thermal_safe_remove(&safe_ops, NULL);
  CHECK(strcmp(steps, "AZPDTLW") == 0);

  steps_reset();
  thermal_safe_shutdown(&safe_ops, NULL);
  CHECK(strcmp(steps, "ZPTL") == 0);
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
  /* Out of range at load, the parameter is held and stored back, so it
   * reads back as the lease in use; in range it is left as it is. */
  {
    unsigned int param = 0;
    CHECK(heater_lease_ms_take(&param) == 2000 && param == 2000);
    param = 5000;
    CHECK(heater_lease_ms_take(&param) == 5000 && param == 5000);
  }

  test_write_and_tick();
  test_pwm_edge();
  test_store();
  test_safe_orders();

  if (failures) {
    fprintf(stderr, "heater_lease_test: %d failure(s)\n", failures);
    return 1;
  }
  printf("heater_lease_test: all passed\n");
  return 0;
}
