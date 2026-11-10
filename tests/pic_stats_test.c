// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pic_stats_test.c
 *
 * Host-side test for the PIC sampler's rolling statistics
 * (src/pic_stats.c): a window's mean, minimum, maximum and count are exact
 * across the ring's wrap, a window not yet full reports what it holds, the
 * short and long windows disagree on a step as they should, a mean in
 * thousandths of a count rounds half up, and a cleared window starts over.
 * Also the sampler's judgments about time (src/pic_sampler_policy.h): a
 * run of reads broken by a gap, the age past which a read is stale, and
 * what a reader is refused (a stale read, no read yet, a sampler that is
 * off).
 *
 * Copyright 2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "pic_stats.h"
#include "pic_sampler_policy.h"

#include <stdio.h>

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

static void push1(struct pic_stats *st, uint16_t a, uint16_t b)
{
  uint16_t v[2] = { a, b };
  pic_stats_push(st, v);
}

/* A window not yet full reports only what it holds. */
static void test_partial(void)
{
  uint16_t ring[2 * 10];
  struct pic_stats st;
  struct pic_stats_summary s;

  CHECK(pic_stats_init(&st, 2, 4, 10, ring) == 0);
  pic_stats_summarize(&st, 0, &s);
  CHECK(s.n_short == 0 && s.n_long == 0 && s.mean_short_milli == 0 && s.min_short == 0 && s.max_short == 0);
  push1(&st, 700, 10);
  push1(&st, 704, 20);
  pic_stats_summarize(&st, 0, &s);
  CHECK(s.n_short == 2 && s.n_long == 2);
  CHECK(s.mean_short_milli == 702000 && s.mean_long_milli == 702000);
  CHECK(s.min_short == 700 && s.max_short == 704);
  pic_stats_summarize(&st, 1, &s);
  CHECK(s.mean_short_milli == 15000 && s.min_short == 10 && s.max_short == 20);
}

/* Across many wraps the windows hold exactly the newest samples: checked
 * against a direct computation over the pushed sequence. */
static void test_wrap_exact(void)
{
  enum { NS = 3, NL = 7, PUSHES = 40 };
  uint16_t ring[2 * NL];
  uint16_t seq[PUSHES];
  struct pic_stats st;
  struct pic_stats_summary s;
  int i;

  CHECK(pic_stats_init(&st, 2, NS, NL, ring) == 0);
  for (i = 0; i < PUSHES; i++) {
    seq[i] = (uint16_t)(500 + (i * 37) % 23);
    push1(&st, seq[i], (uint16_t)(1000 - seq[i]));
  }
  {
    uint64_t sum_s = 0, sum_l = 0;
    uint16_t mn = 0xffff, mx = 0;
    for (i = PUSHES - NS; i < PUSHES; i++) {
      sum_s += seq[i];
      if (seq[i] < mn) mn = seq[i];
      if (seq[i] > mx) mx = seq[i];
    }
    for (i = PUSHES - NL; i < PUSHES; i++) {
      sum_l += seq[i];
    }
    pic_stats_summarize(&st, 0, &s);
    CHECK(s.n_short == NS && s.n_long == NL);
    CHECK(s.mean_short_milli == (sum_s * 1000 + NS / 2) / NS);
    CHECK(s.mean_long_milli == (sum_l * 1000 + NL / 2) / NL);
    CHECK(s.min_short == mn && s.max_short == mx);
    /* the second channel is kept apart from the first */
    pic_stats_summarize(&st, 1, &s);
    CHECK(s.mean_long_milli == ((uint64_t)NL * 1000 * 1000 - sum_l * 1000 + NL / 2) / NL);
  }
}

/* A step: the short window has moved to the new level while the long one
 * still averages both. */
static void test_step(void)
{
  uint16_t ring[2 * 10];
  struct pic_stats st;
  struct pic_stats_summary s;
  int i;

  CHECK(pic_stats_init(&st, 2, 2, 10, ring) == 0);
  for (i = 0; i < 10; i++) {
    push1(&st, 700, 0);
  }
  push1(&st, 720, 0);
  push1(&st, 720, 0);
  pic_stats_summarize(&st, 0, &s);
  CHECK(s.mean_short_milli == 720000);
  CHECK(s.mean_long_milli == 704000);   /* 8 x 700 + 2 x 720 over 10 */
  CHECK(s.min_short == 720 && s.max_short == 720);
}

/* Thousandths round half up: 1 + 0 over 2 is 0.5 -> 500; 2 over 3 is
 * 666.67 -> 667; 1 over 3 is 333.33 -> 333. */
static void test_rounding(void)
{
  uint16_t ring[2 * 3];
  struct pic_stats st;
  struct pic_stats_summary s;

  CHECK(pic_stats_init(&st, 2, 3, 3, ring) == 0);
  push1(&st, 1, 0);
  push1(&st, 0, 1);
  pic_stats_summarize(&st, 0, &s);
  CHECK(s.mean_short_milli == 500);
  push1(&st, 1, 0);
  pic_stats_summarize(&st, 0, &s);
  CHECK(s.mean_short_milli == 667);
  pic_stats_summarize(&st, 1, &s);
  CHECK(s.mean_short_milli == 333);
}

/* Sizes that cannot work are refused. */
static void test_init_refuses(void)
{
  uint16_t ring[4];
  struct pic_stats st;

  CHECK(pic_stats_init(&st, 0, 1, 2, ring) == -1);
  CHECK(pic_stats_init(&st, PIC_STATS_MAX_CHANNELS + 1, 1, 2, ring) == -1);
  CHECK(pic_stats_init(&st, 1, 0, 2, ring) == -1);
  CHECK(pic_stats_init(&st, 1, 3, 2, ring) == -1);
  CHECK(pic_stats_init(&st, 1, 1, 2, NULL) == -1);
  CHECK(pic_stats_ring_len(10, 500) == 5000);
}

/* A cleared window holds nothing of the run before it: the old level is
 * gone from both means, and the counts start again. */
static void test_clear(void)
{
  uint16_t ring[2 * 10];
  struct pic_stats st;
  struct pic_stats_summary s;
  int i;

  CHECK(pic_stats_init(&st, 2, 4, 10, ring) == 0);
  for (i = 0; i < 12; i++) {
    push1(&st, 700, 5);
  }
  pic_stats_clear(&st);
  pic_stats_summarize(&st, 0, &s);
  CHECK(s.n_short == 0 && s.n_long == 0 && s.mean_short_milli == 0);
  push1(&st, 900, 7);
  pic_stats_summarize(&st, 0, &s);
  CHECK(s.n_short == 1 && s.n_long == 1);
  CHECK(s.mean_short_milli == 900000 && s.mean_long_milli == 900000);
  CHECK(s.min_short == 900 && s.max_short == 900);
}

/* A gap of more than two periods breaks the run; reads on time, or the
 * first read, do not. The stale age is one second, or two periods under
 * 2 Hz, so a read on time at 1 Hz is never refused. */
static void test_policy(void)
{
  const long long s = PIC_SAMPLER_NS_PER_S;

  CHECK(!pic_sampler_run_broken(0, 5 * s, 50));
  CHECK(!pic_sampler_run_broken(s, s + s / 50, 50));
  CHECK(!pic_sampler_run_broken(s, s + 2 * (s / 50), 50));
  CHECK(pic_sampler_run_broken(s, s + 2 * (s / 50) + 1, 50));
  CHECK(pic_sampler_run_broken(s, 10 * s, 50));
  CHECK(!pic_sampler_run_broken(s, 2 * s, 1));
  CHECK(pic_sampler_run_broken(s, 3 * s + 1, 1));
  CHECK(pic_sampler_stale_ns(50) == s);
  CHECK(pic_sampler_stale_ns(200) == s);
  CHECK(pic_sampler_stale_ns(1) == 2 * s);
  CHECK(pic_sampler_stale_ns(0) == 2 * s);
}

/* A reader gets the newest read up to the stale age and not past it, and
 * gets nothing before the first read or with the sampler off. */
static void test_fresh(void)
{
  const long long s = PIC_SAMPLER_NS_PER_S;
  const long long read = 100 * s;

  CHECK(pic_sampler_fresh(1, read, read, 50));
  CHECK(pic_sampler_fresh(1, read, read + s / 50, 50));
  CHECK(pic_sampler_fresh(1, read, read + s, 50));
  CHECK(!pic_sampler_fresh(1, read, read + s + 1, 50));
  CHECK(!pic_sampler_fresh(1, read, read + 3600 * s, 50));
  /* at 1 Hz a read on time, a full period old, is still reported */
  CHECK(pic_sampler_fresh(1, read, read + s + 1, 1));
  CHECK(!pic_sampler_fresh(1, read, read + 2 * s + 1, 1));
  /* no read yet: refused even inside the first second of the clock */
  CHECK(!pic_sampler_fresh(1, 0, s / 2, 50));
  /* the sampler off: refused whatever it last held */
  CHECK(!pic_sampler_fresh(0, read, read, 50));
}

int main(void)
{
  test_partial();
  test_wrap_exact();
  test_step();
  test_rounding();
  test_init_refuses();
  test_clear();
  test_policy();
  test_fresh();
  if (failures) {
    fprintf(stderr, "pic_stats_test: %d failure(s)\n", failures);
    return 1;
  }
  printf("pic_stats_test: all passed\n");
  return 0;
}
