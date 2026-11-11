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
 * off). And the sampler's work (src/pic_sampler_core.h) against a faked
 * kernel side: which reads count, the read's time, the tick grid, the
 * readers' lock, the means text and the snapshot's bytes, and its start
 * and stop.
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
#include "pic_sampler_core.h"

#include <stdio.h>
#include <string.h>

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
  /* the rate asked for, held to the most the sampler makes */
  CHECK(pic_sampler_rate(0) == 0);
  CHECK(pic_sampler_rate(1) == 1);
  CHECK(pic_sampler_rate(50) == 50);
  CHECK(pic_sampler_rate(PIC_SAMPLER_MAX_HZ) == PIC_SAMPLER_MAX_HZ);
  CHECK(pic_sampler_rate(PIC_SAMPLER_MAX_HZ + 1) == PIC_SAMPLER_MAX_HZ);
  CHECK(pic_sampler_rate(4000000000u) == PIC_SAMPLER_MAX_HZ);
  CHECK(PIC_SAMPLER_MAX_HZ == 200);
  /* the module parameter: 50 unless asked otherwise; a rate over the most
   * is held and stored back, so the parameter reads back as the rate in use */
  {
    unsigned int param = 201;
    CHECK(PIC_SAMPLER_DEFAULT_HZ == 50);
    CHECK(pic_sampler_rate_take(&param) == 200 && param == 200);
  }
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

/* The kernel's side of the sampler (src/pic_sampler_core.h), faked: a clock
 * the transfer advances, the transfer's registers and result, the lock,
 * the ring's memory and the thread, with the steps in the order taken. */
#define MS 1000000LL

struct fake {
  struct pic_sampler_core *core;
  long long clock;
  long long read_cost;
  int read_ret;
  pic_value regs[PIC_NUM_REGISTERS];
  int locked;
  int clock_locked;
  int alloc_fails;
  int thread_err;
  int running_at_free;
  uint16_t ring[10 * 10 * 50];
  char steps[16];
  int nsteps;
};

static void f_step(struct fake *f, char c)
{
  if (f->nsteps < (int)sizeof(f->steps) - 1) {
    f->steps[f->nsteps++] = c;
    f->steps[f->nsteps] = '\0';
  }
}

static void f_steps_reset(struct fake *f)
{
  f->nsteps = 0;
  f->steps[0] = '\0';
}

static long long f_now(void *ctx)
{
  struct fake *f = ctx;
  f->clock_locked = f->locked;
  return f->clock;
}

static int f_read(void *ctx, pic_value *regs)
{
  struct fake *f = ctx;
  f->clock += f->read_cost;
  memcpy(regs, f->regs, sizeof(f->regs));
  return f->read_ret;
}

static void f_lock(void *ctx)
{
  struct fake *f = ctx;
  f->locked = 1;
  f_step(f, 'L');
}

static void f_unlock(void *ctx)
{
  struct fake *f = ctx;
  f->locked = 0;
  f_step(f, 'U');
}

static uint16_t *f_alloc_ring(void *ctx, size_t len)
{
  struct fake *f = ctx;
  f_step(f, 'A');
  return f->alloc_fails || len > sizeof(f->ring) / sizeof(f->ring[0]) ? NULL : f->ring;
}

static void f_free_ring(void *ctx, uint16_t *ring)
{
  struct fake *f = ctx;
  (void)ring;
  f->running_at_free = f->core->running;
  f_step(f, 'F');
}

static int f_thread_start(void *ctx)
{
  struct fake *f = ctx;
  f_step(f, 'S');
  return f->thread_err;
}

static void f_thread_stop(void *ctx)
{
  struct fake *f = ctx;
  f_step(f, 'T');
}

static const struct pic_sampler_ops fake_ops = {
  .now = f_now,
  .read = f_read,
  .lock = f_lock,
  .unlock = f_unlock,
  .alloc_ring = f_alloc_ring,
  .free_ring = f_free_ring,
  .thread_start = f_thread_start,
  .thread_stop = f_thread_stop,
};

/* A sampler started at 50 Hz on the fake, whose transfer reads every
 * register: register 0 the PIC's ID, register r otherwise 100 r + 7. */
static void fake_up(struct fake *f, struct pic_sampler_core *c)
{
  unsigned int r;
  int err;

  memset(f, 0, sizeof(*f));
  memset(c, 0, sizeof(*c));
  f->core = c;
  f->read_ret = (int)sizeof(f->regs);
  f->regs[0] = PIC_MAGIC_NUMBER;
  for (r = 1; r < PIC_NUM_REGISTERS; r++) {
    f->regs[r] = (pic_value)(100 * r + 7);
  }
  CHECK(pic_sampler_start_core(c, &fake_ops, f, 50, &err) == PIC_SAMPLER_STARTED && err == 0);
}

/* A read counts only with every register and the PIC's ID word in it; any
 * other read is an error that empties the windows and leaves the newest
 * good read as it was. A good read more than two periods after the last
 * good one empties the windows before it joins them. */
static void test_take(void)
{
  struct fake f;
  struct pic_sampler_core c;
  struct pic_stats_summary s;
  pic_value dead[PIC_NUM_REGISTERS];

  fake_up(&f, &c);
  CHECK(pic_sampler_take(&c, 1000 * MS, f.read_ret, f.regs) == 0);
  CHECK(pic_sampler_take(&c, 1020 * MS, f.read_ret, f.regs) == 0);
  /* every register, but no ID word: a dead PIC reads zeros */
  memset(dead, 0, sizeof(dead));
  CHECK(pic_sampler_take(&c, 1040 * MS, f.read_ret, dead) == -1);
  pic_stats_summarize(&c.stats, 0, &s);
  CHECK(c.reads == 2 && c.errors == 1 && s.n_long == 0);
  CHECK(c.read_ns == 1020 * MS && c.regs[0] == PIC_MAGIC_NUMBER);
  /* the ID word, but a short transfer */
  CHECK(pic_sampler_take(&c, 1060 * MS, f.read_ret, f.regs) == 0);
  CHECK(pic_sampler_take(&c, 1080 * MS, 20, f.regs) == -1);
  pic_stats_summarize(&c.stats, 0, &s);
  CHECK(c.reads == 3 && c.errors == 2 && c.read_ns == 1060 * MS && s.n_long == 0);

  /* two periods (40 ms) after the last good read keeps the run; a
   * nanosecond more starts it over, holding only the new read */
  CHECK(pic_sampler_take(&c, 1100 * MS, f.read_ret, f.regs) == 0);
  CHECK(pic_sampler_take(&c, 1140 * MS, f.read_ret, f.regs) == 0);
  pic_stats_summarize(&c.stats, 0, &s);
  CHECK(s.n_long == 2);
  CHECK(pic_sampler_take(&c, 1180 * MS + 1, f.read_ret, f.regs) == 0);
  pic_stats_summarize(&c.stats, 0, &s);
  CHECK(s.n_long == 1);
}

/* A read's time is the clock before its transfer, so a slow transfer never
 * makes the read look newer than it is. */
static void test_cycle(void)
{
  struct fake f;
  struct pic_sampler_core c;

  fake_up(&f, &c);
  f.clock = 5000 * MS;
  f.read_cost = 30 * MS;
  CHECK(pic_sampler_cycle(&c, &fake_ops, &f) == 0);
  CHECK(c.read_ns == 5000 * MS && f.clock == 5030 * MS);
}

/* The thread's ticks: one period on from the last tick while on time;
 * after an overrun, one period on from now, never the missed ticks back
 * to back. */
static void test_tick(void)
{
  const long long period = pic_sampler_period_ns(50);

  CHECK(period == 20 * MS);
  CHECK(pic_sampler_next_tick(1000 * MS, 1005 * MS, period) == 1020 * MS);
  CHECK(pic_sampler_next_tick(1000 * MS, 1065 * MS, period) == 1085 * MS);
}

/* The means and the snapshot read the clock under the lock, and refuse a
 * newest read more than a second old. */
static void test_readers(void)
{
  struct fake f;
  struct pic_sampler_core c;
  struct pic_sampler_means m;
  struct pic_sampler_snapshot snap;

  fake_up(&f, &c);
  f.clock = 2000 * MS;
  CHECK(pic_sampler_cycle(&c, &fake_ops, &f) == 0);
  f.clock_locked = 0;
  CHECK(pic_sampler_means_read(&c, &fake_ops, &f, &m) == 0 && f.clock_locked);
  f.clock_locked = 0;
  CHECK(pic_sampler_snapshot_read(&c, &fake_ops, &f, &snap) == 0 && f.clock_locked);
  f.clock += PIC_SAMPLER_NS_PER_S + 1;
  CHECK(pic_sampler_means_read(&c, &fake_ops, &f, &m) == -1);
  CHECK(pic_sampler_snapshot_read(&c, &fake_ops, &f, &snap) == -1);
}

/* The means text, exactly: it is a contract, forgectrl's flowrun.c parses
 * it. Every register holds a value of its own, so each channel's line
 * shows the register it was taken from. */
static void test_means_text(void)
{
  static const char want[] =
    "hz 50 age_ms 12 reads 1000 errors 0\n"
    "water_temp_1 702.000 702.000 700 704 50 500\n"
    "water_temp_2 207.500 207.500 207 208 50 500\n"
    "tec_temp 307.020 307.020 307 308 50 500\n"
    "pwr_temp 407.000 407.000 407 407 50 500\n"
    "lid_ir_1 507.000 507.000 507 507 50 500\n"
    "lid_ir_2 607.000 607.000 607 607 50 500\n"
    "lid_ir_3 707.000 707.000 707 707 50 500\n"
    "lid_ir_4 807.000 807.000 807 807 50 500\n"
    "hv_current 1507.000 1507.000 1507 1507 50 500\n"
    "hv_voltage 1607.000 1607.000 1607 1607 50 500\n";
  struct fake f;
  struct pic_sampler_core c;
  struct pic_sampler_means m;
  char buf[4096];
  int i, len;

  fake_up(&f, &c);
  f.clock = 1000 * MS;
  for (i = 0; i < 1000; i++) {
    f.regs[PIC_REG_WATER_THERM1] = (pic_value)(i % 2 ? 704 : 700);
    f.regs[PIC_REG_WATER_THERM2] = (pic_value)(i % 2 ? 208 : 207);
    f.regs[PIC_REG_TEC_THERM] = (pic_value)(i % 50 ? 307 : 308);
    CHECK(pic_sampler_cycle(&c, &fake_ops, &f) == 0);
    f.clock += 20 * MS;
  }
  /* 12.9 ms after the last read */
  f.clock += 12900000LL - 20 * MS;
  CHECK(pic_sampler_means_read(&c, &fake_ops, &f, &m) == 0);
  len = pic_sampler_means_format(buf, sizeof(buf), &m);
  CHECK(len == (int)strlen(want) && strcmp(buf, want) == 0);
  if (strcmp(buf, want) != 0) {
    fprintf(stderr, "means text:\n%s", buf);
  }
}

/* The snapshot's 42 bytes: the read's time as a little-endian 64-bit word,
 * then every register in register order as little-endian 16-bit words. */
static void test_snapshot(void)
{
  struct pic_sampler_snapshot snap;
  unsigned char out[64];
  unsigned int i;
  int laid_out = 1;

  snap.read_ns = 0x0102030405060708LL;
  snap.regs[0] = PIC_MAGIC_NUMBER;
  for (i = 1; i < PIC_NUM_REGISTERS; i++) {
    snap.regs[i] = (pic_value)(0x1100 + i);
  }
  CHECK(pic_sampler_snapshot_pack(out, &snap) == 42);
  for (i = 0; i < 8; i++) {
    laid_out &= out[i] == 8 - i;
  }
  laid_out &= out[8] == 0x53 && out[9] == 0x4d;
  for (i = 1; i < PIC_NUM_REGISTERS; i++) {
    laid_out &= out[8 + 2 * i] == i && out[9 + 2 * i] == 0x11;
  }
  CHECK(laid_out);
}

/* A start that fails leaves no sampler running and nothing allocated, so
 * the means and the snapshot refuse. A stop ends the thread first, then
 * clears running under the lock, and only then frees the ring. */
static void test_start_stop(void)
{
  struct fake f;
  struct pic_sampler_core c;
  int err;

  memset(&f, 0, sizeof(f));
  memset(&c, 0, sizeof(c));
  f.core = &c;
  CHECK(pic_sampler_start_core(&c, &fake_ops, &f, 0, &err) == PIC_SAMPLER_OFF && f.nsteps == 0);
  f.alloc_fails = 1;
  CHECK(pic_sampler_start_core(&c, &fake_ops, &f, 50, &err) == PIC_SAMPLER_NO_MEMORY && !c.running);
  /* statistics that do not initialize: a rate whose ten-second window
   * wraps to fewer reads than its one-second window */
  f.alloc_fails = 0;
  f_steps_reset(&f);
  CHECK(pic_sampler_start_core(&c, &fake_ops, &f, 429496730u, &err) == PIC_SAMPLER_NO_STATS);
  CHECK(!c.running && strcmp(f.steps, "AF") == 0);
  f.thread_err = -11;
  f.running_at_free = 1;
  CHECK(pic_sampler_start_core(&c, &fake_ops, &f, 50, &err) == PIC_SAMPLER_NO_THREAD && err == -11);
  CHECK(!c.running && c.ring == NULL && f.running_at_free == 0);

  f.thread_err = 0;
  CHECK(pic_sampler_start_core(&c, &fake_ops, &f, 50, &err) == PIC_SAMPLER_STARTED);
  CHECK(c.running && c.hz == 50 && c.ring == f.ring);
  f_steps_reset(&f);
  f.running_at_free = 1;
  pic_sampler_stop_core(&c, &fake_ops, &f);
  CHECK(strcmp(f.steps, "TLUF") == 0 && f.running_at_free == 0 && !c.running && c.ring == NULL);
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
  test_take();
  test_cycle();
  test_tick();
  test_readers();
  test_means_text();
  test_snapshot();
  test_start_stop();
  if (failures) {
    fprintf(stderr, "pic_stats_test: %d failure(s)\n", failures);
    return 1;
  }
  printf("pic_stats_test: all passed\n");
  return 0;
}
