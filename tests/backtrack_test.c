// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * backtrack_test.c
 *
 * Host-side test for the bounds of a backward run over the pulse ring
 * (src/cnc_backtrack.h). A pause walks the program backward with the laser
 * off so the resume can lead back up to the pause point over material the
 * job already cut, and these bounds are what keep that walk on genuine data.
 * The cases pin:
 *
 *   - only played bytes are backtrackable (a job that has not played yet
 *     has no history, however much of it has been enqueued);
 *   - only resident bytes are backtrackable (a live feed overwrites the
 *     oldest played bytes as it tops the ring up);
 *   - the writer's retained gap is the floor, so a live-fed job keeps a
 *     pause's worth of history without the feeder arranging one;
 *   - the deceleration tail is held back, so a run that is allowed always
 *     ramps down instead of hitting the dead stop;
 *   - the dead stop is the start of the job while the job fits the ring,
 *     and one ring back once a live feed has wrapped it.
 *
 * Copyright (C) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "cnc_backtrack.h"

#include <stdio.h>
#include <stdlib.h>

/* The shipped geometry: a 32 MiB ring with a 32 KiB retained gap, played at
 * the 10 kHz print tick with the default 125 kHz/s ramp. */
#define RING      (32u * 1024u * 1024u)
#define GAP       (32u * 1024u)
#define PRINT_HZ  10000u
#define RAMP      125000u
/* The factory pause: back up 2000 ticks, lead the laser back on after 1950. */
#define PAUSE_TICKS 2000u

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

static uint32_t print_tail(void)
{
  return cnc_backtrack_decel_steps(PRINT_HZ, RAMP);
}

/* v^2/2a at the print tick: 10 kHz into a 125 kHz/s ramp is 400 steps.
 * Faster runs cost more, and the answer never underflows. */
static void test_decel_tail(void)
{
  CHECK(print_tail() == 400);
  CHECK(cnc_backtrack_decel_steps(100000u, RAMP) == 40000);
  CHECK(cnc_backtrack_decel_steps(1000u, 500000u) == 1);
  /* A degenerate ramp rate must not divide by zero. */
  CHECK(cnc_backtrack_decel_steps(PRINT_HZ, 0) > 0);
}

/* Preloaded job, paused part way in: the history is what has played. */
static void test_preload_mid_job(void)
{
  const uint64_t job = 1024u * 1024u;
  const uint32_t played = 100u * 1024u;
  uint32_t unplayed = (uint32_t)job - played;
  uint32_t max = cnc_backtrack_max_steps(RING, unplayed, job, print_tail());

  CHECK(max == played - print_tail());
  CHECK(max > PAUSE_TICKS);
}

/* Preloaded job that has not played a byte: nothing to back into. The bytes
 * behind the play head are ring memory from whatever ran before. */
static void test_preload_nothing_played(void)
{
  const uint64_t job = 1024u * 1024u;
  CHECK(cnc_backtrack_max_steps(RING, (uint32_t)job, job, print_tail()) == 0);
}

/* Early in a preloaded job the whole of what has played is available, and a
 * pause deeper than that is not: the caller shortens its retrace. */
static void test_preload_early_pause(void)
{
  const uint64_t job = 1024u * 1024u;
  uint32_t played = 500u;
  uint32_t max = cnc_backtrack_max_steps(RING, (uint32_t)job - played, job,
                                         print_tail());
  CHECK(max < PAUSE_TICKS);
  CHECK(max == 100);      /* 500 played, 400 of it owed to the ramp */
}

/* A live feed several times the ring, topped up to the writer's gap: the
 * retained gap is the history, and it covers the factory pause with room to
 * spare. */
static void test_streamed_ring_full(void)
{
  const uint64_t total = 100u * 1024u * 1024u;
  uint32_t unplayed = RING - GAP;
  uint32_t max = cnc_backtrack_max_steps(RING, unplayed, total, print_tail());

  CHECK(max == GAP - print_tail());
  CHECK(max > PAUSE_TICKS);
}

/* The same feed with the ring half drained: more of it is history. */
static void test_streamed_ring_half(void)
{
  const uint64_t total = 100u * 1024u * 1024u;
  uint32_t unplayed = RING / 2u;
  uint32_t max = cnc_backtrack_max_steps(RING, unplayed, total, print_tail());

  CHECK(max == (RING / 2u) - print_tail());
}

/* The gap floor holds for every fill a writer can legally leave behind, once
 * the job has played more than the ring holds. */
static void test_gap_is_the_floor(void)
{
  const uint64_t total = 100u * 1024u * 1024u;
  uint32_t unplayed;

  for (unplayed = 0; unplayed <= RING - GAP; unplayed += 64u * 1024u) {
    uint32_t max = cnc_backtrack_max_steps(RING, unplayed, total, print_tail());
    CHECK(max >= GAP - print_tail());
    CHECK(max <= RING - unplayed);
    CHECK(max > PAUSE_TICKS);
  }
}

/* A soak outgrows 32 bits of enqueued bytes; the answer stays a ring-bounded
 * step count rather than wrapping into a large one. */
static void test_long_soak_does_not_wrap(void)
{
  const uint64_t total = 5000000000ULL;   /* past 4 GiB */
  uint32_t unplayed = 1024u;
  uint32_t max = cnc_backtrack_max_steps(RING, unplayed, total, print_tail());

  CHECK(max == RING - unplayed - print_tail());
}

/* An empty ring has no history, and a tail longer than the history clamps to
 * zero rather than underflowing into a huge allowance. */
static void test_no_history(void)
{
  CHECK(cnc_backtrack_max_steps(RING, 0, 0, print_tail()) == 0);
  CHECK(cnc_backtrack_max_steps(RING, 0, 100, print_tail()) == 0);
  CHECK(cnc_backtrack_max_steps(RING, 0, 100, 100000u) == 0);
}

/* The dead stop: the start of the job while the job fits the ring, one ring
 * back once a live feed has wrapped it. Either way it is a real index behind
 * the write head, so a backward run cannot walk out of genuine data. */
static void test_dead_stop_span(void)
{
  const uint32_t in = 0x1234u;            /* wrapped indexes are ordinary */
  uint64_t job = 1024u * 1024u;
  uint64_t streamed = 100u * 1024u * 1024u;

  CHECK(cnc_backtrack_span(RING, job) == job);
  CHECK(cnc_backtrack_span(RING, streamed) == RING);
  CHECK(cnc_backtrack_span(RING, 0) == 0);
  /* in - span is the oldest genuine byte; the distance back to it is never
   * more than the ring holds. */
  CHECK((uint32_t)(in - (in - cnc_backtrack_span(RING, streamed))) == RING);
  CHECK((uint32_t)(in - (in - cnc_backtrack_span(RING, job))) == job);
}

/* What the allowance is for: a run that is allowed can always be asked for,
 * played out, and decelerated inside genuine data. */
static void test_allowed_run_fits_the_history(void)
{
  const uint64_t total = 100u * 1024u * 1024u;
  uint32_t unplayed = RING - GAP;
  uint32_t max = cnc_backtrack_max_steps(RING, unplayed, total, print_tail());
  uint32_t history = RING - unplayed;

  CHECK(max + print_tail() <= history);
  CHECK(PAUSE_TICKS + print_tail() <= history);
}

int main(void)
{
  test_decel_tail();
  test_preload_mid_job();
  test_preload_nothing_played();
  test_preload_early_pause();
  test_streamed_ring_full();
  test_streamed_ring_half();
  test_gap_is_the_floor();
  test_long_soak_does_not_wrap();
  test_no_history();
  test_dead_stop_span();
  test_allowed_run_fits_the_history();

  if (failures) {
    fprintf(stderr, "backtrack_test: %d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  printf("backtrack_test: all checks passed\n");
  return EXIT_SUCCESS;
}
