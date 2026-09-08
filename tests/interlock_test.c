// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * interlock_test.c
 *
 * Host-side test for the remote-interlock latch policy (src/cnc_interlock.c).
 * The policy decides the level of the INTERLOCK_LATCH_RESET line (1 = set the
 * hardware latch, emission blocked) from the gpio-keys switch device's view of
 * the loop. These tests pin the safety-relevant behavior:
 *
 *   - blocked until a switch device attaches and reports the loop closed;
 *   - an open loop sets the latch, a closed loop releases it;
 *   - losing the switch device (unobservable loop) re-blocks;
 *   - unrelated input events never touch the line;
 *   - the line is only written when the wanted level changes.
 *
 * Copyright 2020-2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "cnc_interlock.h"

#include <stdio.h>
#include <stdlib.h>

/* ---- drive recorder ---- */

static int drive_level = -1;   /* last level written to the line */
static int drive_writes = 0;   /* how many times the line was written */

static void record_drive(struct cnc_interlock *il, int level)
{
  (void)il;
  drive_level = level;
  drive_writes++;
}

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

static void reset_recorder(void)
{
  drive_level = -1;
  drive_writes = 0;
}

/* ---- cases ---- */

/* Fresh state: blocked, and the block is written out immediately. */
static void test_init_blocks(void)
{
  struct cnc_interlock il;
  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  CHECK(drive_level == 1);
  CHECK(drive_writes == 1);
  CHECK(cnc_interlock_level(&il) == 1);
}

/* A switch device attaching with the loop closed releases the latch;
 * attaching with the loop open keeps it set. */
static void test_attach_reports_loop(void)
{
  struct cnc_interlock il;

  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  cnc_interlock_attach(&il, false);
  CHECK(drive_level == 0);
  CHECK(drive_writes == 2);

  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  cnc_interlock_attach(&il, true);
  CHECK(drive_level == 1);
  CHECK(drive_writes == 1);   /* already blocked: no second write */
}

/* Loop edges after attach: open -> set, closed -> release. */
static void test_events_follow_loop(void)
{
  struct cnc_interlock il;
  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  cnc_interlock_attach(&il, false);
  CHECK(drive_level == 0);

  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 1);
  CHECK(drive_level == 1);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 0);
  CHECK(drive_level == 0);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 1);
  CHECK(drive_level == 1);
  CHECK(drive_writes == 5);   /* init, attach, open, closed, open */
}

/* Attached with the loop already open (a Pro whose external chain is
 * unlocked at boot): stays blocked, releases on the closing edge. */
static void test_open_at_boot(void)
{
  struct cnc_interlock il;
  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  cnc_interlock_attach(&il, true);
  CHECK(drive_level == 1);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 0);
  CHECK(drive_level == 0);
}

/* Anything that is not the interlock switch is ignored, in both level states. */
static void test_unrelated_events_ignored(void)
{
  struct cnc_interlock il;
  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  cnc_interlock_attach(&il, false);
  CHECK(drive_level == 0);
  cnc_interlock_event(&il, EV_SW, 3, 1);            /* doors */
  cnc_interlock_event(&il, EV_SW, 4, 0);            /* hv_enable readback */
  cnc_interlock_event(&il, EV_SW, 6, 1);            /* interlock_latch readback */
  cnc_interlock_event(&il, EV_KEY, CNC_INTERLOCK_SW_CODE, 1);
  cnc_interlock_event(&il, EV_SYN, 0, 0);
  CHECK(drive_level == 0);
  CHECK(drive_writes == 2);

  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 1);
  CHECK(drive_level == 1);
  cnc_interlock_event(&il, EV_SW, 3, 0);
  cnc_interlock_event(&il, EV_KEY, CNC_INTERLOCK_SW_CODE, 0);
  CHECK(drive_level == 1);
  CHECK(drive_writes == 3);
}

/* Repeated same-state events do not rewrite the line. */
static void test_no_redundant_writes(void)
{
  struct cnc_interlock il;
  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  cnc_interlock_attach(&il, false);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 0);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 0);
  CHECK(drive_writes == 2);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 1);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 1);
  CHECK(drive_writes == 3);
  CHECK(drive_level == 1);
}

/* Losing the switch device makes the loop unobservable: block, and stay
 * blocked through stray events, until a device attaches again. */
static void test_detach_blocks(void)
{
  struct cnc_interlock il;
  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  cnc_interlock_attach(&il, false);
  CHECK(drive_level == 0);
  cnc_interlock_detach(&il);
  CHECK(drive_level == 1);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 0);
  CHECK(drive_level == 1);            /* nobody attached: a stray "closed" cannot release */
  cnc_interlock_attach(&il, false);
  CHECK(drive_level == 0);
  /* Detaching more times than attached never goes negative or releases. */
  cnc_interlock_detach(&il);
  cnc_interlock_detach(&il);
  CHECK(drive_level == 1);
  cnc_interlock_attach(&il, false);
  CHECK(drive_level == 0);
}

/* Two devices reporting the switch: the line follows the latest reading and
 * stays observable while at least one device remains. */
static void test_multiple_devices(void)
{
  struct cnc_interlock il;
  reset_recorder();
  cnc_interlock_init(&il, record_drive);
  cnc_interlock_attach(&il, false);
  cnc_interlock_attach(&il, false);
  CHECK(drive_level == 0);
  cnc_interlock_detach(&il);
  CHECK(drive_level == 0);
  cnc_interlock_event(&il, EV_SW, CNC_INTERLOCK_SW_CODE, 1);
  CHECK(drive_level == 1);
  cnc_interlock_detach(&il);
  CHECK(drive_level == 1);
}

int main(void)
{
  test_init_blocks();
  test_attach_reports_loop();
  test_events_follow_loop();
  test_open_at_boot();
  test_unrelated_events_ignored();
  test_no_redundant_writes();
  test_detach_blocks();
  test_multiple_devices();

  if (failures) {
    fprintf(stderr, "interlock_test: %d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  printf("interlock_test: all checks passed\n");
  return EXIT_SUCCESS;
}
