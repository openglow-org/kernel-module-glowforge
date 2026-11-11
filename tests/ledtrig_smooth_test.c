// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ledtrig_smooth_test.c
 *
 * Host-side test for the "smooth" LED trigger's fade
 * (src/ledtrig_smooth_fade.h): a settled LED's target reads back the level
 * that was written and the LED stands on it. Every button pattern the
 * machine's writers use (forgectrl's, the GRBL controller's, the cloud
 * client's), put out at every step of its cycle, settles dark with its target
 * at 0; a fade from dark to any level at any of their speeds settles on that
 * level, 255 included. The attribute writes are replayed in the writers'
 * order, each followed by the step its store handler takes.
 *
 * Copyright 2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "ledtrig_smooth_fade.h"

#include <stdio.h>

static int failures = 0;

#define CHECK(cond, ...) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond); \
    fprintf(stderr, __VA_ARGS__); \
    fprintf(stderr, "\n"); \
    failures++; \
  } \
} while (0)

/* One LED as the driver runs it: the fade, whether its timer is armed, and
 * the level it shows. */
struct led {
  struct smooth_fade f;
  bool timer;
  int level;
};

static void step(struct led *l)
{
  l->timer = smooth_step(&l->f, &l->level);
}

/* The store handlers: each write, then the step it takes. */
static void wr_target(struct led *l, unsigned int v) { smooth_set_target(&l->f, v); step(l); }
static void wr_speed(struct led *l, unsigned int v) { smooth_set_speed(&l->f, v); step(l); }
static void wr_pulse_on(struct led *l, int ms)
{
  if (smooth_set_pulse_periods(&l->f, ms / SMOOTH_MSECS_PER_UPDATE, l->f.pulse_off_count))
    step(l);
}
static void wr_pulse_off(struct led *l, int ms)
{
  if (smooth_set_pulse_periods(&l->f, l->f.pulse_on_count, ms / SMOOTH_MSECS_PER_UPDATE))
    step(l);
}

/* A pattern as every writer puts it on one LED: the pulse stopped, the speed,
 * the target, then the pulse when there is one. */
static void pattern(struct led *l, unsigned int level, int on_ms, int off_ms, unsigned int speed)
{
  wr_pulse_on(l, 0);
  wr_pulse_off(l, 0);
  wr_speed(l, speed);
  wr_target(l, level);
  if (on_ms && level) {
    wr_pulse_on(l, on_ms);
    wr_pulse_off(l, off_ms);
  }
}

/* The timer's steps: n of them, or until it stops. */
static void run(struct led *l, int n)
{
  for (int i = 0; i < n && l->timer; i++)
    step(l);
}

/* Steps until the fade settles; false if it does not within 60 s. */
static bool settle(struct led *l)
{
  run(l, 60000 / SMOOTH_MSECS_PER_UPDATE);
  return !l->timer;
}

/* The patterns on one LED channel: forgectrl's (led.c), the GRBL
 * controller's (glowforge_laser.c button_lamp), the cloud client's
 * (gfhardware/leds.py). */
static const struct { const char *name; unsigned int level; int on, off; unsigned int speed; } patterns[] = {
  { "amber, red",          255,  350,  350, 160 },
  { "amber, green",        110,  350,  350, 160 },
  { "red flash",           255, 1000, 1000, 160 },
  { "teal, green",         180, 1400, 1400,  16 },
  { "teal, blue",          200, 1400, 1400,  16 },
  { "breathing white",     200, 1800, 1800,  12 },
  { "solid white",         255,    0,    0,  64 },
  { "solid green, blue",    40,    0,    0,  64 },
};

/* Any pattern, put out at any step of its cycle, settles dark: target 0,
 * nothing shown. */
static void test_put_out_reads_zero(void)
{
  for (unsigned int p = 0; p < sizeof(patterns) / sizeof(patterns[0]); p++) {
    int cycle = (patterns[p].on + patterns[p].off) / SMOOTH_MSECS_PER_UPDATE;
    int steps = cycle ? cycle : 100;
    int bad = 0;
    for (int at = 0; at < steps; at++) {
      struct led l;
      smooth_fade_rest(&l.f, 0);
      l.timer = false;
      pattern(&l, patterns[p].level, patterns[p].on, patterns[p].off, patterns[p].speed);
      run(&l, 2 * steps + at);              /* a cycle and more, then the put-out */
      pattern(&l, 0, 0, 0, 64);
      bool settled = settle(&l);
      if (!settled || smooth_target(&l.f) != 0 || l.level != 0) {
        if (!bad++)
          fprintf(stderr, "  %s put out at step %d: settled %d, target %d, level %d\n",
                  patterns[p].name, at, settled, smooth_target(&l.f), l.level);
      }
    }
    CHECK(bad == 0, "%s: %d of %d put-outs did not settle at target 0", patterns[p].name, bad, steps);
  }
}

/* A fade from dark to any level, at any writer's speed, settles on that
 * level: the target reads back as written and the LED shows it. */
static void test_fade_settles_on_its_level(void)
{
  static const unsigned int speeds[] = { 12, 16, 64, 160 };
  for (unsigned int s = 0; s < sizeof(speeds) / sizeof(speeds[0]); s++) {
    int bad = 0, first = -1, got_t = 0, got_l = 0;
    for (unsigned int v = 0; v <= 255; v++) {
      struct led l;
      smooth_fade_rest(&l.f, 0);
      l.timer = false;
      pattern(&l, v, 0, 0, speeds[s]);
      bool settled = settle(&l);
      if (!settled || smooth_target(&l.f) != (int)v || l.level != (int)v) {
        if (first < 0) { first = (int)v; got_t = smooth_target(&l.f); got_l = l.level; }
        bad++;
      }
    }
    CHECK(bad == 0, "speed %u: %d levels did not settle on themselves (first %d: target %d, level %d)",
          speeds[s], bad, first, got_t, got_l);
  }
}

int main(void)
{
  test_put_out_reads_zero();
  test_fade_settles_on_its_level();
  if (failures) {
    fprintf(stderr, "ledtrig_smooth_test: %d failure(s)\n", failures);
    return 1;
  }
  printf("ledtrig_smooth_test: ok\n");
  return 0;
}
