/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * ledtrig_smooth_fade.h
 *
 * The "smooth" LED trigger's fade: a critically damped mass-spring system
 * stepped every SMOOTH_MSECS_PER_UPDATE, in fixed point. The LED eases from
 * its value to its target; with both pulse periods set, the target swings
 * between 255 and 0 by itself. A fade ends once the value is within a
 * sixteenth of a count of the target, and the LED then stands on the target
 * itself, so a settled LED's target reads back the level that was written
 * (0 to 255), whatever path the fade took to it.
 *
 * Free of kernel dependencies so it builds and is tested on the host.
 *
 * Copyright 2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 * Copyright (C) 2015-2021 Glowforge, Inc. <opensource@glowforge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef KERNEL_SRC_LEDTRIG_SMOOTH_FADE_H_
#define KERNEL_SRC_LEDTRIG_SMOOTH_FADE_H_

#include "portable.h"

#ifndef __KERNEL__
#include <stdbool.h>
#endif

/** The fade's step, milliseconds. */
#define SMOOTH_MSECS_PER_UPDATE 20

/** The "speed" attribute is the damping constant as a 6.2 fixed-point
 *  number (shifted right by this), so the writable range has finer steps. */
#define SMOOTH_SPEED_C_SHIFT 2

/** The highest "speed": above it the fixed-point step can overflow. */
#define SMOOTH_SPEED_MAX 160

/**
 * Signed fixed point. Floating point is not available in the kernel, and
 * GCC's fixed-point extensions need libgcc, which a module does not link.
 */
#define SMOOTH_INT_BITS  20
#define SMOOTH_FRAC_BITS 12
typedef union {
  struct {
    unsigned int fpart:SMOOTH_FRAC_BITS;
    signed int ipart:SMOOTH_INT_BITS;
  };
  int32_t intval;
} fixed;

#define FIXED_INT(i)  (fixed){ .ipart=(i), .fpart=0 }
#define FIXED(i,f)    (fixed){ .ipart=(i), .fpart=(f)*(1<<SMOOTH_FRAC_BITS) } /* 0 <= f < 1 */

static inline fixed fixadd(fixed a, fixed b) {
  return (fixed)(a.intval + b.intval);
}

static inline fixed fixsub(fixed a, fixed b) {
  return (fixed)(a.intval - b.intval);
}

static inline fixed fixmul(fixed a, fixed b) {
  int64_t quot = (((int64_t)a.intval * (int64_t)b.intval) >> SMOOTH_FRAC_BITS);
  return (fixed)((int32_t)(quot));
}

static inline fixed fixdiv2(fixed a) {
  return (fixed)(a.intval >> 1);
}

static inline fixed fixdiv4(fixed a) {
  return (fixed)(a.intval >> 2);
}

static inline bool fixeq(fixed a, fixed b, fixed tolerance) {
  int32_t d = a.intval - b.intval;
  return (d < 0 ? -d : d) <= tolerance.intval;
}

/* The step, in seconds. */
static const fixed smooth_dt = FIXED(0, SMOOTH_MSECS_PER_UPDATE*0.001);

/* How close to its target a fade ends. */
static const fixed smooth_epsilon = FIXED(0, 0.0625);

/** One LED's fade: the parameters the attributes write and the system's
 *  state. All zero is an LED at rest at 0 with no damping set. */
struct smooth_fade {
  /* Parameters */
  fixed target;
  fixed c; /* damping constant; spring constant is derived */
  int pulse_on_count; /* multiples of SMOOTH_MSECS_PER_UPDATE */
  int pulse_off_count; /* multiples of SMOOTH_MSECS_PER_UPDATE */

  /* Simulation state */
  fixed value;
  fixed velocity;
  fixed accel;
  int pulse_counter;
};

/** An LED at rest on @level (0 to 255), damping at "speed" 64. */
static inline void smooth_fade_rest(struct smooth_fade *s, int level)
{
  *s = (struct smooth_fade){ 0 };
  s->value = FIXED_INT(level);
  s->target = s->value;
  s->c = FIXED_INT(16);
}

/** The target, set to @level (0 to 255). */
static inline void smooth_set_target(struct smooth_fade *s, unsigned int level)
{
  s->target = FIXED_INT(level);
}

/** The target as the attribute reads it. */
static inline int smooth_target(const struct smooth_fade *s)
{
  return s->target.ipart;
}

/** The damping, set from a "speed" (1 to SMOOTH_SPEED_MAX). */
static inline void smooth_set_speed(struct smooth_fade *s, unsigned int speed)
{
  s->c.intval = speed << (SMOOTH_FRAC_BITS - SMOOTH_SPEED_C_SHIFT);
}

/** The damping as the "speed" attribute reads it. */
static inline unsigned int smooth_speed(const struct smooth_fade *s)
{
  return (unsigned int)(uint8_t)(s->c.ipart << SMOOTH_SPEED_C_SHIFT);
}

/* The pulse's next target, while both periods are set. Returns whether the
 * LED pulses. */
static inline bool smooth_update_pulser(struct smooth_fade *s)
{
  bool pulsing = (s->pulse_on_count && s->pulse_off_count);
  if (!pulsing) { return false; }

  if (s->pulse_counter == 0) {
    s->target = FIXED_INT(255);
  } else if (s->pulse_counter == s->pulse_on_count) {
    s->target = FIXED_INT(0);
    s->pulse_counter = -s->pulse_off_count;
  }
  s->pulse_counter++;
  return true;
}

/** The pulse periods, in steps. Returns whether the LED now pulses. */
static inline bool smooth_set_pulse_periods(struct smooth_fade *s, int on, int off)
{
  /* When setting on/off to smaller values, */
  /* need to make sure to update the current count to prevent a wraparound, */
  /* or a period that's too long */
  if (on && s->pulse_counter > on) {
    s->pulse_counter = on;
  } else if (off && s->pulse_counter < -off) {
    s->pulse_counter = -(off-1);
  }
  s->pulse_on_count = on;
  s->pulse_off_count = off;
  if (!on || !off) { s->pulse_counter = 0; }
  return (on && off);
}

/**
 * One step of the fade. *@level is the level the LED shows after it, 0 to
 * 255. Returns true while the fade or the pulse goes on (the caller steps it
 * again in SMOOTH_MSECS_PER_UPDATE), false once the fade has settled: the
 * value is within a sixteenth of the target, and the LED stands on the
 * target, which is left as it was written.
 */
static inline bool smooth_step(struct smooth_fade *s, int *level)
{
  bool pulsing = smooth_update_pulser(s);
  bool settled;
  int lv;

  /* For a critically damped system (zeta=1), c/(2*sqrt(m*k)) = 1. */
  /* If m=1 (as in this case), then k = (c^2)/4. */
  /* https://en.wikipedia.org/wiki/Damping#Critical_damping_.28.CE.B6_.3D_1.29 */
  fixed k = fixdiv4(fixmul(s->c, s->c));

  /* Fspring = k * (target - value[t]) */
  fixed offset = fixsub(s->target, s->value);
  fixed fspring = fixmul(offset, k);
  /* -Fdamping = c * v[t] */
  fixed fdamping_neg = fixmul(s->c, s->velocity);
  /* a[t+dt] = Fspring - (-Fdamping) (mass = 1, no division necessary) */
  fixed new_accel = fixsub(fspring, fdamping_neg);

  /* Velocity Verlet integration. */
  /* https://en.wikipedia.org/wiki/Verlet_integration#Velocity_Verlet */
  /* Compute intermediate term: v[t] + (1/2)*a[t]*dt */
  fixed half_v = fixadd(s->velocity, fixmul(fixdiv2(s->accel), smooth_dt));
  /* x[t+dt] = x[t] + halfV*dt */
  /* (expanded: x[t+dt] = x[t] + v[t]*dt + (1/2)*a[t]*dt^2) */
  s->value = fixadd(s->value, fixmul(half_v, smooth_dt));
  /* v[t+dt] = halfV + (1/2)*a[t+dt]*dt */
  /* (expanded: v[t+dt] = v[t] + (1/2)*(a[t] + a[d+dt])*dt) */
  s->velocity = fixadd(half_v, fixmul(fixdiv2(new_accel), smooth_dt));
  s->accel = new_accel;

  /* Close enough: the LED stands on the target. A fade can stop just past
   * it (a fast one down to 0 a fraction below zero); the target is what was
   * written, and what the attribute reads back. */
  settled = !pulsing && fixeq(s->target, s->value, smooth_epsilon);
  if (settled) {
    s->value = s->target;
    s->velocity = FIXED_INT(0);
    s->accel = FIXED_INT(0);
  }

  lv = s->value.ipart;
  *level = lv < 0 ? 0 : lv > 255 ? 255 : lv;
  return !settled;
}

#endif
