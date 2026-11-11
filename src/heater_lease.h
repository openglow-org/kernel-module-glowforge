/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * heater_lease.h
 *
 * The loop heater's lease: a duty above zero holds only for the lease
 * (heater_lease_ms) after the write that set it. A writer that wants the
 * heater on writes it again within the lease, and keeps doing so; when
 * the writes stop (the writer died, hung, or forgot), the heater goes off
 * by itself at the end of the lease. A write of zero needs no lease. The
 * heater's software PWM looks at the lease on every edge, so a lapsed
 * heater is off within one PWM period of its lease's end.
 *
 * Free of kernel dependencies so it builds and is tested on the host.
 *
 * Copyright 2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
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
#ifndef KERNEL_SRC_HEATER_LEASE_H_
#define KERNEL_SRC_HEATER_LEASE_H_

/** The lease, milliseconds: the default and the range a parameter is held
 *  to. The minimum is two of a writer's periods (the writers write once a
 *  second), so one late write does not end a lease at the minimum. */
#define HEATER_LEASE_MS_DEFAULT 5000u
#define HEATER_LEASE_MS_MIN     2000u
#define HEATER_LEASE_MS_MAX     60000u

/** The heater's software PWM: its period, and the duty's full scale (a
 *  16-bit fraction, HEATER_DUTY_MAX for always on). */
#define HEATER_PWM_PERIOD_NS 10000000LL
#define HEATER_DUTY_BITS     16
#define HEATER_DUTY_MAX      65535u

/** A lease parameter held to its range. */
static inline unsigned int heater_lease_ms_held(unsigned int ms)
{
  return ms < HEATER_LEASE_MS_MIN ? HEATER_LEASE_MS_MIN
       : ms > HEATER_LEASE_MS_MAX ? HEATER_LEASE_MS_MAX : ms;
}

/** The lease the parameter *@ms asks for, held to its range and stored
 *  back, so the parameter reads back as the lease in use. Returns it. */
static inline unsigned int heater_lease_ms_take(unsigned int *ms)
{
  *ms = heater_lease_ms_held(*ms);
  return *ms;
}

/** When a write made at @now_ns stops holding, for a lease of @lease_ms. */
static inline long long heater_lease_until(long long now_ns, unsigned int lease_ms)
{
  return now_ns + (long long)heater_lease_ms_held(lease_ms) * 1000000LL;
}

/** Whether a heater at @duty has outlived its lease at @now_ns. */
static inline int heater_lease_lapsed(unsigned int duty, long long now_ns, long long until_ns)
{
  return duty != 0 && now_ns >= until_ns;
}

/** A write of @new_duty at @now_ns, for a lease of @lease_ms: the duty
 *  becomes @new_duty, and a duty above zero starts a new lease. A write of
 *  zero leaves the lease's end where it was. The caller holds whatever
 *  guards @duty and @until_ns. */
static inline void heater_lease_write(unsigned short *duty, long long *until_ns,
                                      unsigned short new_duty, long long now_ns,
                                      unsigned int lease_ms)
{
  if (new_duty != 0) {
    *until_ns = heater_lease_until(now_ns, lease_ms);
  }
  *duty = new_duty;
}

/** A write of @value asked for at @now_ns (the heater_pwm attribute, the
 *  safe state): over HEATER_DUTY_MAX it is refused (-1) and changes
 *  nothing, neither the duty nor the lease's end; anything else is
 *  heater_lease_write (0). The caller holds whatever guards them. */
static inline int heater_lease_store(unsigned short *duty, long long *until_ns,
                                     unsigned long value, long long now_ns,
                                     unsigned int lease_ms)
{
  if (value > HEATER_DUTY_MAX) {
    return -1;
  }
  heater_lease_write(duty, until_ns, (unsigned short)value, now_ns, lease_ms);
  return 0;
}

/** One look at the lease at @now_ns, as the heater's timer takes on every
 *  edge: a heater that has outlived its lease goes to zero and the lapse is
 *  counted, once. Returns whether this look ended the lease; the duty to
 *  drive is *@duty afterward. The caller holds whatever guards them. */
static inline int heater_lease_tick(unsigned short *duty, long long until_ns,
                                    long long now_ns, unsigned int *lapses)
{
  if (!heater_lease_lapsed(*duty, now_ns, until_ns)) {
    return 0;
  }
  *duty = 0;
  (*lapses)++;
  return 1;
}

/** The time to the PWM's next edge for @frac of the period, in
 *  HEATER_DUTY_MAX + 1 parts: the whole period at 0 and at full (the level
 *  holds), that part of it otherwise; never more than one period. */
static inline long long heater_pwm_interval_ns(unsigned int frac)
{
  if (frac == 0 || frac == HEATER_DUTY_MAX) {
    return HEATER_PWM_PERIOD_NS;
  }
  return (HEATER_PWM_PERIOD_NS * frac) >> HEATER_DUTY_BITS;
}

/** One edge of the heater's software PWM at @now_ns, as its timer takes
 *  it: first the look at the lease (heater_lease_tick), then the pin's
 *  next level from the duty that look leaves. At 0 the pin is low and at
 *  full it is high, each for a whole period; between them it takes the
 *  other level from @level_now (the pin's level now), high for the duty's
 *  part of the period and low for the rest. *@next_level is the level to
 *  drive (1: the heater on), *@next_ns the time to the next edge. Returns
 *  whether this look ended the lease. The caller holds whatever guards the
 *  duty, the lease and the count. */
static inline int heater_pwm_edge(unsigned short *duty, long long until_ns, long long now_ns,
                                  unsigned int *lapses, int level_now,
                                  int *next_level, long long *next_ns)
{
  int lapsed = heater_lease_tick(duty, until_ns, now_ns, lapses);
  unsigned int d = *duty;

  if (d == 0 || d == HEATER_DUTY_MAX) {
    *next_level = d != 0;
    *next_ns = heater_pwm_interval_ns(d);
  } else {
    *next_level = !level_now;
    *next_ns = heater_pwm_interval_ns(*next_level ? d : HEATER_DUTY_MAX - d);
  }
  return lapsed;
}

#endif
