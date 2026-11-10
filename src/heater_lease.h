/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * heater_lease.h
 *
 * The loop heater's lease: a duty above zero holds only for the lease
 * (heater_lease_ms) after the write that set it. A writer that wants the
 * heater on writes it again within the lease, and keeps doing so; when
 * the writes stop (the writer died, hung, or forgot), the heater goes off
 * by itself at the end of the lease. A write of zero needs no lease.
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

/** A lease parameter held to its range. */
static inline unsigned int heater_lease_ms_held(unsigned int ms)
{
  return ms < HEATER_LEASE_MS_MIN ? HEATER_LEASE_MS_MIN
       : ms > HEATER_LEASE_MS_MAX ? HEATER_LEASE_MS_MAX : ms;
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

#endif
