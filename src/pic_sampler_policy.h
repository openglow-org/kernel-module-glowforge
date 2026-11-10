/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * pic_sampler_policy.h
 *
 * The PIC sampler's judgments about time, free of kernel dependencies so
 * they build and are tested on the host. A read's time is taken as the read
 * starts, before its transfer, so a read never looks newer than it is.
 *
 * - When its newest read is too old to report: one second, or two periods
 *   at a rate under 2 Hz, so a read on time is never refused.
 * - Whether a reader gets the newest read at all: only while the sampler
 *   runs, has a read, and that read is not too old.
 * - When a run of reads is broken: a gap of more than two periods since the
 *   last good read (a stall, an overrun, failed reads between). The windows
 *   then start over, so "the last second" holds only reads from the last
 *   second, never an old run with a new read on its end.
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
#ifndef KERNEL_SRC_PIC_SAMPLER_POLICY_H_
#define KERNEL_SRC_PIC_SAMPLER_POLICY_H_

#define PIC_SAMPLER_NS_PER_S 1000000000LL

/** The period at @hz reads a second, nanoseconds (@hz at least 1). A
 *  32-bit division: a second's nanoseconds fit, and 32-bit ARM has no
 *  64-bit divide in the kernel. */
static inline long long pic_sampler_period_ns(unsigned int hz)
{
  return (long long)(1000000000u / (hz ? hz : 1u));
}

/** The age past which the newest read is not reported. */
static inline long long pic_sampler_stale_ns(unsigned int hz)
{
  long long two = 2 * pic_sampler_period_ns(hz);
  return two > PIC_SAMPLER_NS_PER_S ? two : PIC_SAMPLER_NS_PER_S;
}

/** Whether the newest read is current enough to report at @now_ns: the
 *  sampler is @running, it has a read (@read_ns is its time, 0: none yet),
 *  and the read is no older than the stale age. */
static inline int pic_sampler_fresh(int running, long long read_ns, long long now_ns, unsigned int hz)
{
  return running && read_ns != 0 && now_ns - read_ns <= pic_sampler_stale_ns(hz);
}

/** Whether a good read at @now_ns breaks the run that ended at @prev_ns
 *  (0: no read yet, nothing to break). */
static inline int pic_sampler_run_broken(long long prev_ns, long long now_ns, unsigned int hz)
{
  return prev_ns != 0 && now_ns - prev_ns > 2 * pic_sampler_period_ns(hz);
}

#endif
