/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * pic_stats.h
 *
 * Rolling statistics for the PIC sampler: per channel, the last n_long
 * samples in a ring, with running sums that give the mean of the newest
 * n_short samples (the short window) and of all n_long (the long window)
 * exactly, and the short window's minimum and maximum by a scan. The
 * sampler sizes the windows from its rate: one second and ten seconds.
 *
 * Free of kernel dependencies so it builds and is tested on the host; the
 * caller provides the ring's storage (channels * n_long values).
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
#ifndef KERNEL_SRC_PIC_STATS_H_
#define KERNEL_SRC_PIC_STATS_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stddef.h>
#include <stdint.h>
#endif

/** The most channels one set of statistics carries. */
#define PIC_STATS_MAX_CHANNELS 16

struct pic_stats {
  unsigned int channels;
  /** Samples in the short and the long window; n_short <= n_long. */
  unsigned int n_short;
  unsigned int n_long;
  /** Ring storage, channels * n_long values, channel-major. */
  uint16_t *ring;
  /** Next slot written, shared by every channel. */
  unsigned int head;
  /** Samples held, up to n_long. */
  unsigned int count;
  uint64_t sum_short[PIC_STATS_MAX_CHANNELS];
  uint64_t sum_long[PIC_STATS_MAX_CHANNELS];
};

/** One channel's view of both windows. */
struct pic_stats_summary {
  /** Samples in each window: fewer than its size until it fills. */
  unsigned int n_short;
  unsigned int n_long;
  /** Means in thousandths of a count, rounded half up; 0 when empty. */
  uint64_t mean_short_milli;
  uint64_t mean_long_milli;
  /** The short window's extremes; 0 when empty. */
  uint16_t min_short;
  uint16_t max_short;
};

/** Number of ring values to allocate for @channels and @n_long. */
size_t pic_stats_ring_len(unsigned int channels, unsigned int n_long);

/**
 * Initializes empty statistics over @ring (pic_stats_ring_len values).
 * Returns 0, or -1 when the sizes do not fit (no channels, more than
 * PIC_STATS_MAX_CHANNELS, an empty window, n_short > n_long).
 */
int pic_stats_init(struct pic_stats *st, unsigned int channels,
                   unsigned int n_short, unsigned int n_long, uint16_t *ring);

/** Adds one sample: @values holds one value per channel. */
void pic_stats_push(struct pic_stats *st, const uint16_t *values);

/** Fills @out with channel @ch's windows. */
void pic_stats_summarize(const struct pic_stats *st, unsigned int ch,
                         struct pic_stats_summary *out);

#endif
