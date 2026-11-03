// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * pic_stats.c
 *
 * Rolling statistics for the PIC sampler. See pic_stats.h.
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
#include "pic_stats.h"

#ifdef __KERNEL__
#include <linux/math64.h>
#else
static inline uint64_t div_u64(uint64_t dividend, uint32_t divisor)
{
  return dividend / divisor;
}
#endif

size_t pic_stats_ring_len(unsigned int channels, unsigned int n_long)
{
  return (size_t)channels * n_long;
}


int pic_stats_init(struct pic_stats *st, unsigned int channels,
                   unsigned int n_short, unsigned int n_long, uint16_t *ring)
{
  unsigned int ch;

  if (channels == 0 || channels > PIC_STATS_MAX_CHANNELS ||
      n_short == 0 || n_short > n_long || !ring) {
    return -1;
  }
  st->channels = channels;
  st->n_short = n_short;
  st->n_long = n_long;
  st->ring = ring;
  st->head = 0;
  st->count = 0;
  for (ch = 0; ch < PIC_STATS_MAX_CHANNELS; ch++) {
    st->sum_short[ch] = 0;
    st->sum_long[ch] = 0;
  }
  return 0;
}


void pic_stats_push(struct pic_stats *st, const uint16_t *values)
{
  /* The slot n_short back from the head leaves the short window as this
   * sample enters it; the slot at the head leaves the long window. Both are
   * read before the head is overwritten. */
  unsigned int short_out = (st->head + st->n_long - st->n_short) % st->n_long;
  unsigned int ch;

  for (ch = 0; ch < st->channels; ch++) {
    uint16_t *ring = st->ring + (size_t)ch * st->n_long;

    if (st->count >= st->n_short) {
      st->sum_short[ch] -= ring[short_out];
    }
    if (st->count >= st->n_long) {
      st->sum_long[ch] -= ring[st->head];
    }
    ring[st->head] = values[ch];
    st->sum_short[ch] += values[ch];
    st->sum_long[ch] += values[ch];
  }
  st->head = (st->head + 1) % st->n_long;
  if (st->count < st->n_long) {
    st->count++;
  }
}


static uint64_t mean_milli(uint64_t sum, unsigned int n)
{
  return n ? div_u64(sum * 1000 + n / 2, n) : 0;
}


void pic_stats_summarize(const struct pic_stats *st, unsigned int ch,
                         struct pic_stats_summary *out)
{
  const uint16_t *ring = st->ring + (size_t)ch * st->n_long;
  unsigned int n_short = st->count < st->n_short ? st->count : st->n_short;
  unsigned int i;

  out->n_short = n_short;
  out->n_long = st->count;
  out->mean_short_milli = mean_milli(st->sum_short[ch], n_short);
  out->mean_long_milli = mean_milli(st->sum_long[ch], st->count);
  out->min_short = 0;
  out->max_short = 0;
  for (i = 0; i < n_short; i++) {
    uint16_t v = ring[(st->head + st->n_long - 1 - i) % st->n_long];

    if (i == 0 || v < out->min_short) {
      out->min_short = v;
    }
    if (i == 0 || v > out->max_short) {
      out->max_short = v;
    }
  }
}
