/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * pic_sampler_core.h
 *
 * The PIC sampler's work, free of kernel dependencies so it builds and is
 * tested on the host. pic_sampler.c gives it the clock, the transfer, the
 * lock, the ring's memory and the thread (struct pic_sampler_ops).
 *
 * - A cycle: the read's time, then the transfer, then the read taken under
 *   the lock. A read counts only with every register in it and the PIC's ID
 *   word in register 0; any other read counts as an error and empties the
 *   windows. A good read after a gap (pic_sampler_run_broken) empties them
 *   before it joins them.
 * - The tick: an absolute grid of one period; after an overrun the grid
 *   restarts from now instead of firing the missed ticks back to back.
 * - The readers: under the lock, the clock, then the freshness check
 *   (pic_sampler_fresh), then the copy; the text and the bytes are made
 *   from the copy, outside the lock.
 * - Start and stop: the ring is allocated and the statistics initialized
 *   before the sampler is marked running, and a start that fails leaves it
 *   not running with nothing allocated. The ring is freed only after the
 *   thread has stopped and running is clear, so neither the thread nor a
 *   reader touches freed memory.
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
#ifndef KERNEL_SRC_PIC_SAMPLER_CORE_H_
#define KERNEL_SRC_PIC_SAMPLER_CORE_H_

#include "portable.h"
#include "pic_stats.h"
#include "pic_sampler_policy.h"
#include "uapi/glowforge.h"

/** An analog channel the statistics carry: named as its attribute, from
 *  the register that attribute reads. */
struct pic_sampler_channel {
  const char *name;
  enum pic_register reg;
};

static const struct pic_sampler_channel pic_sampler_channels[] = {
  { "water_temp_1", PIC_REG_WATER_THERM1 },
  { "water_temp_2", PIC_REG_WATER_THERM2 },
  { "tec_temp", PIC_REG_TEC_THERM },
  { "pwr_temp", PIC_REG_PWR_THERM },
  { "lid_ir_1", PIC_REG_LID_IR_DET1 },
  { "lid_ir_2", PIC_REG_LID_IR_DET2 },
  { "lid_ir_3", PIC_REG_LID_IR_DET3 },
  { "lid_ir_4", PIC_REG_LID_IR_DET4 },
  { "hv_current", PIC_REG_HV_ISENSE },
  { "hv_voltage", PIC_REG_HV_VSENSE },
};
#define PIC_SAMPLER_CHANNELS (sizeof(pic_sampler_channels) / sizeof(pic_sampler_channels[0]))

/** The snapshot's length: the read's time, then every register. */
#define PIC_SAMPLER_SNAPSHOT_LEN (8 + PIC_NUM_REGISTERS * 2)

/** The sampler's state. Everything here is under the sampler's lock. */
struct pic_sampler_core {
  /** Set while the thread runs and the statistics may be read. */
  int running;
  unsigned int hz;
  struct pic_stats stats;
  /** The statistics' ring, from a start to its stop. */
  uint16_t *ring;
  /** The newest good read, and its CLOCK_MONOTONIC time in ns (0: none yet). */
  pic_value regs[PIC_NUM_REGISTERS];
  long long read_ns;
  uint64_t reads;
  uint64_t errors;
};

/** The kernel's side; ctx is the caller's own. */
struct pic_sampler_ops {
  /** CLOCK_MONOTONIC, ns. */
  long long (*now)(void *ctx);
  /** Every register in one transfer into @regs: the bytes read, or a
   *  negative errno. */
  int (*read)(void *ctx, pic_value *regs);
  /** The sampler's lock, which the thread and the readers take. */
  void (*lock)(void *ctx);
  void (*unlock)(void *ctx);
  /** @len zeroed values for the ring, or NULL; and their release. */
  uint16_t *(*alloc_ring)(void *ctx, size_t len);
  void (*free_ring)(void *ctx, uint16_t *ring);
  /** The thread: started (0, or a negative errno), and stopped (returns
   *  once it has ended). */
  int (*thread_start)(void *ctx);
  void (*thread_stop)(void *ctx);
};

/** One read, under the lock: @ret is its transfer's result, @start_ns the
 *  time it started. A read counts only with every register in it and the
 *  PIC's ID word in register 0: the bus master never errors on a dead or
 *  reset PIC, it reads all zeros or all ones. Any other read counts as an
 *  error and empties the windows, and leaves the newest good read as it
 *  was. Returns 0 for a read that counts, -1 for an error. */
static inline int pic_sampler_take(struct pic_sampler_core *c, long long start_ns, int ret,
                                   const pic_value *regs)
{
  uint16_t values[PIC_SAMPLER_CHANNELS];
  unsigned int i;

  if (ret != (int)(PIC_NUM_REGISTERS * sizeof(pic_value)) ||
      regs[PIC_REG_DUMMY - PIC_FIRST_REGISTER] != PIC_MAGIC_NUMBER) {
    c->errors++;
    pic_stats_clear(&c->stats);
    return -1;
  }
  /* The windows hold one unbroken run of reads (pic_sampler_policy.h).
   * The gap is from one read's start to the next: a transfer that stalls
   * for more than a period puts the next tick a period after its end, so
   * the read after it finds the gap. */
  if (pic_sampler_run_broken(c->read_ns, start_ns, c->hz)) {
    pic_stats_clear(&c->stats);
  }
  for (i = 0; i < PIC_NUM_REGISTERS; i++) {
    c->regs[i] = regs[i];
  }
  c->read_ns = start_ns;
  for (i = 0; i < PIC_SAMPLER_CHANNELS; i++) {
    values[i] = regs[pic_sampler_channels[i].reg];
  }
  pic_stats_push(&c->stats, values);
  c->reads++;
  return 0;
}

/** One cycle of the thread: the read's time, the transfer, then the read
 *  taken under the lock. Returns pic_sampler_take's verdict. */
static inline int pic_sampler_cycle(struct pic_sampler_core *c, const struct pic_sampler_ops *o,
                                    void *ctx)
{
  pic_value regs[PIC_NUM_REGISTERS];
  /* The read's time is taken before the transfer: whatever delays the
   * thread between the transfer and the lock, the read is reported at
   * least as old as it is, never newer. */
  long long start_ns = o->now(ctx);
  int ret = o->read(ctx, regs);

  o->lock(ctx);
  ret = pic_sampler_take(c, start_ns, ret, regs);
  o->unlock(ctx);
  return ret;
}

/** The thread's next tick after the one at @next_ns, at @now_ns: the next
 *  on the absolute grid of @period_ns; after an overrun the grid restarts
 *  from now instead of firing the missed ticks back to back. */
static inline long long pic_sampler_next_tick(long long next_ns, long long now_ns, long long period_ns)
{
  next_ns += period_ns;
  return next_ns <= now_ns ? now_ns + period_ns : next_ns;
}

/** What the means report, copied under the lock. */
struct pic_sampler_means {
  unsigned int hz;
  long long age_ns;
  uint64_t reads;
  uint64_t errors;
  struct pic_stats_summary sum[PIC_SAMPLER_CHANNELS];
};

/** Under the lock, at @now_ns: the means' copy, or -1 when the newest read
 *  is not current enough to report (pic_sampler_fresh). */
static inline int pic_sampler_means_take(const struct pic_sampler_core *c, long long now_ns,
                                         struct pic_sampler_means *m)
{
  unsigned int i;

  if (!pic_sampler_fresh(c->running, c->read_ns, now_ns, c->hz)) {
    return -1;
  }
  for (i = 0; i < PIC_SAMPLER_CHANNELS; i++) {
    pic_stats_summarize(&c->stats, i, &m->sum[i]);
  }
  m->hz = c->hz;
  m->age_ns = now_ns - c->read_ns;
  m->reads = c->reads;
  m->errors = c->errors;
  return 0;
}

/** A reader's copy of the means: 0, or -1 (refused, nothing copied). */
static inline int pic_sampler_means_read(const struct pic_sampler_core *c,
                                         const struct pic_sampler_ops *o, void *ctx,
                                         struct pic_sampler_means *m)
{
  int ret;

  o->lock(ctx);
  /* The clock under the lock: a reader preempted before the lock must not
   * under-report the age. */
  ret = pic_sampler_means_take(c, o->now(ctx), m);
  o->unlock(ctx);
  return ret;
}

/** The means as text, a contract (forgectrl's flowrun.c parses it): a first
 *  line "hz H age_ms A reads R errors E", then a line a channel in
 *  pic_sampler_channels' order, "name mean_1s mean_10s min_1s max_1s n_1s
 *  n_10s", the means in counts with three decimals. Returns its length. */
static inline int pic_sampler_means_format(char *buf, size_t cap, const struct pic_sampler_means *m)
{
  unsigned int i;
  int len = scnprintf(buf, cap, "hz %u age_ms %lld reads %llu errors %llu\n", m->hz,
                      (long long)div_s64(m->age_ns, 1000000), (unsigned long long)m->reads,
                      (unsigned long long)m->errors);

  for (i = 0; i < PIC_SAMPLER_CHANNELS; i++) {
    const struct pic_stats_summary *s = &m->sum[i];
    uint32_t short_frac, long_frac;
    uint64_t short_int = div_u64_rem(s->mean_short_milli, 1000, &short_frac);
    uint64_t long_int = div_u64_rem(s->mean_long_milli, 1000, &long_frac);

    len += scnprintf(buf + len, cap - len, "%s %llu.%03u %llu.%03u %u %u %u %u\n",
                     pic_sampler_channels[i].name, (unsigned long long)short_int, short_frac,
                     (unsigned long long)long_int, long_frac, (unsigned int)s->min_short,
                     (unsigned int)s->max_short, s->n_short, s->n_long);
  }
  return len;
}

/** The newest read as the snapshot reports it. */
struct pic_sampler_snapshot {
  long long read_ns;
  pic_value regs[PIC_NUM_REGISTERS];
};

/** A reader's copy of the newest read: 0, or -1 (refused, nothing copied),
 *  judged as the means are. */
static inline int pic_sampler_snapshot_read(const struct pic_sampler_core *c,
                                            const struct pic_sampler_ops *o, void *ctx,
                                            struct pic_sampler_snapshot *out)
{
  unsigned int i;
  int ret = -1;

  o->lock(ctx);
  if (pic_sampler_fresh(c->running, c->read_ns, o->now(ctx), c->hz)) {
    out->read_ns = c->read_ns;
    for (i = 0; i < PIC_NUM_REGISTERS; i++) {
      out->regs[i] = c->regs[i];
    }
    ret = 0;
  }
  o->unlock(ctx);
  return ret;
}

/** The snapshot's bytes into @buf: the read's CLOCK_MONOTONIC time in ns,
 *  little-endian 64-bit, then every register in register order (grp_all's),
 *  little-endian 16-bit. Returns PIC_SAMPLER_SNAPSHOT_LEN. */
static inline int pic_sampler_snapshot_pack(void *buf, const struct pic_sampler_snapshot *s)
{
  unsigned char *out = buf;
  unsigned long long t = (unsigned long long)s->read_ns;
  unsigned int i;

  for (i = 0; i < 8; i++) {
    out[i] = (unsigned char)(t >> (8 * i));
  }
  for (i = 0; i < PIC_NUM_REGISTERS; i++) {
    out[8 + 2 * i] = (unsigned char)(s->regs[i] & 0xff);
    out[9 + 2 * i] = (unsigned char)(s->regs[i] >> 8);
  }
  return PIC_SAMPLER_SNAPSHOT_LEN;
}

/** How a start ended. */
enum pic_sampler_start {
  PIC_SAMPLER_STARTED,
  /** 0 reads a second were asked for. */
  PIC_SAMPLER_OFF,
  /** No memory for the ring. */
  PIC_SAMPLER_NO_MEMORY,
  /** The statistics did not initialize. */
  PIC_SAMPLER_NO_STATS,
  /** The thread did not start (its errno in *err). */
  PIC_SAMPLER_NO_THREAD,
};

/** Starts the sampler at @hz reads a second (already held to its range),
 *  with windows of one and ten seconds. Anything but PIC_SAMPLER_STARTED
 *  leaves it not running with nothing allocated, so the means and the
 *  snapshot refuse and the rest of the PIC works on. */
static inline enum pic_sampler_start pic_sampler_start_core(struct pic_sampler_core *c,
                                                            const struct pic_sampler_ops *o,
                                                            void *ctx, unsigned int hz, int *err)
{
  uint16_t *ring;

  *err = 0;
  if (hz == 0) {
    return PIC_SAMPLER_OFF;
  }
  ring = o->alloc_ring(ctx, pic_stats_ring_len(PIC_SAMPLER_CHANNELS, 10 * hz));
  if (!ring) {
    return PIC_SAMPLER_NO_MEMORY;
  }
  if (pic_stats_init(&c->stats, PIC_SAMPLER_CHANNELS, hz, 10 * hz, ring) < 0) {
    o->free_ring(ctx, ring);
    return PIC_SAMPLER_NO_STATS;
  }
  c->ring = ring;
  c->hz = hz;
  c->read_ns = 0;
  c->reads = 0;
  c->errors = 0;
  o->lock(ctx);
  c->running = 1;
  o->unlock(ctx);
  *err = o->thread_start(ctx);
  if (*err) {
    o->lock(ctx);
    c->running = 0;
    o->unlock(ctx);
    o->free_ring(ctx, c->ring);
    c->ring = NULL;
    return PIC_SAMPLER_NO_THREAD;
  }
  return PIC_SAMPLER_STARTED;
}

/** Stops the sampler: the thread first, then running cleared under the
 *  lock, then the ring freed. Readers only touch the statistics under the
 *  lock and while running, so the ring can go once running is clear. */
static inline void pic_sampler_stop_core(struct pic_sampler_core *c, const struct pic_sampler_ops *o,
                                         void *ctx)
{
  o->thread_stop(ctx);
  o->lock(ctx);
  c->running = 0;
  o->unlock(ctx);
  o->free_ring(ctx, c->ring);
  c->ring = NULL;
}

#endif
