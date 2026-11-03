// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * pic_sampler.c
 *
 * The PIC sampler: a kernel thread that reads every PIC register in one SPI
 * transaction (pic_read_register_range, so with the same pic_settle_us of
 * busy CPU before it as any other read) pic_sampler_hz times a second,
 * keeps the newest read, and keeps one- and ten-second rolling statistics
 * of the analog channels (pic_stats.c). Every channel of one read is
 * converted under the same conditions, and a reader takes a mean of many
 * reads instead of one noisy read of its own.
 *
 * /sys/glowforge/pic/means and /sys/glowforge/pic/snapshot report them; both
 * refuse (-ENODATA) while the sampler is off or its newest read is more than
 * a second old, so a reader never takes stale values for current ones. The
 * single-register attributes are unchanged and still read the PIC live.
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

#include "pic_private.h"
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

/** Module parameter (glowforge.c): reads a second, 0 = no sampler. */
extern unsigned int pic_sampler_hz;

#define PIC_SAMPLER_MAX_HZ      200
/** Newest read older than this: the attributes refuse. */
#define PIC_SAMPLER_STALE_NS    NSEC_PER_SEC
/** Timer slack on each tick. */
#define PIC_SAMPLER_SLACK_NS    (100 * NSEC_PER_USEC)
/** The snapshot: s64 read time, then every register. */
#define PIC_SAMPLER_SNAPSHOT_LEN (sizeof(s64) + PIC_NUM_REGISTERS * sizeof(pic_value))

/** The analog channels the statistics carry, named as their attributes. */
static const struct {
  const char *name;
  enum pic_register reg;
} pic_sampler_channels[] = {
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
#define PIC_SAMPLER_CHANNELS ARRAY_SIZE(pic_sampler_channels)


void pic_sampler_init(struct spi_device *spi)
{
  struct pic *self = spi_get_drvdata(spi);
  struct pic_sampler *s = &self->sampler;

  spin_lock_init(&s->lock);
  s->running = false;
  s->task = NULL;
  s->ring = NULL;
}


static int pic_sampler_thread(void *data)
{
  struct spi_device *spi = data;
  struct pic *self = spi_get_drvdata(spi);
  struct pic_sampler *s = &self->sampler;
  const ktime_t period = ns_to_ktime(div_u64(NSEC_PER_SEC, s->hz));
  ktime_t next = ktime_get();
  pic_value regs[PIC_NUM_REGISTERS];
  uint16_t values[PIC_SAMPLER_CHANNELS];
  unsigned int i;

  while (!kthread_should_stop()) {
    ssize_t ret = pic_read_register_range(spi, PIC_FIRST_REGISTER, PIC_LAST_REGISTER, (char *)regs);
    s64 now_ns = ktime_get_ns();

    spin_lock(&s->lock);
    if (ret == (ssize_t)sizeof(regs)) {
      memcpy(s->regs, regs, sizeof(regs));
      s->read_ns = now_ns;
      for (i = 0; i < PIC_SAMPLER_CHANNELS; i++) {
        values[i] = regs[pic_sampler_channels[i].reg];
      }
      pic_stats_push(&s->stats, values);
      s->reads++;
    } else {
      s->errors++;
    }
    spin_unlock(&s->lock);

    /* The next tick on the absolute grid; after an overrun the grid
     * restarts from now instead of firing the missed ticks back to back. */
    next = ktime_add(next, period);
    if (ktime_compare(next, ktime_get()) <= 0) {
      next = ktime_add(ktime_get(), period);
    }
    set_current_state(TASK_INTERRUPTIBLE);
    if (kthread_should_stop()) {
      __set_current_state(TASK_RUNNING);
      break;
    }
    schedule_hrtimeout_range(&next, PIC_SAMPLER_SLACK_NS, HRTIMER_MODE_ABS);
  }
  return 0;
}


void pic_sampler_start(struct spi_device *spi)
{
  struct pic *self = spi_get_drvdata(spi);
  struct pic_sampler *s = &self->sampler;
  unsigned int hz = READ_ONCE(pic_sampler_hz);
  struct task_struct *task;

  if (hz == 0) {
    dev_info(&spi->dev, "sampler off (pic_sampler_hz=0)");
    return;
  }
  if (hz > PIC_SAMPLER_MAX_HZ) {
    dev_warn(&spi->dev, "pic_sampler_hz=%u is over %u; sampling at %u", hz, PIC_SAMPLER_MAX_HZ,
             PIC_SAMPLER_MAX_HZ);
    hz = PIC_SAMPLER_MAX_HZ;
  }
  s->ring = kvcalloc(pic_stats_ring_len(PIC_SAMPLER_CHANNELS, 10 * hz), sizeof(uint16_t), GFP_KERNEL);
  if (!s->ring) {
    dev_warn(&spi->dev, "sampler off: no memory for its statistics");
    return;
  }
  /* Windows of one and ten seconds. */
  if (pic_stats_init(&s->stats, PIC_SAMPLER_CHANNELS, hz, 10 * hz, s->ring) < 0) {
    dev_warn(&spi->dev, "sampler off: its statistics did not initialize");
    kvfree(s->ring);
    s->ring = NULL;
    return;
  }
  s->hz = hz;
  s->read_ns = 0;
  s->reads = 0;
  s->errors = 0;
  spin_lock(&s->lock);
  s->running = true;
  spin_unlock(&s->lock);
  task = kthread_run(pic_sampler_thread, spi, "pic-sampler");
  if (IS_ERR(task)) {
    dev_warn(&spi->dev, "sampler off: its thread did not start (%ld)", PTR_ERR(task));
    spin_lock(&s->lock);
    s->running = false;
    spin_unlock(&s->lock);
    kvfree(s->ring);
    s->ring = NULL;
    return;
  }
  s->task = task;
  dev_info(&spi->dev, "sampler at %u Hz", hz);
}


void pic_sampler_stop(struct spi_device *spi)
{
  struct pic *self = spi_get_drvdata(spi);
  struct pic_sampler *s = &self->sampler;

  if (s->task) {
    kthread_stop(s->task);
    s->task = NULL;
  }
  /* Readers only touch the statistics under the lock and while running,
   * so the ring can go once running is clear. */
  spin_lock(&s->lock);
  s->running = false;
  spin_unlock(&s->lock);
  kvfree(s->ring);
  s->ring = NULL;
}


/* Under the lock: whether the newest read is current enough to report. */
static bool pic_sampler_fresh_locked(const struct pic_sampler *s, s64 now_ns)
{
  return s->running && s->read_ns != 0 && now_ns - s->read_ns <= PIC_SAMPLER_STALE_NS;
}


ssize_t pic_sampler_means_show(struct spi_device *spi, char *buf)
{
  struct pic *self = spi_get_drvdata(spi);
  struct pic_sampler *s = &self->sampler;
  struct pic_stats_summary sum[PIC_SAMPLER_CHANNELS];
  s64 now_ns = ktime_get_ns();
  s64 age_ns;
  unsigned int hz;
  u64 reads, errors;
  unsigned int i;
  int len;

  spin_lock(&s->lock);
  if (!pic_sampler_fresh_locked(s, now_ns)) {
    spin_unlock(&s->lock);
    return -ENODATA;
  }
  for (i = 0; i < PIC_SAMPLER_CHANNELS; i++) {
    pic_stats_summarize(&s->stats, i, &sum[i]);
  }
  hz = s->hz;
  age_ns = now_ns - s->read_ns;
  reads = s->reads;
  errors = s->errors;
  spin_unlock(&s->lock);

  len = sysfs_emit(buf, "hz %u age_ms %lld reads %llu errors %llu\n", hz,
                   (long long)div_s64(age_ns, NSEC_PER_MSEC), reads, errors);
  for (i = 0; i < PIC_SAMPLER_CHANNELS; i++) {
    u32 short_frac, long_frac;
    u64 short_int = div_u64_rem(sum[i].mean_short_milli, 1000, &short_frac);
    u64 long_int = div_u64_rem(sum[i].mean_long_milli, 1000, &long_frac);

    len += sysfs_emit_at(buf, len, "%s %llu.%03u %llu.%03u %u %u %u %u\n",
                         pic_sampler_channels[i].name, short_int, short_frac, long_int, long_frac,
                         sum[i].min_short, sum[i].max_short, sum[i].n_short, sum[i].n_long);
  }
  return len;
}


ssize_t pic_sampler_snapshot_show(struct spi_device *spi, char *buf)
{
  struct pic *self = spi_get_drvdata(spi);
  struct pic_sampler *s = &self->sampler;
  pic_value regs[PIC_NUM_REGISTERS];
  s64 read_ns;
  unsigned int i;

  spin_lock(&s->lock);
  if (!pic_sampler_fresh_locked(s, ktime_get_ns())) {
    spin_unlock(&s->lock);
    return -ENODATA;
  }
  memcpy(regs, s->regs, sizeof(regs));
  read_ns = s->read_ns;
  spin_unlock(&s->lock);

  put_unaligned_le64(read_ns, buf);
  for (i = 0; i < PIC_NUM_REGISTERS; i++) {
    put_unaligned_le16(regs[i], buf + sizeof(s64) + i * sizeof(pic_value));
  }
  return PIC_SAMPLER_SNAPSHOT_LEN;
}
