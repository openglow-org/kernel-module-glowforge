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
 * refuse (-ENODATA) while the sampler is off, before its first read, and once
 * its newest read is older than the stale age (pic_sampler_policy.h: a second,
 * or two periods below 2 Hz), so a reader never takes stale values for
 * current ones. The single-register attributes are unchanged and still read
 * the PIC live.
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
#include "pic_sampler_core.h"
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>

/** Module parameter (glowforge.c): reads a second, 0 = no sampler. */
extern unsigned int pic_sampler_hz;

/** Timer slack on each tick. */
#define PIC_SAMPLER_SLACK_NS    (100 * NSEC_PER_USEC)

static int pic_sampler_thread(void *data);


static struct pic_sampler *pic_sampler_of(void *spi)
{
  struct pic *self = spi_get_drvdata(spi);

  return &self->sampler;
}


/* The kernel's side of the sampler (pic_sampler_core.h); ctx is the
 * spi_device. */
static long long pic_sampler_op_now(void *ctx)
{
  return ktime_get_ns();
}

static int pic_sampler_op_read(void *ctx, pic_value *regs)
{
  return pic_read_register_range(ctx, PIC_FIRST_REGISTER, PIC_LAST_REGISTER, (char *)regs);
}

static void pic_sampler_op_lock(void *ctx)
{
  spin_lock(&pic_sampler_of(ctx)->lock);
}

static void pic_sampler_op_unlock(void *ctx)
{
  spin_unlock(&pic_sampler_of(ctx)->lock);
}

static uint16_t *pic_sampler_op_alloc_ring(void *ctx, size_t len)
{
  return kvcalloc(len, sizeof(uint16_t), GFP_KERNEL);
}

static void pic_sampler_op_free_ring(void *ctx, uint16_t *ring)
{
  kvfree(ring);
}

static int pic_sampler_op_thread_start(void *ctx)
{
  struct task_struct *task = kthread_run(pic_sampler_thread, ctx, "pic-sampler");

  if (IS_ERR(task)) {
    return PTR_ERR(task);
  }
  pic_sampler_of(ctx)->task = task;
  return 0;
}

static void pic_sampler_op_thread_stop(void *ctx)
{
  struct pic_sampler *s = pic_sampler_of(ctx);

  if (s->task) {
    kthread_stop(s->task);
    s->task = NULL;
  }
}

static const struct pic_sampler_ops pic_sampler_ops = {
  .now = pic_sampler_op_now,
  .read = pic_sampler_op_read,
  .lock = pic_sampler_op_lock,
  .unlock = pic_sampler_op_unlock,
  .alloc_ring = pic_sampler_op_alloc_ring,
  .free_ring = pic_sampler_op_free_ring,
  .thread_start = pic_sampler_op_thread_start,
  .thread_stop = pic_sampler_op_thread_stop,
};


void pic_sampler_init(struct spi_device *spi)
{
  struct pic_sampler *s = pic_sampler_of(spi);

  spin_lock_init(&s->lock);
  s->core.running = 0;
  s->task = NULL;
  s->core.ring = NULL;
}


static int pic_sampler_thread(void *data)
{
  struct pic_sampler *s = pic_sampler_of(data);
  const long long period_ns = pic_sampler_period_ns(s->core.hz);
  ktime_t next = ktime_get();

  while (!kthread_should_stop()) {
    pic_sampler_cycle(&s->core, &pic_sampler_ops, data);

    next = ns_to_ktime(pic_sampler_next_tick(ktime_to_ns(next), ktime_get_ns(), period_ns));
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
  struct pic_sampler *s = pic_sampler_of(spi);
  unsigned int asked = READ_ONCE(pic_sampler_hz);
  /* The parameter reads back as the rate it is held to. */
  unsigned int hz = pic_sampler_rate_take(&pic_sampler_hz);
  int err;

  if (hz != asked) {
    dev_warn(&spi->dev, "pic_sampler_hz=%u is over %u; sampling at %u", asked, PIC_SAMPLER_MAX_HZ, hz);
  }
  switch (pic_sampler_start_core(&s->core, &pic_sampler_ops, spi, hz, &err)) {
  case PIC_SAMPLER_STARTED:
    dev_info(&spi->dev, "sampler at %u Hz", hz);
    break;
  case PIC_SAMPLER_OFF:
    dev_info(&spi->dev, "sampler off (pic_sampler_hz=0)");
    break;
  case PIC_SAMPLER_NO_MEMORY:
    dev_warn(&spi->dev, "sampler off: no memory for its statistics");
    break;
  case PIC_SAMPLER_NO_STATS:
    dev_warn(&spi->dev, "sampler off: its statistics did not initialize");
    break;
  case PIC_SAMPLER_NO_THREAD:
    dev_warn(&spi->dev, "sampler off: its thread did not start (%d)", err);
    break;
  }
}


void pic_sampler_stop(struct spi_device *spi)
{
  pic_sampler_stop_core(&pic_sampler_of(spi)->core, &pic_sampler_ops, spi);
}


ssize_t pic_sampler_means_show(struct spi_device *spi, char *buf)
{
  struct pic_sampler_means m;

  if (pic_sampler_means_read(&pic_sampler_of(spi)->core, &pic_sampler_ops, spi, &m) < 0) {
    return -ENODATA;
  }
  return pic_sampler_means_format(buf, PAGE_SIZE, &m);
}


ssize_t pic_sampler_snapshot_show(struct spi_device *spi, char *buf)
{
  struct pic_sampler_snapshot snap;

  if (pic_sampler_snapshot_read(&pic_sampler_of(spi)->core, &pic_sampler_ops, spi, &snap) < 0) {
    return -ENODATA;
  }
  return pic_sampler_snapshot_pack(buf, &snap);
}
