// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * cnc_buffer.c
 *
 * Manages the queue of pulse data shared with the SDMA engine.
 *
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
#include <linux/log2.h>
#include <linux/moduleparam.h>

#include "cnc_backtrack.h"
#include "cnc_private.h"
#include "sdma_macros.h"

/**
 * Size of the contiguous memory region used as the pulse data buffer, in
 * MiB. Must be a power of two and fit within the cnc reserved-memory pool
 * (cnc-pulsebuf in the device tree). Sizing: the grblHAL live feed keeps
 * only ~200 ms (a few KB) in flight, while legacy cloud mode preloads a
 * job's ENTIRE pulse file into the ring before playback - the ring is the
 * cloud-mode job-length cap at ~1 MiB per 100 s of 10 kHz stream (32 MiB
 * = ~56 min, the same ring the factory firmware runs). Raise ring_mb (and
 * the DT pool) for longer cloud jobs.
 */
#define CNC_BUFFER_DEFAULT_MB 32

static unsigned int ring_mb = CNC_BUFFER_DEFAULT_MB;
module_param(ring_mb, uint, 0444);
MODULE_PARM_DESC(ring_mb, "pulse ring size in MiB (power of two, must fit "
                          "the cnc reserved pool; default 32)");

/**
 * Enforce a minimum gap between head and tail.
 * This allows a small amount of data to be retained after it has been
 * processed: the writer stops this far short of overwriting the play head's
 * history, so a pause always has at least this many bytes to back into,
 * whether the job was preloaded or is being fed live (cnc_backtrack.h).
 * 32 KiB is 3.2 seconds of history at the 10 kHz print tick.
 */
#define CNC_BUFFER_GAP_SIZE (32 * SZ_1K)


int cnc_buffer_init(struct cnc *self)
{
  if (!is_power_of_2(ring_mb) || ring_mb < 1 || ring_mb > 1024) {
    dev_warn(self->dev, "ring_mb=%u invalid (power of two 1..1024); using %u",
             ring_mb, CNC_BUFFER_DEFAULT_MB);
    ring_mb = CNC_BUFFER_DEFAULT_MB;
  }
  self->pulsebuf_total_bytes = 0;
  self->pulsebuf_size = ring_mb * SZ_1M;
  self->pulsebuf_virt = dma_alloc_coherent(self->dev, self->pulsebuf_size, &self->pulsebuf_phys, GFP_DMA|GFP_KERNEL);
  if (!self->pulsebuf_virt) {
    dev_err(self->dev, "unable to allocate coherent buffer of size %u", self->pulsebuf_size);
    return -ENOMEM;
  }
  return kfifo_init(&self->pulsebuf_fifo, self->pulsebuf_virt, self->pulsebuf_size);
}


void cnc_buffer_destroy(struct cnc *self)
{
  if (self->pulsebuf_virt && self->pulsebuf_phys && self->pulsebuf_size) {
    dma_free_coherent(self->dev, self->pulsebuf_size, self->pulsebuf_virt, self->pulsebuf_phys);
  }
  /* idempotent: probe-error unwind and remove may both get here */
  self->pulsebuf_virt = NULL;
  self->pulsebuf_phys = 0;
  self->pulsebuf_size = 0;
}


/**
 * Synchronizes the current head value from the SDMA engine to the kfifo.
 * Should be called before any operation that requires querying the amount of
 * free space in the fifo.
 * Performed by cnc_buffer_is_empty() and cnc_buffer_add_user_data().
 * May sleep.
 * Returns nonzero if there was an error reading the SDMA channel context.
 */
static int cnc_buffer_sync_head(struct cnc *self)
{
  int ret = sdma_get_reg(self->sdmac, &self->pulsebuf_fifo.kfifo.out, scratch4);
  if (ret) {
    dev_err(self->dev, "context fetch failed: %d", ret);
  }
  return ret;
}


ssize_t cnc_buffer_add_user_data(struct cnc *self, const uint8_t __user *data, size_t count)
{
  unsigned int copied;
  int ret;

  /* One-open exclusivity does not bound the number of writers: the
   * brokered fd is inherited, so two processes can write() concurrently.
   * kfifo is single-producer; serialize the whole mutate-and-publish. */
  mutex_lock(&self->pulsebuf_lock);

  /* Reject writes while running backward: the tail publish below rewrites
   * scratch5, which a backward run repurposes as the oldest-valid-data dead
   * stop; clobbering it would let the engine replay up to the whole ring
   * of stale data backward. Checked under pulsebuf_lock, the lock the
   * backtrack start commits its scratch5 and state under, so a write
   * cannot slip between the check and the start. */
  spin_lock_bh(&self->status_lock);
  if (self->status.state == STATE_RUNNING && self->status.running_backward) {
    spin_unlock_bh(&self->status_lock);
    mutex_unlock(&self->pulsebuf_lock);
    return -EBUSY;
  }
  spin_unlock_bh(&self->status_lock);

  /* read current head value from SDMA */
  ret = cnc_buffer_sync_head(self);
  if (ret) { goto out; }

  /* bail if there's not enough room */
  if (kfifo_avail(&self->pulsebuf_fifo) < count+CNC_BUFFER_GAP_SIZE) {
    ret = -ENOMEM;
    goto out;
  }

  /* copy userspace data into fifo; */
  /* entire buffer must fit, don't allow partial copies */
  ret = kfifo_from_user(&self->pulsebuf_fifo, data, count, &copied);
  if (ret || copied != count) {
    /* Roll back partially-committed bytes so kfifo.in never diverges from
     * what the SDMA engine will be told about (scratch5 is only published
     * on full success). */
    self->pulsebuf_fifo.kfifo.in -= copied;
    if (!ret) { ret = -ENOMEM; }
    goto out;
  }

  self->pulsebuf_total_bytes += count;
  /* inform SDMA of the new tail index; just change one register */
  ret = sdma_set_reg(self->sdmac, &self->pulsebuf_fifo.kfifo.in, scratch5);
  if (ret) {
    dev_err(self->dev, "context load failed: %d", ret);
    goto out;
  }
  mutex_unlock(&self->pulsebuf_lock);
  return count;

out:
  mutex_unlock(&self->pulsebuf_lock);
  return ret;
}


/* Only clears the necessary registers in the SDMA context, */
/* leaving everything else (its working registers, program counter, etc) */
/* alone. You should be able to call this while the SDMA engine is running. */
int cnc_buffer_clear(struct cnc *self, unsigned int flags)
{
  bool clear_data = false;
  uint32_t regs_to_clear = 0;
  int first_reg, last_reg;
  struct sdma_context_data cleared_context = {{0}};

  /* Determine which registers we need to clear. */
  /* (note: we can only clear a contiguous region) */
  if (flags & CNC_BUFFER_CLEAR_DATA) {
    clear_data = true;
    regs_to_clear |= (1 << sdma_reg_number(scratch3))  /* byte count */
                  |  (1 << sdma_reg_number(scratch4))  /* head */
                  |  (1 << sdma_reg_number(scratch5)); /* tail */
  }
  if (flags & CNC_BUFFER_CLEAR_POSITION) {
    regs_to_clear |= (1 << sdma_reg_number(scratch0))  /* X */
                  |  (1 << sdma_reg_number(scratch1))  /* Y */
                  |  (1 << sdma_reg_number(scratch2)); /* Z */
  }
  if (regs_to_clear == 0) { return -EINVAL; }

  /* Find the range of registers to clear */
  /* (basically, take the union of the desired ranges) */
  /* note: ffs()/fls() return values are 1-indexed */
  first_reg = ffs(regs_to_clear)-1;
  last_reg = fls(regs_to_clear)-1;

  if (clear_data) {
    kfifo_reset(&self->pulsebuf_fifo);
    /* script needs the new head/tail indexes (they might not be 0) */
    cleared_context.scratch4 = self->pulsebuf_fifo.kfifo.out;
    cleared_context.scratch5 = self->pulsebuf_fifo.kfifo.in;
    self->pulsebuf_total_bytes = 0;
  }

  /* convert register numbers (i.e. word offsets) to byte offsets */
  return sdma_load_partial_context(self->sdmac,
    (struct sdma_context_data *)(((uint32_t *)(&cleared_context))+first_reg), /* source byte pointer */
    first_reg*sizeof(uint32_t), /* destination byte offset */
    (last_reg-first_reg+1)*sizeof(uint32_t)); /* byte count */
}

/* may sleep */
int cnc_buffer_is_empty(struct cnc *self)
{
  /* A failed head fetch must not let the run-start "no data" gate
   * decide on a stale index: report empty, which refuses the run. */
  if (cnc_buffer_sync_head(self)) {
    return 1;
  }
  return kfifo_is_empty(&self->pulsebuf_fifo);
}

/* may sleep */
size_t cnc_buffer_get_free_space(struct cnc *self)
{
  size_t avail;
  cnc_buffer_sync_head(self);
  avail = kfifo_avail(&self->pulsebuf_fifo);
  /* subtract the gap size */
  return (avail > CNC_BUFFER_GAP_SIZE) ? avail-CNC_BUFFER_GAP_SIZE : 0;
}

/* may sleep */
uint32_t cnc_buffer_max_backtrack_length(struct cnc *self)
{
  /* Bytes that are played, genuine and still resident, less the tail the
   * controlled deceleration plays out past the waypoint. The gap the writer
   * leaves puts a floor under this, so a live feed keeps a pause's worth of
   * history without the feeder having to arrange one (cnc_backtrack.h). */
  uint32_t unplayed;
  /* A failed head fetch must not answer from a stale play position: report
   * no history, which refuses the backward run. */
  if (cnc_buffer_sync_head(self)) {
    return 0;
  }
  unplayed = self->pulsebuf_fifo.kfifo.in - self->pulsebuf_fifo.kfifo.out;
  return cnc_backtrack_max_steps(kfifo_size(&self->pulsebuf_fifo), unplayed,
                                 self->pulsebuf_total_bytes,
                                 cnc_backtrack_decel_steps(
                                   cnc_get_step_frequency(self),
                                   cnc_get_ramp_rate_hz_per_s(self)));
}


uint32_t cnc_buffer_backtrack_dead_stop(struct cnc *self)
{
  /* Where a backward run must come to a dead stop: the oldest byte behind
   * the write index that is still genuine job data. Under the preload model
   * that is the first byte of the job; once the ring has wrapped under a
   * live feed it is one ring back, and anything older is overwritten. */
  return self->pulsebuf_fifo.kfifo.in
       - cnc_backtrack_span(kfifo_size(&self->pulsebuf_fifo),
                            self->pulsebuf_total_bytes);
}

uint32_t cnc_buffer_fifo_bitmask(struct cnc *self)
{ return self->pulsebuf_fifo.kfifo.mask; }

uint32_t cnc_buffer_fifo_start_phys(struct cnc *self)
{ return self->pulsebuf_phys; }

uint32_t cnc_buffer_total_bytes(struct cnc *self)
{
  /* Saturate for the 32-bit position ABI instead of wrapping mid-soak. */
  return self->pulsebuf_total_bytes > 0xFFFFFFFFULL
       ? 0xFFFFFFFFu : (uint32_t)self->pulsebuf_total_bytes;
}