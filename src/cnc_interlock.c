// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * cnc_interlock.c
 *
 * Remote-interlock latch drive: policy plus the kernel input-handler glue.
 * See cnc_interlock.h for the hardware background.
 *
 * Copyright 2020-2026 514 LLC d/b/a OpenGlow
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
#include "cnc_interlock.h"

#ifdef __KERNEL__
#include <linux/bitops.h>
#include <linux/slab.h>
#endif

/* ------------------------------------------------------------------------
 * Policy
 *
 * The line is high (latch set, emission blocked) unless a switch device is
 * attached AND its last reading says the loop is closed. Nothing here sleeps
 * or allocates: the kernel calls it from the input event path.
 * ---------------------------------------------------------------------- */

int cnc_interlock_level(const struct cnc_interlock *il)
{
  return (il->attached > 0 && !il->loop_open) ? 0 : 1;
}

static void cnc_interlock_apply(struct cnc_interlock *il)
{
  int level = cnc_interlock_level(il);
  if (level != il->level) {
    il->level = level;
    il->drive(il, level);
  }
}

void cnc_interlock_init(struct cnc_interlock *il, cnc_interlock_drive_fn drive)
{
#ifdef __KERNEL__
  spin_lock_init(&il->lock);
#endif
  il->drive = drive;
  il->attached = 0;
  il->loop_open = true;
  il->level = -1;
  cnc_interlock_apply(il);
}

void cnc_interlock_attach(struct cnc_interlock *il, bool loop_open)
{
  il->attached++;
  il->loop_open = loop_open;
  cnc_interlock_apply(il);
}

void cnc_interlock_detach(struct cnc_interlock *il)
{
  if (il->attached > 0) {
    il->attached--;
  }
  cnc_interlock_apply(il);
}

void cnc_interlock_event(struct cnc_interlock *il, unsigned int type, unsigned int code, int value)
{
  if (type != EV_SW || code != CNC_INTERLOCK_SW_CODE) {
    return;
  }
  il->loop_open = (value != 0);
  cnc_interlock_apply(il);
}


#ifdef __KERNEL__
/* ------------------------------------------------------------------------
 * Kernel glue: an input handler bound to any device that exposes the
 * interlock switch code (the gpio-keys "switches" device on the machine
 * device tree). This leaves the GPIO owned by gpio-keys - userspace keeps
 * its EV_SW view - and needs no device-tree change.
 * ---------------------------------------------------------------------- */

static void cnc_interlock_input_event(struct input_handle *handle,
  unsigned int type, unsigned int code, int value)
{
  /* Called with the device's event_lock held and interrupts disabled;
   * the drive callback is a plain memory-mapped GPIO write. */
  struct cnc_interlock *il = handle->private;
  spin_lock(&il->lock);
  cnc_interlock_event(il, type, code, value);
  spin_unlock(&il->lock);
}


static int cnc_interlock_input_connect(struct input_handler *handler,
  struct input_dev *dev, const struct input_device_id *id)
{
  struct cnc_interlock *il = container_of(handler, struct cnc_interlock, handler);
  struct input_handle *handle;
  unsigned long flags;
  bool loop_open;
  int err;

  handle = kzalloc(sizeof(*handle), GFP_KERNEL);
  if (!handle) {
    return -ENOMEM;
  }
  handle->dev = dev;
  handle->handler = handler;
  handle->name = handler->name;
  handle->private = il;

  err = input_register_handle(handle);
  if (err) {
    goto err_free;
  }
  err = input_open_device(handle);
  if (err) {
    goto err_unregister;
  }

  /* Snapshot the loop state under the device's event lock: the input core
   * suppresses no-change EV_SW events, so an already-open loop would
   * otherwise never be reported to us. Lock order matches the event path
   * (dev->event_lock, then il->lock). */
  spin_lock_irqsave(&dev->event_lock, flags);
  loop_open = test_bit(CNC_INTERLOCK_SW_CODE, dev->sw);
  spin_lock(&il->lock);
  cnc_interlock_attach(il, loop_open);
  spin_unlock(&il->lock);
  spin_unlock_irqrestore(&dev->event_lock, flags);
  return 0;

err_unregister:
  input_unregister_handle(handle);
err_free:
  kfree(handle);
  return err;
}


static void cnc_interlock_input_disconnect(struct input_handle *handle)
{
  struct cnc_interlock *il = handle->private;
  unsigned long flags;

  input_close_device(handle);
  input_unregister_handle(handle);
  spin_lock_irqsave(&il->lock, flags);
  cnc_interlock_detach(il);
  spin_unlock_irqrestore(&il->lock, flags);
  kfree(handle);
}


static const struct input_device_id cnc_interlock_ids[] = {
  {
    .flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_SWBIT,
    .evbit = { BIT_MASK(EV_SW) },
    .swbit = { [BIT_WORD(CNC_INTERLOCK_SW_CODE)] = BIT_MASK(CNC_INTERLOCK_SW_CODE) },
  },
  { },
};


int cnc_interlock_register(struct cnc_interlock *il, const char *name)
{
  il->handler.event = cnc_interlock_input_event;
  il->handler.connect = cnc_interlock_input_connect;
  il->handler.disconnect = cnc_interlock_input_disconnect;
  il->handler.name = name;
  il->handler.id_table = cnc_interlock_ids;
  return input_register_handler(&il->handler);
}


void cnc_interlock_unregister(struct cnc_interlock *il)
{
  /* Disconnects every handle, which detaches and re-blocks the line. */
  input_unregister_handler(&il->handler);
}
#endif /* __KERNEL__ */
