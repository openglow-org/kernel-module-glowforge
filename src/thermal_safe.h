/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * thermal_safe.h
 *
 * The thermal driver's safe state and the orders it is torn down in, as
 * sequences of the driver's steps (struct thermal_safe_ops, in thermal.c).
 *
 * - The safe state (the dead man's switch): the heater's duty to zero
 *   first, so the running PWM timer holds the pin low instead of raising
 *   it on its next edge, then the heat sources' pins (heater and TEC) low.
 *   The pump and the fans are not part of it.
 * - Stopping the heater: its timer first, then its pin low, whatever the
 *   timer's last edge was. Only the timer raises the pin, so once it has
 *   stopped no write of the duty can turn the heater on.
 * - Remove: the attributes first, so no write turns the heater on between
 *   the safe state and the timer's end; then the safe state, the dead
 *   man's switch notifier, the heater stopped, the lapse work.
 * - Shutdown (halt, power-off, reboot): the safe state and the heater
 *   stopped; the device stays bound, so the attributes and the notifier
 *   stay.
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
#ifndef KERNEL_SRC_THERMAL_SAFE_H_
#define KERNEL_SRC_THERMAL_SAFE_H_

/** The driver's steps; ctx is its own. */
struct thermal_safe_ops {
  /** The heater's duty to zero through the lease write, under the
   *  heater's lock (heater_lease.h: a zero moves no lease). */
  void (*zero_heater)(void *ctx);
  /** The heat sources' pins low: the heater and the TEC. */
  void (*pins_low)(void *ctx);
  /** The heater's PWM timer stopped; returns once no callback runs. */
  void (*stop_timer)(void *ctx);
  /** The heater's pin driven low. */
  void (*heater_pin_low)(void *ctx);
  /** The sysfs attributes and the /sys/glowforge link removed. */
  void (*remove_attrs)(void *ctx);
  /** The dead man's switch notifier unregistered. */
  void (*unregister_dms)(void *ctx);
  /** The lapse's log work canceled; returns once it does not run. */
  void (*cancel_lapse_work)(void *ctx);
};

/** The safe state. */
static inline void thermal_safe_state(const struct thermal_safe_ops *o, void *ctx)
{
  o->zero_heater(ctx);
  o->pins_low(ctx);
}

/** The heater stopped. */
static inline void thermal_safe_stop_heater(const struct thermal_safe_ops *o, void *ctx)
{
  o->stop_timer(ctx);
  o->heater_pin_low(ctx);
}

/** Remove, up to the release of the pins, PWMs and interrupts. */
static inline void thermal_safe_remove(const struct thermal_safe_ops *o, void *ctx)
{
  o->remove_attrs(ctx);
  thermal_safe_state(o, ctx);
  o->unregister_dms(ctx);
  thermal_safe_stop_heater(o, ctx);
  o->cancel_lapse_work(ctx);
}

/** Halt, power-off and reboot. */
static inline void thermal_safe_shutdown(const struct thermal_safe_ops *o, void *ctx)
{
  thermal_safe_state(o, ctx);
  thermal_safe_stop_heater(o, ctx);
}

#endif
