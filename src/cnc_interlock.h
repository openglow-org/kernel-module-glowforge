// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * cnc_interlock.h
 *
 * Remote-interlock latch drive.
 *
 * The control board's interlock latch (a CD4043B R/S latch in the laser
 * safing chain) blocks the LASER_ON gate while its Q is set. Its RESET input
 * is the interlock loop itself (closed = reset), but its SET input is the
 * SoC's INTERLOCK_LATCH_RESET line: an open loop only releases the reset, so
 * the latch never trips unless the SoC sets it. This unit owns that line and
 * holds it high whenever the loop reads open - or whenever no switch device
 * reports the loop at all - so that an open interlock blocks emission in
 * hardware. Because the latch is set-dominant, it stays blocked until the
 * line is released again *and* the loop is closed.
 *
 * The policy is deliberately free of kernel dependencies so it can be built
 * and exercised on the host; the kernel glue (an input handler on the
 * gpio-keys switch device) is compiled only under __KERNEL__.
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
#ifndef KERNEL_SRC_CNC_INTERLOCK_H_
#define KERNEL_SRC_CNC_INTERLOCK_H_

#ifdef __KERNEL__
#include <linux/input.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#else
#include <linux/input-event-codes.h>
#include <stdbool.h>
#endif

/**
 * EV_SW code the machine device tree assigns to the remote-interlock loop
 * (gpio-keys "interlock"; the switch reads active while the loop is OPEN).
 */
#define CNC_INTERLOCK_SW_CODE 5

struct cnc_interlock;

/** Writes the INTERLOCK_LATCH_RESET line: 1 = set the latch (block). */
typedef void (*cnc_interlock_drive_fn)(struct cnc_interlock *il, int level);

struct cnc_interlock {
  cnc_interlock_drive_fn drive;
  /** Number of switch devices currently reporting the loop. */
  int attached;
  /** Last reported loop state; meaningful only while attached > 0. */
  bool loop_open;
  /** Level last handed to drive(); -1 until the first call. */
  int level;
#ifdef __KERNEL__
  /** Serializes the policy state between the input path and connect/disconnect. */
  spinlock_t lock;
  struct input_handler handler;
#endif
};

/* ---- policy (host-buildable) ---- */

/** Initializes the state and drives the fail-safe level (blocked). */
void cnc_interlock_init(struct cnc_interlock *il, cnc_interlock_drive_fn drive);
/** A switch device that reports the loop is now attached; @loop_open is its current reading. */
void cnc_interlock_attach(struct cnc_interlock *il, bool loop_open);
/** A previously attached switch device went away. */
void cnc_interlock_detach(struct cnc_interlock *il);
/** Feeds one input event; anything other than the interlock switch is ignored. */
void cnc_interlock_event(struct cnc_interlock *il, unsigned int type, unsigned int code, int value);
/** The level the policy wants on the line for the current state. */
int cnc_interlock_level(const struct cnc_interlock *il);

#ifdef __KERNEL__
/* ---- kernel glue ---- */

/** Registers the input handler; call after cnc_interlock_init(). */
int cnc_interlock_register(struct cnc_interlock *il, const char *name);
/** Unregisters the input handler; the line is left in the blocked state. */
void cnc_interlock_unregister(struct cnc_interlock *il);
#endif

#endif
