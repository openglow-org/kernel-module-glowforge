/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * ledtrig_smooth.h
 *
 * "smooth" LED trigger.
 *
 * Copyright 2020-2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
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

#ifndef KERNEL_SRC_LEDTRIG_SMOOTH_H_
#define KERNEL_SRC_LEDTRIG_SMOOTH_H_

/** Registers the "smooth" LED trigger. Returns 0 on success. */
int ledtrig_smooth_init(void);

/** Unregisters the "smooth" LED trigger. */
void ledtrig_smooth_remove(void);

#endif
