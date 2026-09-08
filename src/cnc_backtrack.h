// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * cnc_backtrack.h
 *
 * Bounds of a backward run over the pulse ring.
 *
 * A pause stops the program, walks back over ground the job already cut with
 * the laser off, and holds; the resume runs forward again and only re-enables
 * the beam once the head is back at feed and retracing cut material. Walking
 * backward is legal only over bytes that are all three of:
 *
 *   - genuinely part of this job (not ring memory from an earlier one),
 *   - already played, and
 *   - still physically in the ring.
 *
 * Both of the last two follow from the ring geometry. With an unplayed span
 * of ``u = in - out`` bytes:
 *
 *   played   = total bytes enqueued - u
 *   resident = ring size - u
 *
 * ``resident`` is the part of the ring that is not holding unplayed data: it
 * carries the most recently played bytes, oldest first in line to be reused,
 * and it is exactly the space the writer sees as free. The writer must leave
 * a gap of unwritten ring behind it (CNC_BUFFER_GAP_SIZE), so ``resident``
 * never falls below that gap. The retained gap IS the backtrack history,
 * which is what lets a live-fed job pause the same way a preloaded one does.
 *
 * The controlled deceleration runs on past the waypoint, so the distance a
 * run can be asked for is the genuine span less that tail. Asking for more
 * would dead-stop the engine at the oldest genuine byte instead of ramping
 * it down.
 *
 * The arithmetic is kept free of kernel dependencies so the host test can
 * pin it.
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
#ifndef KERNEL_SRC_CNC_BACKTRACK_H_
#define KERNEL_SRC_CNC_BACKTRACK_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/**
 * Steps consumed by a controlled deceleration from @step_freq (Hz) at
 * @ramp_rate (Hz/s): the v^2/2a tail the engine plays out after the
 * waypoint interrupt fires.
 *
 * Scaled through kHz so the product stays inside 32 bits - the driver side
 * of this runs on arm32, where a 64-bit divide is a libgcc call - and
 * rounded so the tail is overstated rather than understated.
 */
static inline uint32_t cnc_backtrack_decel_steps(uint32_t step_freq,
                                                 uint32_t ramp_rate)
{
  uint32_t f_khz = (step_freq + 999u) / 1000u;
  uint32_t r_khz = ramp_rate / 1000u;
  if (r_khz == 0u) { r_khz = 1u; }
  return (step_freq * f_khz) / (2u * r_khz);
}

/**
 * The longest backward run that can be asked for: genuine played bytes still
 * resident in the ring, less the deceleration tail.
 *
 * @ring_size:   size of the pulse ring in bytes
 * @unplayed:    bytes enqueued and not yet played (kfifo in - out)
 * @total_bytes: bytes enqueued since the last data clear
 * @decel_steps: tail from cnc_backtrack_decel_steps()
 */
static inline uint32_t cnc_backtrack_max_steps(uint32_t ring_size,
                                               uint32_t unplayed,
                                               uint64_t total_bytes,
                                               uint32_t decel_steps)
{
  uint64_t played = (total_bytes > unplayed) ? total_bytes - unplayed : 0;
  uint32_t resident = ring_size - unplayed;
  uint32_t genuine = (played < resident) ? (uint32_t)played : resident;
  return (genuine > decel_steps) ? genuine - decel_steps : 0;
}

/**
 * Bytes behind the write index that are genuine job data, i.e. how far back
 * of ``in`` the engine may consume before it must come to a dead stop. Under
 * the preload model that is the whole job; under a live feed the ring has
 * wrapped and it is the ring itself.
 */
static inline uint32_t cnc_backtrack_span(uint32_t ring_size,
                                          uint64_t total_bytes)
{
  return (total_bytes < ring_size) ? (uint32_t)total_bytes : ring_size;
}

#endif
