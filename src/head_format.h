/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * head_format.h
 *
 * The text of the head's read-only attributes. A failed read is returned
 * as its errno, never printed: these registers include the beam detector
 * and the accelerometer's flag, where a failed read must never look like
 * a reading.
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
#ifndef KERNEL_SRC_HEAD_FORMAT_H_
#define KERNEL_SRC_HEAD_FORMAT_H_

#include "portable.h"

/** The beam detector's coefficients: five words. */
#define HEAD_BEAM_COEFFS 5

/** A register's @value (a byte or a word, or a failed read's negative
 *  errno) as its attribute reads: a decimal line, or the errno. */
static inline int head_format_value(char *buf, size_t cap, int value)
{
  if (value < 0) {
    return value;
  }
  return scnprintf(buf, cap, "%d\n", value);
}

/** The beam detector's five coefficients, read in order from @regs by
 *  @read_word (a word, or a negative errno), as one line of four-digit hex
 *  words. The first failed read ends it and is returned: never a partial
 *  line, and no read after it. */
static inline int head_format_coeffs(char *buf, size_t cap, const int regs[HEAD_BEAM_COEFFS],
                                     int (*read_word)(void *ctx, int reg), void *ctx)
{
  int v[HEAD_BEAM_COEFFS];
  unsigned int i;

  for (i = 0; i < HEAD_BEAM_COEFFS; i++) {
    v[i] = read_word(ctx, regs[i]);
    if (v[i] < 0) {
      return v[i];
    }
  }
  return scnprintf(buf, cap, "%04x %04x %04x %04x %04x\n", v[0], v[1], v[2], v[3], v[4]);
}

#endif
