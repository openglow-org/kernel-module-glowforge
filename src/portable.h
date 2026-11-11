/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * portable.h
 *
 * What the kernel-independent units take from the kernel, with the host's
 * stand-ins, so they build in the module and in the host tests alike: the
 * fixed-width types, memcpy, scnprintf (the length written, never more than
 * the buffer holds), and the 64-bit divisions that 32-bit ARM makes as
 * calls in the kernel.
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
#ifndef KERNEL_SRC_PORTABLE_H_
#define KERNEL_SRC_PORTABLE_H_

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/string.h>
#include <linux/types.h>
#else
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static inline int scnprintf(char *buf, size_t size, const char *fmt, ...)
  __attribute__((format(printf, 3, 4)));

static inline int scnprintf(char *buf, size_t size, const char *fmt, ...)
{
  va_list ap;
  int n;

  if (size == 0) {
    return 0;
  }
  va_start(ap, fmt);
  n = vsnprintf(buf, size, fmt, ap);
  va_end(ap);
  if (n < 0) {
    return 0;
  }
  return (size_t)n >= size ? (int)(size - 1) : n;
}

static inline uint64_t div_u64_rem(uint64_t dividend, uint32_t divisor, uint32_t *remainder)
{
  *remainder = (uint32_t)(dividend % divisor);
  return dividend / divisor;
}

static inline int64_t div_s64(int64_t dividend, int32_t divisor)
{
  return dividend / divisor;
}
#endif

#endif
