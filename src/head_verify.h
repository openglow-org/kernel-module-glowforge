/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * head_verify.h
 *
 * A write the head holds. The head board takes its register writes over an
 * I2C bus it shares with the head camera's sensor, and while that camera
 * starts or stops it now and then takes a write and does not apply it: the
 * bus reports no error and the register keeps its old value. So every write
 * to a writable head register is read back and written again, a few times a
 * short pause apart, until the register reads what was written. On the
 * bench reference every writable head register (all eight) reads back
 * exactly the value written to it.
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
#ifndef KERNEL_SRC_HEAD_VERIFY_H_
#define KERNEL_SRC_HEAD_VERIFY_H_

#ifdef __KERNEL__
#include <linux/errno.h>
#else
#include <errno.h>
#endif

/** Writes before a register is given up on, and the pause between them. */
#define HEAD_WRITE_TRIES    5
#define HEAD_WRITE_PAUSE_MS 20

/** The bus as the check reaches it: write returns 0 or a negative errno,
 *  read the register's value or a negative errno, pause sleeps. The driver
 *  passes its SMBus calls (byte or word, by the register), a host test its
 *  stand-in. */
struct head_write_io {
  int (*write)(void *ctx, int reg, int value);
  int (*read)(void *ctx, int reg);
  void (*pause_ms)(void *ctx, int ms);
  void *ctx;
};

/**
 * Write @value to @wreg until the register @rreg reads @want in the bits of
 * @mask, at most HEAD_WRITE_TRIES writes HEAD_WRITE_PAUSE_MS apart. A word
 * register is written and read at one address with every bit compared; a
 * bit of a group register is written through the group's set or clear
 * address and read at the group's own, with only that bit compared.
 *
 * Returns the number of writes made when the register held, or a negative
 * errno when it never did: the last write's error, or -EIO when the writes
 * went through and the register did not take them. *@got is the last read
 * (a negative errno when it failed).
 */
static inline int head_write_held(const struct head_write_io *io, int wreg, int value,
                                  int rreg, int mask, int want, int *got)
{
  int last_err = 0;
  int i;

  *got = -EIO;
  for (i = 0; i < HEAD_WRITE_TRIES; i++) {
    int ret;

    if (i)
      io->pause_ms(io->ctx, HEAD_WRITE_PAUSE_MS);
    ret = io->write(io->ctx, wreg, value);
    last_err = ret < 0 ? ret : 0;
    *got = io->read(io->ctx, rreg);
    if (*got >= 0 && (*got & mask) == (want & mask))
      return i + 1;
  }
  return last_err ? last_err : -EIO;
}

#endif
