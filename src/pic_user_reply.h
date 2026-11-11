/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * pic_user_reply.h
 *
 * The reply the raw and hex attributes read back: the PIC's reply to the
 * last raw or hex write, kept in a buffer of its own, apart from the
 * transfer buffer that every later transfer overwrites (the sampler's, 50
 * times a second, and every single-register read). A write that fails
 * leaves nothing to read back.
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
#ifndef KERNEL_SRC_PIC_USER_REPLY_H_
#define KERNEL_SRC_PIC_USER_REPLY_H_

#include "portable.h"

/** Keeps the reply to a raw or hex write of @count bytes, whose transfer
 *  left @rx and returned @ret, in @reply: a copy of its @count bytes when
 *  the transfer succeeded (@ret 0), nothing (*@reply_size 0) when it
 *  failed. The caller holds the PIC's lock. */
static inline void pic_user_reply_take(void *reply, size_t *reply_size, const void *rx,
                                       size_t count, int ret)
{
  memcpy(reply, rx, count);
  *reply_size = ret == 0 ? count : 0;
}

/** Copies the kept reply into @out; returns its length. The caller holds
 *  the PIC's lock. */
static inline int pic_user_reply_copy(void *out, const void *reply, size_t reply_size)
{
  memcpy(out, reply, reply_size);
  return (int)reply_size;
}

#endif
