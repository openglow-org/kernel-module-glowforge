// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pic_reply_test.c
 *
 * Host-side test for the reply the PIC's raw and hex attributes read back
 * (src/pic_user_reply.h): the reply to the last raw or hex write, kept
 * apart from the transfer buffer that every later transfer overwrites, and
 * nothing after a write that failed.
 *
 * Copyright 2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "pic_user_reply.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

int main(void)
{
  static const unsigned char want[6] = { 0x02, 0x53, 0x4d, 0x04, 0x10, 0x00 };
  unsigned char rx[6], reply[6], out[6];
  size_t size = 0;

  /* A write that succeeded reads back its reply, even after a later
   * transfer has overwritten the transfer buffer. */
  memcpy(rx, want, sizeof(rx));
  memset(reply, 0, sizeof(reply));
  pic_user_reply_take(reply, &size, rx, sizeof(rx), 0);
  memset(rx, 0xff, sizeof(rx));
  CHECK(pic_user_reply_copy(out, reply, size) == 6 && memcmp(out, want, sizeof(want)) == 0);

  /* A write that failed leaves nothing to read back. */
  pic_user_reply_take(reply, &size, rx, sizeof(rx), -5);
  CHECK(pic_user_reply_copy(out, reply, size) == 0);

  if (failures) {
    fprintf(stderr, "pic_reply_test: %d failure(s)\n", failures);
    return 1;
  }
  printf("pic_reply_test: all passed\n");
  return 0;
}
