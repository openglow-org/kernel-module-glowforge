// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * head_verify_test.c
 *
 * Host-side test for a write the head holds (src/head_verify.h): a write is
 * read back and written again until the register reads it. The head here
 * is a stand-in that takes a write and does not apply it (the bus reports
 * no error), refuses a write with a bus error, or fails a read, a given
 * number of times; its group register keeps bits of its own beside the
 * ones written through the set and clear addresses.
 *
 * Copyright 2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "head_verify.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

#define GRP     0x06
#define SET     0x40
#define CLEAR   0x80
#define WORD    0x1c    /* the measuring laser */
#define OWN     0x70    /* the group's bits of its own */

static struct {
  int reg[256];
  int drop, refuse, read_fail;
  int writes, pauses;
} head;

static int fake_write(void *ctx, int reg, int value)
{
  (void)ctx;
  head.writes++;
  if (head.refuse > 0) {
    head.refuse--;
    return -ENXIO;
  }
  if (head.drop > 0) {
    head.drop--;
    return 0;
  }
  if (reg == (GRP | SET))
    head.reg[GRP] |= value;
  else if (reg == (GRP | CLEAR))
    head.reg[GRP] &= ~value;
  else
    head.reg[reg] = value;
  return 0;
}

static int fake_read(void *ctx, int reg)
{
  (void)ctx;
  if (head.read_fail > 0) {
    head.read_fail--;
    return -EIO;
  }
  return head.reg[reg];
}

static void fake_pause(void *ctx, int ms)
{
  (void)ctx;
  (void)ms;
  head.pauses++;
}

static const struct head_write_io io = { fake_write, fake_read, fake_pause, NULL };

static void reset(void)
{
  memset(&head, 0, sizeof(head));
  head.reg[WORD] = 100;
  head.reg[GRP] = OWN | 0x02;
}

static int word(int value, int *got)
{
  return head_write_held(&io, WORD, value, WORD, 0xffff, value, got);
}

static int bits(int b, int set, int *got)
{
  return head_write_held(&io, GRP | (set ? SET : CLEAR), b, GRP, b, set ? b : 0, got);
}

int main(void)
{
  int got, ret;

  /* A write the head applies holds at the first read. */
  reset();
  ret = word(0, &got);
  CHECK(ret == 1 && head.reg[WORD] == 0 && head.pauses == 0 && got == 0);

  /* Taken and not applied: the read shows the old value, the next write lands. */
  reset();
  head.drop = 1;
  ret = word(0, &got);
  CHECK(ret == 2 && head.reg[WORD] == 0 && head.pauses == 1);

  /* Refused with a bus error, twice, then applied. */
  reset();
  head.refuse = 2;
  ret = word(0, &got);
  CHECK(ret == 3 && head.reg[WORD] == 0);

  /* A read that fails is no proof: the register is written again. */
  reset();
  head.read_fail = 1;
  ret = word(0, &got);
  CHECK(ret == 2 && head.reg[WORD] == 0);

  /* Never applied: -EIO after HEAD_WRITE_TRIES writes, a pause between each,
   * and the register still reads its old value. */
  reset();
  head.drop = 100;
  ret = word(0, &got);
  CHECK(ret == -EIO && head.writes == HEAD_WRITE_TRIES && head.pauses == HEAD_WRITE_TRIES - 1);
  CHECK(got == 100 && head.reg[WORD] == 100);

  /* Every write refused: the bus's own error comes back. */
  reset();
  head.refuse = 100;
  ret = word(0, &got);
  CHECK(ret == -ENXIO && head.writes == HEAD_WRITE_TRIES);

  /* A bit is judged on itself: the group's own bits do not fail a set, and
   * a dropped clear is written again, the other bits left as they were. */
  reset();
  ret = bits(0x01, 1, &got);
  CHECK(ret == 1 && head.reg[GRP] == (OWN | 0x03));
  head.drop = 1;
  ret = bits(0x02, 0, &got);
  CHECK(ret == 2 && head.reg[GRP] == (OWN | 0x01));

  if (failures) {
    fprintf(stderr, "head_verify_test: %d failure(s)\n", failures);
    return 1;
  }
  printf("head_verify_test: all passed\n");
  return 0;
}
