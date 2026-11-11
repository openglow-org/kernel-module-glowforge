// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * head_format_test.c
 *
 * Host-side test for the text of the head's read-only attributes
 * (src/head_format.h): a register reads as a decimal line and a failed
 * read as its errno, never as a number; the beam detector's coefficients
 * read as five four-digit hex words, and the first failed read ends them,
 * with no partial line and no read after it.
 *
 * Copyright 2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include "head_format.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;

#define CHECK(cond) do { \
  if (!(cond)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    failures++; \
  } \
} while (0)

/* The head's coefficient registers, one of which may fail; counts reads. */
struct fake_head {
  int fail_reg;
  int reads;
};

static int read_word(void *ctx, int reg)
{
  struct fake_head *h = ctx;

  h->reads++;
  if (reg == h->fail_reg) {
    return -EIO;
  }
  switch (reg) {
  case 0x22: return 0x4000;
  case 0x24: return 0xf333;
  case 0x26: return 0x0078;
  case 0x28: return 0x00c0;
  case 0x2a: return 0x0190;
  default: return -ENXIO;
  }
}

int main(void)
{
  static const int regs[HEAD_BEAM_COEFFS] = { 0x22, 0x24, 0x26, 0x28, 0x2a };
  struct fake_head h = { -1, 0 };
  char buf[64];

  /* A register reads as a decimal line; a failed read is its errno. */
  CHECK(head_format_value(buf, sizeof(buf), 1234) == 5 && strcmp(buf, "1234\n") == 0);
  memset(buf, 0, sizeof(buf));
  CHECK(head_format_value(buf, sizeof(buf), -EIO) == -EIO && buf[0] == '\0');

  /* The coefficients in register order; the first failed read ends them. */
  CHECK(head_format_coeffs(buf, sizeof(buf), regs, read_word, &h) == 25);
  CHECK(strcmp(buf, "4000 f333 0078 00c0 0190\n") == 0);
  memset(buf, 0, sizeof(buf));
  h.fail_reg = 0x26;
  h.reads = 0;
  CHECK(head_format_coeffs(buf, sizeof(buf), regs, read_word, &h) == -EIO);
  CHECK(h.reads == 3 && buf[0] == '\0');

  if (failures) {
    fprintf(stderr, "head_format_test: %d failure(s)\n", failures);
    return 1;
  }
  printf("head_format_test: all passed\n");
  return 0;
}
