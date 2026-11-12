/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * head_private.h
 *
 * Private header for the Glowforge head driver.
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

#ifndef KERNEL_SRC_HEAD_PRIVATE_H_
#define KERNEL_SRC_HEAD_PRIVATE_H_

#include "head.h"
#include "notifiers.h"
#include "uapi/glowforge.h"

#define HEAD_MAGIC_NUMBER 0x044c

#define HEAD_REG_ID 0x00
#define HEAD_REG_VER_1 0xc9
#define HEAD_REG_VER_2 0x89
#define HEAD_REG_VER_3 0x49
#define HEAD_REG_VER_4 0x09

#define HEAD_REG_SER_LSB 0x0a
#define HEAD_REG_SER_MSB 0x0c


#define HEAD_REG_RO_GRP 0x05
#define HEAD_REG_RO_HALL_SENSOR 0x01 << 0
#define HEAD_REG_RO_ACCEL_IRQ 0x01 << 1
#define HEAD_REG_RO_BEAM_DET_DIG 0x01 << 2

/* Bit writes address a group register with one of these modifiers; the data
 * byte is then the bit mask to set or clear. */
#define HEAD_REG_BITS_SET 0x40
#define HEAD_REG_BITS_CLEAR 0x80

#define HEAD_REG_RW_GRP 0x06
#define HEAD_REG_RW_Z_ENABLE 0x01 << 0
#define HEAD_REG_RW_Z_CURRENT 0x01 << 1
#define HEAD_REG_RW_PURGE_AIR 0x01 << 2
#define HEAD_REG_RW_Z_MODE 0x01 << 3

#define HEAD_REG_AIR_ASSIST_PWM 0x10
#define HEAD_REG_WHITE_LED 0x18
#define HEAD_REG_UV_LED 0x1a
#define HEAD_REG_MSR_LASER 0x1c

#define HEAD_REG_LAMBDA_K 0x22
#define HEAD_REG_LAMBDA_T 0x24
#define HEAD_REG_THETA_R 0x26
#define HEAD_REG_THETA_T 0x28
#define HEAD_REG_E_T 0x2a

#define HEAD_REG_AIR_ASSIST_TACH 0x12
#define HEAD_REG_PURGE_AIR_CUR 0x14
#define HEAD_REG_BEAM_DET_ANA 0x16

/* The head MCU's DAC; register 6 bit 1 (z_current) sets it too. */
#define HEAD_REG_DAC 0x1e
/* The beam detector's samples in its last averaging window, and its own
 * baseline. */
#define HEAD_REG_BEAM_SAMPLES 0x2c
#define HEAD_REG_BEAM_BASELINE 0x2e


struct head_data {
        /** Device that owns this data */
        struct device *dev;
        /** Lock to ensure mutually exclusive access */
        struct mutex lock;
        /** Notifiers */
        struct notifier_block dms_notifier;
};

extern const struct attribute_group head_attr_group;

/* Return the register value, or a negative errno on a failed read. */
int head_read_i2c_byte(struct i2c_client *client, int reg);
int head_read_i2c_word(struct i2c_client *client, int reg);
/* A write the head holds (head_verify.h): read back and written again until
 * the register reads it. Return 0, or a negative errno when it never held. A
 * word register is written whole; a bit of a group register is set or
 * cleared (@set) through the group's modifier addresses. */
int head_write_word_held(struct i2c_client *client, int reg, uint16_t value);
int head_write_bits_held(struct i2c_client *client, int grp, uint8_t bits, int set);

#endif
