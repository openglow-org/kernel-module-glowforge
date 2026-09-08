// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * head_api.c
 *
 * Head userspace API handlers.
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

#include "head_private.h"
#include "device_attr.h"
#include "uapi/glowforge.h"

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

static ssize_t head_write_bit_ascii(struct device *dev, int reg, int bit, int value)
{
  struct i2c_client *client = to_i2c_client(dev);
  reg = value ? (reg | HEAD_REG_BITS_SET) : (reg | HEAD_REG_BITS_CLEAR);
  return head_write_i2c_byte(client, reg, bit);
}


static ssize_t head_read_bit_ascii(struct device *dev, int reg, int bit, char *buf)
{
  struct i2c_client *client = to_i2c_client(dev);
  int value = head_read_i2c_byte(client, reg);
  if (value < 0)
    return value;
  return scnprintf(buf, PAGE_SIZE, "%d\n", (value & bit) ? 1 : 0);
}


static ssize_t head_read_dword_ascii(struct device *dev, int reg, char *buf)
{
  struct i2c_client *client = to_i2c_client(dev);
  int value = head_read_i2c_word(client, reg);
  if (value < 0)
    return value;
  return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}


static ssize_t head_write_dword_ascii(struct device *dev, int reg, const char *buf, size_t count)
{
  struct i2c_client *client = to_i2c_client(dev);
  uint16_t new_value = 0;
  int ret;
  /* Error if the entire buffer is used; we need a null-terminator */
  if (count >= PAGE_SIZE)
    return -E2BIG;

  if (sscanf(buf, "%hu", &new_value) != 1)
    return -EINVAL;

  if (new_value > 0x03ff)
    return -EINVAL;

  ret = head_write_i2c_word(client, reg, new_value);
  return (ret >= 0) ? count : ret;
}


static ssize_t info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct i2c_client *client = to_i2c_client(dev);
  int id, v1, v2, v3, v4, reg5, reg6, serial_lsb, serial_msb;
  uint32_t serial;

  id = head_read_i2c_word(client, HEAD_REG_ID);
  if (id < 0)
    return -EIO;

  v1 = head_read_i2c_byte(client, HEAD_REG_VER_1);
  v2 = head_read_i2c_byte(client, HEAD_REG_VER_2);
  v3 = head_read_i2c_byte(client, HEAD_REG_VER_3);
  v4 = head_read_i2c_byte(client, HEAD_REG_VER_4);
  if (v1 < 0 || v2 < 0 || v3 < 0 || v4 < 0)
    return -EIO;

  serial_lsb = head_read_i2c_word(client, HEAD_REG_SER_LSB);
  serial_msb = head_read_i2c_word(client, HEAD_REG_SER_MSB);
  if (serial_lsb < 0 || serial_msb < 0)
    return -EIO;
  serial = (uint32_t)serial_lsb | ((uint32_t)serial_msb << 16);

  reg5 = head_read_i2c_byte(client, HEAD_REG_RO_GRP);
  reg6 = head_read_i2c_byte(client, HEAD_REG_RW_GRP);
  if (reg5 < 0 || reg6 < 0)
    return -EIO;

  return scnprintf(buf, PAGE_SIZE,
    "id=%04x\nserial=%u\nversion=%02x%02x%02x%02x\nreg5=%c%c%c%c%c%c%c%c\nreg6=%c%c%c%c%c%c%c%c\n",
    id, serial, v1, v2, v3, v4, BYTE_TO_BINARY(reg5), BYTE_TO_BINARY(reg6));
}
static DEVICE_ATTR(info, S_IRUSR, info_show, NULL);


#define DEFINE_HEAD_RO_BOOL_ATTR(name, reg, bit) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return head_read_bit_ascii(dev, reg, bit, buf); } \
  static DEVICE_ATTR(name, S_IRUSR, name##_show, NULL)

#define DEFINE_HEAD_RW_BOOL_ATTR(name, reg, bit) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return head_read_bit_ascii(dev, reg, bit, buf); } \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    if (count < 1) { return -EINVAL; } \
    char ch = *buf; if (ch != '0' && ch != '1') { return -EINVAL; } \
    ssize_t ret = head_write_bit_ascii(dev, reg, bit, ch == '1'); \
    return (ret < 0) ? ret : count; } \
  static DEVICE_ATTR(name, S_IRUSR | S_IWUSR, name##_show, name##_store)

#define DEFINE_HEAD_RO_DWORD_ATTR(name, reg) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return head_read_dword_ascii(dev, reg, buf); } \
  static DEVICE_ATTR(name, S_IRUSR, name##_show, NULL)

#define DEFINE_HEAD_RW_DWORD_ATTR(name, reg) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return head_read_dword_ascii(dev, reg, buf); } \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    return head_write_dword_ascii(dev, reg, buf, count); } \
  static DEVICE_ATTR(name, S_IRUSR | S_IWUSR, name##_show, name##_store)

DEFINE_HEAD_RO_BOOL_ATTR(ATTR_HALL_SENSOR, HEAD_REG_RO_GRP, HEAD_REG_RO_HALL_SENSOR);
DEFINE_HEAD_RO_BOOL_ATTR(ATTR_ACCEL_IRQ, HEAD_REG_RO_GRP, HEAD_REG_RO_ACCEL_IRQ);
DEFINE_HEAD_RO_BOOL_ATTR(ATTR_BEAM_DETECT_DIGITAL, HEAD_REG_RO_GRP, HEAD_REG_RO_BEAM_DET_DIG);

DEFINE_HEAD_RW_BOOL_ATTR(ATTR_Z_ENABLE, HEAD_REG_RW_GRP, HEAD_REG_RW_Z_ENABLE);
DEFINE_HEAD_RW_BOOL_ATTR(ATTR_Z_CURRENT, HEAD_REG_RW_GRP, HEAD_REG_RW_Z_CURRENT);
DEFINE_HEAD_RW_BOOL_ATTR(ATTR_Z_MODE, HEAD_REG_RW_GRP, HEAD_REG_RW_Z_MODE);
DEFINE_HEAD_RW_BOOL_ATTR(ATTR_PURGE_AIR, HEAD_REG_RW_GRP, HEAD_REG_RW_PURGE_AIR);

DEFINE_HEAD_RO_DWORD_ATTR(ATTR_AIR_ASSIST_TACH, HEAD_REG_AIR_ASSIST_TACH);
DEFINE_HEAD_RO_DWORD_ATTR(ATTR_PURGE_CURRENT, HEAD_REG_PURGE_AIR_CUR);
DEFINE_HEAD_RO_DWORD_ATTR(ATTR_BEAM_DETECT_ANALOG, HEAD_REG_BEAM_DET_ANA);

DEFINE_HEAD_RW_DWORD_ATTR(ATTR_AIR_ASSIST_PWM, HEAD_REG_AIR_ASSIST_PWM);
DEFINE_HEAD_RW_DWORD_ATTR(ATTR_WHITE_LED, HEAD_REG_WHITE_LED);
DEFINE_HEAD_RW_DWORD_ATTR(ATTR_UV_LED, HEAD_REG_UV_LED);
DEFINE_HEAD_RW_DWORD_ATTR(ATTR_MEASURE_LASER, HEAD_REG_MSR_LASER);

static struct attribute *head_attrs[] = {
  DEV_ATTR_PTR(ATTR_INFO),
  DEV_ATTR_PTR(ATTR_HALL_SENSOR),
  DEV_ATTR_PTR(ATTR_ACCEL_IRQ),
  DEV_ATTR_PTR(ATTR_BEAM_DETECT_DIGITAL),
  DEV_ATTR_PTR(ATTR_Z_ENABLE),
  DEV_ATTR_PTR(ATTR_Z_CURRENT),
  DEV_ATTR_PTR(ATTR_Z_MODE),
  DEV_ATTR_PTR(ATTR_PURGE_AIR),
  DEV_ATTR_PTR(ATTR_AIR_ASSIST_TACH),
  DEV_ATTR_PTR(ATTR_PURGE_CURRENT),
  DEV_ATTR_PTR(ATTR_BEAM_DETECT_ANALOG),
  DEV_ATTR_PTR(ATTR_AIR_ASSIST_PWM),
  DEV_ATTR_PTR(ATTR_WHITE_LED),
  DEV_ATTR_PTR(ATTR_UV_LED),
  DEV_ATTR_PTR(ATTR_MEASURE_LASER),
  NULL
};


const struct attribute_group head_attr_group = {
  .attrs = head_attrs
};
