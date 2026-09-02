// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * head.c
 *
 * I2C driver for the Glowforge head.
 *
 * Copyright (C) 2020-2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
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
#include "uapi/glowforge.h"
#include "device_attr.h"

extern struct kobject *glowforge_kobj;

/** Module parameters */
extern int head_enabled;


/* Read helpers return the (non-negative) register value or a negative
 * errno. Truncating to the register width here would turn -EIO into a
 * plausible reading - and these registers include the beam detector and
 * the accelerometer flag, where a failed read must never look like a
 * strong positive. */
int head_read_i2c_byte(struct i2c_client *client, int reg)
{
  struct head_data *self = i2c_get_clientdata(client);
  int ret;
  mutex_lock(&self->lock);
  ret = i2c_smbus_read_byte_data(client, reg);
  mutex_unlock(&self->lock);
  return ret;
}


int head_read_i2c_word(struct i2c_client *client, int reg)
{
  struct head_data *self = i2c_get_clientdata(client);
  int ret;
  mutex_lock(&self->lock);
  ret = i2c_smbus_read_word_data(client, reg);
  mutex_unlock(&self->lock);
  return ret;
}


int head_write_i2c_byte(struct i2c_client *client, int reg, uint8_t new_value)
{
  struct head_data *self = i2c_get_clientdata(client);
  int ret;
  mutex_lock(&self->lock);
  ret = i2c_smbus_write_byte_data(client, reg, new_value);
  mutex_unlock(&self->lock);
  return ret;
}


int head_write_i2c_word(struct i2c_client *client, int reg, uint16_t new_value)
{
  struct head_data *self = i2c_get_clientdata(client);
  int ret;
  mutex_lock(&self->lock);
  ret = i2c_smbus_write_word_data(client, reg, new_value);
  mutex_unlock(&self->lock);
  return ret;
}


/**
 * Puts the head in its safe state: emitters off, lens motor de-energized.
 *
 * Runs from the dead man's switch chain, which is blocking, so the I2C writes
 * are allowed here; head_remove() uses it too.
 *
 * Deliberately not touched: air assist, purge air, and the white LED. The
 * cooling engine owns the head fans, and airflow after an aborted cut is
 * wanted rather than stopped; the white LED is the camera lamp, owned by the
 * control daemon. Turning either off here would fight its owner.
 */
static void head_make_safe(struct head_data *self)
{
  struct i2c_client *client = to_i2c_client(self->dev);
  int ret;

  dev_info(self->dev, "making safe");

  /* Both bits are active low: set = driver disabled, low current. */
  ret = head_write_i2c_byte(client, HEAD_REG_RW_GRP | HEAD_REG_BITS_SET,
    HEAD_REG_RW_Z_ENABLE | HEAD_REG_RW_Z_CURRENT);
  if (ret < 0) {
    dev_err(self->dev, "cannot de-energize the lens motor: %d", ret);
  }
  ret = head_write_i2c_word(client, HEAD_REG_MSR_LASER, 0);
  if (ret < 0) {
    dev_err(self->dev, "cannot turn off the measure laser: %d", ret);
  }
  ret = head_write_i2c_word(client, HEAD_REG_UV_LED, 0);
  if (ret < 0) {
    dev_err(self->dev, "cannot turn off the UV LED: %d", ret);
  }
}


static int head_dms_handler(struct notifier_block *nb, unsigned long action, void *data)
{
  struct head_data *self = container_of(nb, struct head_data, dms_notifier);
  head_make_safe(self);
  return 0;
}


int head_probe(struct i2c_client *client)
{
  struct head_data *self;
  int ret = 0;
  if (!head_enabled) {
    dev_info(&client->dev, "%s: disabled, skipping", __func__);
    return 0;
  }
  dev_info(&client->dev, "%s: started", __func__);

  /* Allocate driver data */
  self = devm_kzalloc(&client->dev, sizeof(*self), GFP_KERNEL);
  if (!self)
    return -ENOMEM;

  self->dev = &client->dev;
  mutex_init(&self->lock);
  i2c_set_clientdata(client, self);

  /* Check head identity */
  int head_id = i2c_smbus_read_word_data(client, HEAD_REG_ID);
  if (head_id < 0) {
    dev_err(&client->dev, "head identity read failed: %d\n", head_id);
    ret = head_id;
    goto failed_head_init;
  }
  if (head_id != HEAD_MAGIC_NUMBER) {
    dev_err(&client->dev, "head not detected (id 0x%04x)\n", head_id);
    ret = -ENODEV;
    goto failed_head_init;
  }

  /* Configure head */
  ret = i2c_smbus_write_word_data(client, HEAD_REG_LAMBDA_K, 0x07ae);
  if (ret < 0) goto failed_head_init;
  ret = i2c_smbus_write_word_data(client, HEAD_REG_LAMBDA_T, 0x1999);
  if (ret < 0) goto failed_head_init;
  ret = i2c_smbus_write_word_data(client, HEAD_REG_THETA_R, 0x0020);
  if (ret < 0) goto failed_head_init;
  ret = i2c_smbus_write_word_data(client, HEAD_REG_THETA_T, 0x0028);
  if (ret < 0) goto failed_head_init;
  ret = i2c_smbus_write_word_data(client, HEAD_REG_E_T, 0x0060);
  if (ret < 0) goto failed_head_init;

  /* Create sysfs attributes */
  ret = sysfs_create_group(&client->dev.kobj, &head_attr_group);
  if (ret < 0) {
    dev_err(&client->dev, "failed to register attribute group");
    goto failed_create_group;
  }

  /* Add a link in /sys/glowforge */
  ret = sysfs_create_link(glowforge_kobj, &client->dev.kobj, HEAD_GROUP_NAME);
  if (ret) {
    goto failed_create_link;
  }

  /* Add deadman switch notifier */
  self->dms_notifier.notifier_call = head_dms_handler;
  dms_notifier_chain_register(&dms_notifier_list, &self->dms_notifier);

  dev_info(&client->dev, "%s: done", __func__);
  return 0;

failed_create_link:
  sysfs_remove_group(&client->dev.kobj, &head_attr_group);
failed_create_group:
failed_head_init:
  dev_err(&client->dev, "failed to configure head");
  return ret;
}


void head_remove(struct i2c_client *client)
{
  struct head_data *self = i2c_get_clientdata(client);
  if (!head_enabled)
    return;
  dev_info(&client->dev, "%s: started", __func__);
  head_make_safe(self);
  dms_notifier_chain_unregister(&dms_notifier_list, &self->dms_notifier);
  sysfs_remove_link(glowforge_kobj, HEAD_GROUP_NAME);
  sysfs_remove_group(&client->dev.kobj, &head_attr_group);
  mutex_destroy(&self->lock);
  dev_info(&client->dev, "%s: done", __func__);
  return;
}
