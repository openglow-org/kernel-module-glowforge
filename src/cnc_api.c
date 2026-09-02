// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * cnc_api.c
 *
 * Stepper driver userspace API handlers.
 *
 * Copyright (C) 2015-2021 Glowforge, Inc. <opensource@glowforge.com>
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

#include <linux/bitops.h>
#include <linux/filelock.h>
#include "cnc_private.h"
#include "device_attr.h"
#include "notifiers.h"


#pragma mark - Character device fops

/* miscdevice sets filp's private_data pointer to itself. The driver data
 * is gone once the driver is unbound while a holder keeps the file open
 * (forgectrl holds it for its lifetime): every fop checks for that before
 * touching it, and answers -ENODEV. */
#define DEV_SELF_FROM_FILP(filp) \
  struct device *dev = ((struct miscdevice *)filp->private_data)->parent; \
  struct cnc *self = dev_get_drvdata(dev); \
  if (unlikely(!self)) { return -ENODEV; }

static int pulsedev_open(struct inode *inode, struct file *filp)
{
  DEV_SELF_FROM_FILP(filp);
  if (test_and_set_bit(0, &self->pulsedev_in_use)) {
    dev_warn(dev, PULSE_DEVICE_PATH " is in use");
    return -EBUSY;
  }
  /* A fresh open must not inherit the previous holder's armed dead man's
   * switch; the new holder arms its own with flock(LOCK_EX). */
  self->deadman_switch_active = false;
  dev_info(dev, PULSE_DEVICE_PATH " opened");
  return 0;
}


static int pulsedev_close(struct inode *inode, struct file *filp)
{
  DEV_SELF_FROM_FILP(filp);
  /* If the driver is running, and the deadman switch is active, */
  /* halt the driver */
  enum cnc_state st = cnc_state(self);
  dev_info(dev, PULSE_DEVICE_PATH " closed, driver state is '%s', dms is %s",
    cnc_string_for_state(st),
    (self->deadman_switch_active) ? "active" : "inactive");
  if (self->deadman_switch_active && st == STATE_RUNNING) {
    dev_warn(dev, PULSE_DEVICE_PATH " closed while locked and driver is running! Emergency stop.");
    /* Halt, not disable: dropping the 40 V rail is exactly the fast
     * off/on bounce that can wedge the stepper drivers, and crash
     * recovery must not be left in the state most likely to need the
     * rail-off ladder. The latch relock below covers the beam. */
    cnc_halt(self);
    dms_notifier_call_chain(&dms_notifier_list, 0, NULL);
  }
  cnc_set_laser_latch(self, 1); /* lock laser on close */
  clear_bit(0, &self->pulsedev_in_use);
  return 0;
}



/** Currently, no data is returned when /dev/glowforge is read. */
static ssize_t pulsedev_read(struct file *filp, char __user *data, size_t count, loff_t *offp)
{
  return 0;
}


static ssize_t pulsedev_write(struct file *filp, const char __user *data, size_t count, loff_t *offp)
{
  DEV_SELF_FROM_FILP(filp);
  return cnc_buffer_add_user_data(self, data, count);
}


static int pulsedev_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
{
  return 0;
}


static int pulsedev_flock(struct file *filp, int cmd, struct file_lock *fl)
{
  DEV_SELF_FROM_FILP(filp);
  if (cmd != F_SETLK && cmd != F_SETLKW) {
    return -EINVAL;
  }
  /* A shared lock is meaningless on a single-writer device and must not
   * arm (or silently disarm) the dead man's switch. */
  if (fl->c.flc_type == F_RDLCK) {
    return -EINVAL;
  }
  /* Arm or disarm the dead man's switch */
  self->deadman_switch_active = (fl->c.flc_type != F_UNLCK);
  dev_info(dev, "dms is %s", (self->deadman_switch_active) ? "active" : "inactive");
  return 0;
}


/**
 * Seeking to certain offsets performs the following:
 * 0: clear pulse data, byte counters, and position counters
 * 1: clear pulse data and byte counters only
 * 2: clear position counters only
 */
static loff_t pulsedev_llseek(struct file *filp, loff_t off, int whence)
{
  DEV_SELF_FROM_FILP(filp);
  if (off < 0 || off > PULSEDEV_LSEEK_MAX_VALID_OFFSET || whence != SEEK_SET) {
    return -EINVAL;
  }
  return cnc_clear_pulse_data(self, (enum cnc_lseek_options)off);
}



#pragma mark - Sysfs attributes

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%s\n", cnc_state_string(self));
}


static ssize_t faults_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%u\n", cnc_triggered_faults(self));
}


static ssize_t step_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%u\n", cnc_get_step_frequency(self));
}


static ssize_t step_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  struct cnc *self = dev_get_drvdata(dev);
  unsigned long new_freq;
  int ret = kstrtoul(buf, 10, &new_freq);
  if (ret) { return ret; }
  ret = cnc_set_step_frequency(self, new_freq);
  return (ret == 0) ? count : ret;
}


static ssize_t ramp_rate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%u\n", cnc_get_ramp_rate_hz_per_s(self));
}


static ssize_t ramp_rate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  struct cnc *self = dev_get_drvdata(dev);
  unsigned long new_rate;
  int ret = kstrtoul(buf, 10, &new_rate);
  if (ret) { return ret; }
  ret = cnc_set_ramp_rate_hz_per_s(self, new_rate);
  return (ret == 0) ? count : ret;
}


static ssize_t ignored_faults_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%u\n", self->ignored_faults);
}


static ssize_t ignored_faults_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  struct cnc *self = dev_get_drvdata(dev);
  unsigned long new_mask;
  int ret = kstrtoul(buf, 10, &new_mask);
  if (ret) { return ret; }
  /* Three fault bits exist; anything wider would silently disable all
   * stepper-fault handling (the UAPI documents 0-7). */
  if (new_mask > 7) { return -EINVAL; }
  self->ignored_faults = new_mask;
  return count;
}


static ssize_t motor_lock_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%u\n", cnc_get_motor_lock(self));
}


static ssize_t motor_lock_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  struct cnc *self = dev_get_drvdata(dev);
  unsigned long new_value;
  int ret = kstrtoul(buf, 10, &new_value);
  if (ret) { return ret; }
  ret = cnc_set_motor_lock(self, new_value);
  return (ret == 0) ? count : ret;
}


static ssize_t position_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  struct cnc_position *pos = (struct cnc_position *)buf;
  int ret = cnc_get_position(self, pos);
  return (ret == 0) ? sizeof(*pos) : ret;
}


static ssize_t free_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%zu\n", cnc_buffer_get_free_space(self));
}


/* How far back a pause may walk the program, in steps: what the ring still
 * holds of what it has already played, less the deceleration tail. A caller
 * sizes both its backtrack and its laser-on lead from this, so a pause early
 * in a job (or on a ring that has just been refilled) shortens the retrace
 * instead of failing it. Each read performs SDMA channel-0 transactions. */
static ssize_t max_backtrack_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  uint32_t steps;
  mutex_lock(&self->pulsebuf_lock);
  steps = cnc_buffer_max_backtrack_length(self);
  mutex_unlock(&self->pulsebuf_lock);
  return scnprintf(buf, PAGE_SIZE, "%u\n", steps);
}


static ssize_t sdma_context_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return cnc_print_sdma_context(self, buf);
}


/* Streaming mode: a live feeder sets this to 1 to declare that running out
 * of pulse data mid-run is an UNDERRUN (fault-like STATE_UNDERRUN, position
 * no longer trusted) rather than normal end-of-program. Set it back to 0
 * after enqueueing the final bytes of a job so the terminal end-of-data is
 * treated as completion. */
static ssize_t streaming_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%d\n", self->status.streaming ? 1 : 0);
}


static ssize_t streaming_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  struct cnc *self = dev_get_drvdata(dev);
  char ch;
  if (count < 1) { return -EINVAL; }
  ch = *buf;
  if (ch != '0' && ch != '1') { return -EINVAL; }
  spin_lock_bh(&self->status_lock);
  self->status.streaming = (ch == '1');
  spin_unlock_bh(&self->status_lock);
  return count;
}


static ssize_t underruns_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct cnc *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%u\n", self->underrun_count);
}


void cnc_notify_state_changed(struct cnc *self)
{
  /* safe to call from atomic context */
  if (self->state_attr_node) {
    sysfs_notify_dirent(self->state_attr_node);
  }
}



#define _DEFINE_COMMAND_ATTR(name) \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    struct cnc *self = dev_get_drvdata(dev); char ch; int ret; \
    if (count < 1) { return -EINVAL; } \
    ch = *buf; if (ch != '1') { return -EINVAL; } \
    ret = cnc_##name(self); \
    return (ret == 0) ? count : ret; \
  } \
  static DEVICE_ATTR(name, S_IWUSR, NULL, name##_store)
#define DEFINE_COMMAND_ATTR(name) _DEFINE_COMMAND_ATTR(name)

#define DEFINE_MODE_ATTR(name, axis) \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    struct cnc *self = dev_get_drvdata(dev); \
    int mode, ret; if (sscanf(buf, "%d", &mode) != 1) { return -EINVAL; } \
    ret = cnc_set_microstep_mode(self, axis, mode); \
    return (ret == 0) ? count : ret; } \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    struct cnc *self = dev_get_drvdata(dev); \
    return scnprintf(buf, PAGE_SIZE, "%u\n", cnc_get_microstep_mode(self, axis)); } \
  static DEVICE_ATTR(name, S_IRUSR|S_IWUSR, name##_show, name##_store)

#define DEFINE_DECAY_ATTR(name, axis) \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    struct cnc *self = dev_get_drvdata(dev); \
    int mode, ret; if (sscanf(buf, "%d", &mode) != 1) { return -EINVAL; } \
    ret = cnc_set_decay_mode(self, axis, mode); \
    return (ret == 0) ? count : ret; } \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    struct cnc *self = dev_get_drvdata(dev); \
    return scnprintf(buf, PAGE_SIZE, "%u\n", cnc_get_decay_mode(self, axis)); } \
  static DEVICE_ATTR(name, S_IRUSR|S_IWUSR, name##_show, name##_store)

#define DEFINE_BOOL_ATTR(name, fn) \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    struct cnc *self = dev_get_drvdata(dev); char ch; int ret; \
    if (count < 1) { return -EINVAL; } \
    ch = *buf; if (ch != '0' && ch != '1') { return -EINVAL; } \
    ret = fn(self, ch == '1'); \
    return (ret == 0) ? count : ret; \
  } \
  static DEVICE_ATTR(name, S_IWUSR, NULL, name##_store)

/* Read-only attribute backed by an int-returning cnc_get_* function. */
#define _DEFINE_RO_GETTER_ATTR(name, fn) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    return scnprintf(buf, PAGE_SIZE, "%d\n", fn(dev_get_drvdata(dev))); \
  } \
  static DEVICE_ATTR(name, S_IRUSR, name##_show, NULL)
#define DEFINE_RO_GETTER_ATTR(name, fn) _DEFINE_RO_GETTER_ATTR(name, fn)

/* Negative values: accelerate backwards, then decelerate and stop */
/* Positive values: accelerate forward, reenable laser, and continue */
/* Zero: accelerate forward, continue without reenabling laser */
static ssize_t resume_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
  long long raw_value;
  int32_t value;
  int64_t magnitude;
  /* For ease of interfacing, allow signed and unsigned 2s complement values. */
  /* (i.e. treat numbers in [2147483648,4294967295] as negative) */
  struct cnc *self = dev_get_drvdata(dev);
  int ret = kstrtoll(buf, 10, &raw_value);
  if (ret) { return ret; }
  if (raw_value < -2147483648LL || raw_value > 4294967295LL) {
    return -EINVAL;
  }
  value = (int32_t)raw_value;
  /* The waypoint counter is a 28-bit field: a larger magnitude would
   * silently truncate (e.g. 268435457 -> waypoint 1, re-enabling the
   * laser after ONE step instead of 268M). */
  magnitude = (value < 0) ? -(int64_t)value : value;
  if (magnitude >= (1 << 28)) {
    return -EINVAL;
  }
  if (value < 0) {
    ret = cnc_backtrack(self, -value);
  } else {
    ret = cnc_resume(self, value);
  }
  return (ret == 0) ? count : ret;
}



DEFINE_COMMAND_ATTR(ATTR_RUN);
DEFINE_COMMAND_ATTR(ATTR_STOP);
DEFINE_COMMAND_ATTR(ATTR_HALT);
DEFINE_COMMAND_ATTR(ATTR_DISABLE);
DEFINE_COMMAND_ATTR(ATTR_ENABLE);
DEFINE_DEVICE_ATTR(ATTR_RESUME, S_IWUSR, NULL, resume_store);
DEFINE_DEVICE_ATTR(ATTR_STATE, S_IRUSR, state_show, NULL);
DEFINE_DEVICE_ATTR(ATTR_FAULTS, S_IRUSR, faults_show, NULL);
DEFINE_DEVICE_ATTR(ATTR_IGNORED_FAULTS, S_IRUSR|S_IWUSR, ignored_faults_show, ignored_faults_store);
DEFINE_DEVICE_ATTR(ATTR_STEP_FREQ, S_IRUSR|S_IWUSR, step_freq_show, step_freq_store);
DEFINE_DEVICE_ATTR(ATTR_RAMP_RATE, S_IRUSR|S_IWUSR, ramp_rate_show, ramp_rate_store);
DEFINE_DEVICE_ATTR(ATTR_POSITION, S_IRUSR, position_show, NULL);
DEFINE_DEVICE_ATTR(ATTR_FREE, S_IRUSR, free_show, NULL);
DEFINE_DEVICE_ATTR(ATTR_MAX_BACKTRACK, S_IRUSR, max_backtrack_show, NULL);
DEFINE_DEVICE_ATTR(ATTR_SDMA_CONTEXT, S_IRUSR, sdma_context_show, NULL);
DEFINE_DEVICE_ATTR(ATTR_MOTOR_LOCK, S_IRUSR|S_IWUSR, motor_lock_show, motor_lock_store);
DEFINE_DEVICE_ATTR(ATTR_STREAMING, S_IRUSR|S_IWUSR, streaming_show, streaming_store);
DEFINE_DEVICE_ATTR(ATTR_UNDERRUNS, S_IRUSR, underruns_show, NULL);
DEFINE_MODE_ATTR(ATTR_X_MODE, AXIS_X);
DEFINE_MODE_ATTR(ATTR_Y_MODE, AXIS_Y);
DEFINE_DECAY_ATTR(ATTR_X_DECAY, AXIS_X);
DEFINE_DECAY_ATTR(ATTR_Y_DECAY, AXIS_Y);
DEFINE_BOOL_ATTR(ATTR_Z_STEP, cnc_single_z_step);
DEFINE_BOOL_ATTR(ATTR_LASER_LATCH, cnc_set_laser_latch);
DEFINE_RO_GETTER_ATTR(ATTR_LASER_ENABLE, cnc_get_laser_enable);
DEFINE_RO_GETTER_ATTR(ATTR_LASER_ON, cnc_get_laser_on);
DEFINE_RO_GETTER_ATTR(ATTR_LASER_ON_SAMPLED, cnc_get_laser_on_sampled);
DEFINE_RO_GETTER_ATTR(ATTR_LASER_PGOOD, cnc_get_laser_pgood);
DEFINE_RO_GETTER_ATTR(ATTR_LASER_PGOOD_SAMPLED, cnc_get_laser_pgood_sampled);
DEFINE_RO_GETTER_ATTR(ATTR_INTERLOCK_CIRCUIT, cnc_get_interlock_circuit);
DEFINE_RO_GETTER_ATTR(ATTR_INTERLOCK_LATCH_RESET, cnc_get_interlock_latch_reset);
DEFINE_RO_GETTER_ATTR(ATTR_BUTTON_LATCH, cnc_get_button_latch);
DEFINE_RO_GETTER_ATTR(ATTR_CHARGE_PUMP_ALIVE, cnc_get_charge_pump_alive);

static struct attribute *cnc_attrs[] = {
  DEV_ATTR_PTR(ATTR_STATE),
  DEV_ATTR_PTR(ATTR_FAULTS),
  DEV_ATTR_PTR(ATTR_IGNORED_FAULTS),
  DEV_ATTR_PTR(ATTR_STEP_FREQ),
  DEV_ATTR_PTR(ATTR_RAMP_RATE),
  DEV_ATTR_PTR(ATTR_RUN),
  DEV_ATTR_PTR(ATTR_STOP),
  DEV_ATTR_PTR(ATTR_HALT),
  DEV_ATTR_PTR(ATTR_RESUME),
  DEV_ATTR_PTR(ATTR_DISABLE),
  DEV_ATTR_PTR(ATTR_ENABLE),
  DEV_ATTR_PTR(ATTR_LASER_LATCH),
  DEV_ATTR_PTR(ATTR_POSITION),
  DEV_ATTR_PTR(ATTR_FREE),
  DEV_ATTR_PTR(ATTR_MAX_BACKTRACK),
  DEV_ATTR_PTR(ATTR_STREAMING),
  DEV_ATTR_PTR(ATTR_UNDERRUNS),
  DEV_ATTR_PTR(ATTR_SDMA_CONTEXT),
  DEV_ATTR_PTR(ATTR_X_MODE),
  DEV_ATTR_PTR(ATTR_Y_MODE),
  DEV_ATTR_PTR(ATTR_X_DECAY),
  DEV_ATTR_PTR(ATTR_Y_DECAY),
  DEV_ATTR_PTR(ATTR_Z_STEP),
  DEV_ATTR_PTR(ATTR_MOTOR_LOCK),
  DEV_ATTR_PTR(ATTR_LASER_ENABLE),
  DEV_ATTR_PTR(ATTR_LASER_ON),
  DEV_ATTR_PTR(ATTR_LASER_ON_SAMPLED),
  DEV_ATTR_PTR(ATTR_LASER_PGOOD),
  DEV_ATTR_PTR(ATTR_LASER_PGOOD_SAMPLED),
  DEV_ATTR_PTR(ATTR_INTERLOCK_CIRCUIT),
  DEV_ATTR_PTR(ATTR_INTERLOCK_LATCH_RESET),
  DEV_ATTR_PTR(ATTR_BUTTON_LATCH),
  DEV_ATTR_PTR(ATTR_CHARGE_PUMP_ALIVE),
  NULL
};

const struct attribute_group cnc_attr_group = {
  .attrs = cnc_attrs
};

const struct file_operations pulsedev_fops = {
  .owner    = THIS_MODULE,
  .open     = pulsedev_open,
  .read     = pulsedev_read,
  .write    = pulsedev_write,
  .fsync    = pulsedev_fsync,
  .flock    = pulsedev_flock,
  .llseek   = pulsedev_llseek,
  .release  = pulsedev_close,
};
