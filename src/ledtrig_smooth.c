// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * ledtrig_smooth.c
 *
 * LED trigger that supports smooth fading between values.
 * Compensates for nonlinear brightness using a gamma curve.
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
 *
 * Smooth transitions are achieved by simulating LED brightness with a
 * critically-damped mass-spring system.
 *
 * Four attributes are exposed:
 * - target: (range: [0, 255])
 *   The new brightness set-point. The LED fades from its current value to the
 *   target value. May be changed while the LED is already fading.
 * - speed: (range: [0, 160])
 *   The speed at which the LED seeks its target brightness.
 *   Technically, this is controlling the mass-spring system's damping constant.
 *   (The spring constant is derived from this value.)
 *   The default is 64.
 * - pulse_on:  (milliseconds)
 * - pulse_off: (milliseconds)
 *   When both values are > 0, the LED's target will alternate between minimum
 *   and maximum brightness automatically.
 *   The delay between target=255 and target=0 is specified by pulse_on.
 *   The delay between target=0 and target=255 is specified by pulse_off.
 *   Both values are internally truncated to multiples of
 *   SMOOTH_MSECS_PER_UPDATE.
 *
 * The fade itself is in ledtrig_smooth_fade.h.
 *
 */

#include "ledtrig_smooth.h"
#include "ledtrig_smooth_fade.h"

#include <linux/device.h>
#include <linux/leds.h>
#include <linux/slab.h>

static unsigned int correction_table[256] = {
  /* Gamma 2.2: correction_table[i] = 1023 * (i/256)^2.2 */
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 9, 10, 11, 12, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 29, 30, 32, 33, 35, 36, 37, 39, 40, 42, 43, 45, 47, 48, 50, 52, 54, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 82, 84, 86, 88, 91, 93, 96, 98, 100, 103, 106, 108, 111, 113, 116, 119, 122, 124, 127, 130, 133, 136, 139, 142, 145, 148, 151, 154, 157, 160, 164, 167, 170, 174, 177, 180, 184, 187, 191, 194, 198, 202, 205, 209, 213, 216, 220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 269, 273, 277, 282, 286, 290, 295, 299, 304, 309, 313, 318, 323, 327, 332, 337, 342, 347, 351, 356, 361, 366, 371, 377, 382, 387, 392, 397, 403, 408, 413, 419, 424, 430, 435, 441, 446, 452, 458, 463, 469, 475, 481, 487, 493, 498, 504, 510, 517, 523, 529, 535, 541, 547, 554, 560, 566, 573, 579, 586, 592, 599, 606, 612, 619, 626, 632, 639, 646, 653, 660, 667, 674, 681, 688, 695, 702, 710, 717, 724, 731, 739, 746, 754, 761, 769, 776, 784, 792, 799, 807, 815, 823, 830, 838, 846, 854, 862, 870, 878, 887, 895, 903, 911, 920, 928, 936, 945, 953, 962, 970, 979, 988, 996, 1005, 1014, 1023
};


struct ledtrig_smooth_data {
  struct timer_list timer;
  struct led_classdev *led_cdev;
  /* Serializes the simulation step and parameter writes: timestep runs
   * from the timer (softirq) and from four store handlers (process
   * context) against the same state. */
  spinlock_t lock;

  /* The fade (ledtrig_smooth_fade.h) */
  struct smooth_fade fade;
};



#pragma mark - Simulation

/* Must be called with data->lock held. */
static void timestep_locked(struct led_classdev *led_cdev)
{
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  int level;
  bool running = smooth_step(&data->fade, &level);

  led_set_brightness(led_cdev, correction_table[level]);

  /* Once we hit the target value, we can stop iterating. */
  if (running) {
    mod_timer(&data->timer, jiffies + msecs_to_jiffies(SMOOTH_MSECS_PER_UPDATE));
  }
}


static void timestep(struct led_classdev *led_cdev)
{
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  spin_lock_bh(&data->lock);
  timestep_locked(led_cdev);
  spin_unlock_bh(&data->lock);
}


static int level_from_raw_brightness(int raw_brightness)
{
  /* Binary search to find the index of the given 10-bit brightness value, or */
  /* the index of the next-highest value. */
  /* Can't use bsearch() because it only finds exact matches. */
  /* The following code is adapted from linux/lib/bsearch.c. */
  size_t start = 0, end = sizeof(correction_table)/sizeof(correction_table[0]);
  int result, mid = 0;
  while (start < end) {
    mid = start + (end - start) / 2;
    result = raw_brightness - correction_table[mid];
    if (result < 0) { end = mid; }
    else if (result > 0) { start = mid+1; }
    else { break; }
  }
  return mid;
}



#pragma mark - API

static ssize_t target_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  return sprintf(buf, "%d\n", smooth_target(&data->fade));
}


static ssize_t target_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  unsigned long new_target;
  ssize_t ret = kstrtoul(buf, 10, &new_target);
  if (ret) { return ret; }
  if (new_target > 255) { return -EINVAL; }
  spin_lock_bh(&data->lock);
  smooth_set_target(&data->fade, new_target);
  timestep_locked(led_cdev); /* wake up the simulation */
  spin_unlock_bh(&data->lock);
  return size;
}


static ssize_t speed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  return sprintf(buf, "%u\n", smooth_speed(&data->fade));
}


static ssize_t speed_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  unsigned long new_speed;
  ssize_t ret = kstrtoul(buf, 10, &new_speed);
  if (ret) { return ret; }
  if (new_speed == 0 || new_speed > SMOOTH_SPEED_MAX) { return -EINVAL; }
  spin_lock_bh(&data->lock);
  smooth_set_speed(&data->fade, new_speed);
  timestep_locked(led_cdev); /* wake up the simulation */
  spin_unlock_bh(&data->lock);
  return size;
}


static ssize_t pulse_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  return sprintf(buf, "%d\n", data->fade.pulse_on_count*SMOOTH_MSECS_PER_UPDATE);
}


static ssize_t pulse_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  unsigned long new_on;
  ssize_t ret = kstrtoul(buf, 10, &new_on);
  if (ret) { return ret; }
  spin_lock_bh(&data->lock);
  if (smooth_set_pulse_periods(&data->fade, new_on/SMOOTH_MSECS_PER_UPDATE, data->fade.pulse_off_count)) {
    timestep_locked(led_cdev);
  }
  spin_unlock_bh(&data->lock);
  return size;
}


static ssize_t pulse_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  return sprintf(buf, "%d\n", data->fade.pulse_off_count*SMOOTH_MSECS_PER_UPDATE);
}


static ssize_t pulse_off_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  unsigned long new_off;
  ssize_t ret = kstrtoul(buf, 10, &new_off);
  if (ret) { return ret; }
  spin_lock_bh(&data->lock);
  if (smooth_set_pulse_periods(&data->fade, data->fade.pulse_on_count, new_off/SMOOTH_MSECS_PER_UPDATE)) {
    timestep_locked(led_cdev);
  }
  spin_unlock_bh(&data->lock);
  return size;
}


static DEVICE_ATTR(target, 0644, target_show, target_store);
static DEVICE_ATTR(speed, 0644, speed_show, speed_store);
static DEVICE_ATTR(pulse_on, 0644, pulse_on_show, pulse_on_store);
static DEVICE_ATTR(pulse_off, 0644, pulse_off_show, pulse_off_store);
static struct attribute *smooth_attrs[] = {
  &dev_attr_target.attr,
  &dev_attr_speed.attr,
  &dev_attr_pulse_on.attr,
  &dev_attr_pulse_off.attr,
  NULL
};
const struct attribute_group smooth_attr_group = { .attrs = smooth_attrs };



static void smooth_timer_cb(struct timer_list *t)
{
  struct ledtrig_smooth_data *data = from_timer(data, t, timer);
  timestep(data->led_cdev);
}


#pragma mark - Activate/deactivate

static int smooth_activate(struct led_classdev *led_cdev)
{
  int ret;
  struct ledtrig_smooth_data *data = kzalloc(sizeof(struct ledtrig_smooth_data), GFP_KERNEL);
  if (!data) {
    return -ENOMEM;
  }

  /* The data and the timer exist before the attributes become visible:
   * a store that lands between the two would find no trigger data. */
  /* A reverse lookup in the correction table is required to set the current */
  /* value from the LED's current raw brightness value */
  smooth_fade_rest(&data->fade, level_from_raw_brightness(led_cdev->brightness));
  data->led_cdev = led_cdev;
  spin_lock_init(&data->lock);
  timer_setup(&data->timer, smooth_timer_cb, 0);
  led_set_trigger_data(led_cdev, data);

  ret = sysfs_create_group(&led_cdev->dev->kobj, &smooth_attr_group);
  if (ret) {
    led_set_trigger_data(led_cdev, NULL);
    kfree(data);
    return ret;
  }
  return 0;
}


static void smooth_deactivate(struct led_classdev *led_cdev)
{
  struct ledtrig_smooth_data *data = led_get_trigger_data(led_cdev);
  if (data) {
    /* Remove the attributes FIRST (this drains in-flight store
     * handlers), so no store can mod_timer() after the sync delete
     * and fire the timer on freed data. */
    sysfs_remove_group(&led_cdev->dev->kobj, &smooth_attr_group);
    timer_delete_sync(&data->timer);
    led_set_trigger_data(led_cdev, NULL);
    kfree(data);
  }
}


static struct led_trigger smooth_trigger = {
  .name       = "smooth",
  .activate   = smooth_activate,
  .deactivate = smooth_deactivate,
};


int ledtrig_smooth_init(void)
{
  return led_trigger_register(&smooth_trigger);
}


void ledtrig_smooth_remove(void)
{
  led_trigger_unregister(&smooth_trigger);
}
