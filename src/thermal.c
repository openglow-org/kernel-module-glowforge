// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * thermal.c
 *
 * Copyright 2020-2026 514 LLC d/b/a OpenGlow
 * Written by Scott Wiederhold
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
#include "thermal_private.h"
#include "thermal.h"
#include "device_attr.h"
#include "heater_lease.h"
#include "io.h"
#include "uapi/glowforge.h"
#include <linux/interrupt.h>

/** PWM period for heater control */
#define HEATER_PWM_PERIOD_NS  1e7

/** Heater maximum duty cycle. Since heater_duty_fraction is a u16, this value is 65535. */
#define HEATER_DUTY_CYCLE_BITS  (sizeof_field(struct thermal, heater_duty_fraction)*8)
#define HEATER_DUTY_MAX         ((1 << HEATER_DUTY_CYCLE_BITS)-1)

static const ktime_t heater_pwm_period = HEATER_PWM_PERIOD_NS;
static const ktime_t ktime_zero = {0};
static void thermal_set_heater_duty_fraction(struct thermal *self, u16 duty_fraction);



extern struct kobject *glowforge_kobj;

/** Module parameters */
extern int thermal_enabled;
extern unsigned int heater_lease_ms;

#pragma mark - GPIO/PWM definitions

static const struct pin_config thermal_pin_configs[THERMAL_NUM_GPIO_PINS] = {
  [PIN_WATER_PUMP]      = {"water-pump-gpio",      GPIOF_OUT_INIT_HIGH}, /* pump should be on at start */
  [PIN_WATER_HEATER]    = {"water-heater-gpio",    GPIOF_OUT_INIT_LOW},
  [PIN_TEC]             = {"tec-gpio",             GPIOF_OUT_INIT_LOW},
  [PIN_INTAKE1_TACH]    = {"intake1-tach-gpio",    GPIOF_IN},
  [PIN_INTAKE2_TACH]    = {"intake2-tach-gpio",    GPIOF_IN},
  [PIN_EXHAUST_TACH]    = {"exhaust-tach-gpio",    GPIOF_IN},
};

static const struct pwm_channel_config thermal_pwm_configs[THERMAL_NUM_PWM_CHANNELS] = {
  [PWM_INTAKE]     = {"intake-pwm",     HZ_TO_PERIOD_NS(25000)},
  [PWM_EXHAUST]    = {"exhaust-pwm",    HZ_TO_PERIOD_NS(25000)},
};

static const pin_id tach_pin_ids[THERMAL_NUM_TACH_SIGNALS] = {
  [TACH_INTAKE1]    = PIN_INTAKE1_TACH,
  [TACH_INTAKE2]    = PIN_INTAKE2_TACH,
  [TACH_EXHAUST]    = PIN_EXHAUST_TACH,
};

/**
 * Pin changes for the safe state: heat sources de-energized.
 */
DEFINE_PIN_CHANGE_SET(thermal_safe_pin_changes,
  {PIN_WATER_HEATER, 0},
  {PIN_TEC, 0},
);



#pragma mark - Deadman switch handler

/**
 * Puts the loop in its safe state: heater and TEC de-energized.
 *
 * Deliberately not touched: the coolant pump and the exhaust/intake PWMs.
 * The control daemon's cooling engine owns those channels, coolant
 * circulation and airflow after an aborted cut are wanted rather than
 * stopped (the tube is hot and the workpiece may be smoldering), and a
 * pump stop/start cycle can airlock the loop. Runs from the dead man's
 * switch chain; thermal_remove() uses it too.
 */
static void thermal_make_safe(struct thermal *self)
{
  /* Zero the soft-PWM duty first so the running heater timer holds the
   * pin low instead of re-energizing it on its next callback. */
  thermal_set_heater_duty_fraction(self, 0);
  io_change_pins(self->gpios, THERMAL_NUM_GPIO_PINS, thermal_safe_pin_changes);
  dev_err(self->dev, "making safe");
}


static int thermal_dms_handler(struct notifier_block *nb, unsigned long action, void *data)
{
  struct thermal *self = container_of(nb, struct thermal, dms_notifier);
  thermal_make_safe(self);
  return 0;
}


#pragma mark - Sysfs API

/** Must be called with tach spinlock held */
static inline u64 get_tach_value_ns(struct tach_channel *tach)
{
  ktime_t last_edge = ktime_sub(ktime_get(), tach->last_edge);
  u64 last_edge_ns = ktime_to_ns(last_edge);
  u64 last_period_ns = ktime_to_ns(tach->last_period);

  /* If the last edge or last recorded period was too long ago, we have stalled */
  return ((last_period_ns < MAXIMUM_TACH_PERIOD_NS && last_edge_ns < MAXIMUM_TACH_PERIOD_NS)
      ? last_period_ns : 0);
}

#define DEFINE_PWM_ATTR(name, pwm) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    struct thermal *self = dev_get_drvdata(dev); \
    return scnprintf(buf, PAGE_SIZE, "%hu\n", io_pwm_get_duty_cycle(&self->pwms[pwm])); } \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    struct thermal *self = dev_get_drvdata(dev); \
    unsigned long new_value; \
    ssize_t ret = kstrtoul(buf, 10, &new_value); \
    if (ret) { return ret; } \
    if (new_value > 65535) { return -EINVAL; } \
    io_pwm_set_duty_cycle(&self->pwms[pwm], new_value); \
    return count; } \
  static DEVICE_ATTR(name, S_IRUSR|S_IWUSR, name##_show, name##_store)

#define DEFINE_GPIO_ATTR(name, pin) \
  static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
    struct thermal *self = dev_get_drvdata(dev); char ch; \
    if (count < 1) { return -EINVAL; } \
    ch = *buf; if (ch != '0' && ch != '1') { return -EINVAL; } \
    gpio_set_value(self->gpios[pin], ch == '1'); \
    return count; } \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    struct thermal *self = dev_get_drvdata(dev); \
    return scnprintf(buf, PAGE_SIZE, "%d\n", (gpio_get_value(self->gpios[pin]) != 0)); } \
  static DEVICE_ATTR(name, S_IRUSR|S_IWUSR, name##_show, name##_store)

#define DEFINE_TACH_ATTR(name, tachnum) \
  static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf) { \
    u64 reported_value_ns; \
    unsigned long flags; \
    struct thermal *self = dev_get_drvdata(dev); \
    struct tach_channel *tach = &self->tachs[tachnum]; \
    spin_lock_irqsave(&tach->lock, flags); \
    reported_value_ns = get_tach_value_ns(tach); \
    spin_unlock_irqrestore(&tach->lock, flags); \
    return scnprintf(buf, PAGE_SIZE, "%llu\n", reported_value_ns); } \
  static DEVICE_ATTR(name, S_IRUSR, name##_show, NULL)

static ssize_t heater_pwm_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct thermal *self = dev_get_drvdata(dev);
  return scnprintf(buf, PAGE_SIZE, "%hu\n", self->heater_duty_fraction);
}

/* A write above zero holds for the lease; the writer writes it again
 * within the lease to keep the heater on (heater_lease.h). */
static ssize_t heater_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
  struct thermal *self = dev_get_drvdata(dev);
  unsigned long new_value;
  ssize_t ret = kstrtoul(buf, 10, &new_value);
  if (ret) { return ret; }
  if (new_value > HEATER_DUTY_MAX) { return -EINVAL; }
  thermal_set_heater_duty_fraction(self, new_value);
  return count;
}

/* Leases that ran out with the heater on since the module loaded. */
static ssize_t heater_lapses_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct thermal *self = dev_get_drvdata(dev);
  unsigned long flags;
  u32 lapses;
  spin_lock_irqsave(&self->heater_lock, flags);
  lapses = self->heater_lapses;
  spin_unlock_irqrestore(&self->heater_lock, flags);
  return scnprintf(buf, PAGE_SIZE, "%u\n", lapses);
}

DEFINE_PWM_ATTR(ATTR_INTAKE_PWM,     PWM_INTAKE);
DEFINE_PWM_ATTR(ATTR_EXHAUST_PWM,    PWM_EXHAUST);
static DEVICE_ATTR(heater_pwm, S_IRUSR|S_IWUSR, heater_pwm_show, heater_pwm_store);
static DEVICE_ATTR(heater_lapses, S_IRUSR, heater_lapses_show, NULL);
DEFINE_GPIO_ATTR(ATTR_WATER_PUMP_ON, PIN_WATER_PUMP);
DEFINE_GPIO_ATTR(ATTR_TEC_ON,        PIN_TEC);
DEFINE_TACH_ATTR(ATTR_INTAKE1_TACH,  TACH_INTAKE1);
DEFINE_TACH_ATTR(ATTR_INTAKE2_TACH,  TACH_INTAKE2);
DEFINE_TACH_ATTR(ATTR_EXHAUST_TACH,  TACH_EXHAUST);

static struct attribute *thermal_attrs[] = {
  DEV_ATTR_PTR(ATTR_INTAKE_PWM),
  DEV_ATTR_PTR(ATTR_EXHAUST_PWM),
  DEV_ATTR_PTR(ATTR_HEATER_PWM),
  DEV_ATTR_PTR(ATTR_HEATER_LAPSES),
  DEV_ATTR_PTR(ATTR_WATER_PUMP_ON),
  DEV_ATTR_PTR(ATTR_TEC_ON),
  DEV_ATTR_PTR(ATTR_INTAKE1_TACH),
  DEV_ATTR_PTR(ATTR_INTAKE2_TACH),
  DEV_ATTR_PTR(ATTR_EXHAUST_TACH),
  NULL
};

const struct attribute_group thermal_attr_group = {
  .attrs = thermal_attrs
};



#pragma mark - Interrupt handlers

/**
 * IRQ handler called on every rising edge of fan tach signals.
 */
static irqreturn_t tach_irq_handler(int irq, void *dev_id)
{
  struct tach_channel *tach = (struct tach_channel *)dev_id;
  ktime_t now = ktime_get();

  /* The reader (tach attribute show, irqsave) and this handler share
   * two 64-bit fields; unlocked writes tear on a 32-bit machine. */
  spin_lock(&tach->lock);
  tach->last_period = ktime_sub(now, tach->last_edge);
  tach->last_edge = now;
  spin_unlock(&tach->lock);

  return IRQ_HANDLED;
}



#pragma mark - Probe/remove

/**
 * Teardown tachs, unregistering IRQs. Idempotent: frees only the IRQs
 * that were actually requested and marks them freed.
 */
static void thermal_teardown_tachs(struct thermal *self)
{
  int tach_num;
  for (tach_num = 0; tach_num < THERMAL_NUM_TACH_SIGNALS; tach_num++) {
    if (self->tachs[tach_num].irq_num >= 0) {
      struct tach_channel *tach = &self->tachs[tach_num];
      disable_irq(tach->irq_num);
      free_irq(tach->irq_num, (void *)tach);
      tach->irq_num = -1;
    }
  }
}

/**
 * Initialize tach structures, setting interrupt handlers. Cleans up
 * after itself on failure: no live handler is ever left registered
 * against the (devm-freed on probe failure) driver data, and irq_num
 * only records IRQs that were actually requested.
 */
static int thermal_init_tachs(struct thermal *self)
{
  int tach_num;
  /* init tachs struct to safe values */
  for (tach_num = 0; tach_num < THERMAL_NUM_TACH_SIGNALS; tach_num++) {
    self->tachs[tach_num].irq_num = -1;
    spin_lock_init(&self->tachs[tach_num].lock);
  }
  /* allocate interrupts */
  for (tach_num = 0; tach_num < THERMAL_NUM_TACH_SIGNALS; tach_num++) {
    int pin_id = tach_pin_ids[tach_num];
    int gpio = self->gpios[pin_id];
    int irq_num = gpio_to_irq(gpio);
    struct tach_channel *tach = &self->tachs[tach_num];
    int ret;
    const struct pin_config *pin = &thermal_pin_configs[pin_id];

    tach->last_period = ktime_set(0,0);

    if (unlikely(irq_num < 0)) {
      pr_err("no IRQ for GPIO pin \"%s\": %d\n", pin->name, irq_num);
      thermal_teardown_tachs(self);
      return -ENOENT;
    }

    ret = request_irq(irq_num, tach_irq_handler,
        IRQF_TRIGGER_RISING, pin->name, (void*)tach);
    if (unlikely(ret < 0)) {
      pr_err("request_irq(%d) failed: %d\n", irq_num, ret);
      thermal_teardown_tachs(self);
      return -ENOENT;
    }
    /* Recorded only after the request succeeded, so teardown never
     * frees a never-requested IRQ. */
    tach->irq_num = irq_num;
  }

  return 0;
}



#pragma mark - Heater PWM

static ktime_t duty_to_ktime(u16 duty_fraction)
{
  if (duty_fraction == 0 || duty_fraction == HEATER_DUTY_MAX) {
    /* Return maximum period for 0% and 100% duty cycles. */
    return heater_pwm_period;
  } else {
    /* Otherwise, return the appropriate fraction of the maximum period. */
    return ns_to_ktime((ktime_to_ns(heater_pwm_period)*duty_fraction) >> HEATER_DUTY_CYCLE_BITS);
  }
}


/* Names a lapsed lease, out of the timer's hard IRQ. */
static void heater_lapse_work_fn(struct work_struct *work)
{
  struct thermal *self = container_of(work, struct thermal, heater_lapse_work);
  unsigned long flags;
  u32 lapses;
  spin_lock_irqsave(&self->heater_lock, flags);
  lapses = self->heater_lapses;
  spin_unlock_irqrestore(&self->heater_lock, flags);
  dev_warn(self->dev, "heater: no write within its %u ms lease; heater off (lapse %u)",
           self->heater_lease_ms, lapses);
}


/* Called from hard interrupt context. Every edge checks the lease, so a
 * heater nobody writes again goes off within one PWM period of its end. */
static enum hrtimer_restart heater_pwm_timer_cb(struct hrtimer *timer)
{
  struct thermal *self = container_of(timer, struct thermal, heater_pwm_timer);
  u16 duty_fraction;
  int gpio = self->gpios[PIN_WATER_HEATER];
  int new_gpio_level = 0;
  bool lapsed;

  spin_lock(&self->heater_lock);
  lapsed = heater_lease_tick(&self->heater_duty_fraction, self->heater_until_ns,
                             ktime_get_ns(), &self->heater_lapses);
  duty_fraction = self->heater_duty_fraction;
  spin_unlock(&self->heater_lock);
  if (lapsed) {
    schedule_work(&self->heater_lapse_work);
  }

  if (duty_fraction == 0 || duty_fraction == HEATER_DUTY_MAX) {
    /* 0% - turn heater off  */
    /* 100% - turn heater on */
    new_gpio_level = (duty_fraction != 0);
  } else {
    /* Are we setting the heater GPIO output high or low? */
    /* Read the current level and invert it. */
    new_gpio_level = !gpio_get_value(gpio);
    if (!new_gpio_level) {
      /* On the falling edge, calculate off-time instead of on-time */
      duty_fraction = HEATER_DUTY_MAX-duty_fraction;
    }
  }
  gpio_set_value(gpio, new_gpio_level);
  hrtimer_forward_now(timer, duty_to_ktime(duty_fraction));
  return HRTIMER_RESTART;
}


/* Process context only (the sysfs write, the safe state, probe). The
 * output follows on the next timer callback. A duty above zero starts a
 * new lease. */
static void thermal_set_heater_duty_fraction(struct thermal *self, u16 duty_fraction)
{
  unsigned long flags;
  spin_lock_irqsave(&self->heater_lock, flags);
  heater_lease_write(&self->heater_duty_fraction, &self->heater_until_ns,
                     duty_fraction, ktime_get_ns(), self->heater_lease_ms);
  spin_unlock_irqrestore(&self->heater_lock, flags);
}


/* Stops the heater's timer, then drives its pin low, whatever the timer's
 * last edge was. Only the timer raises the pin, so once it has stopped no
 * write of the duty can turn the heater on. Process context; follows
 * thermal_make_safe(). */
static void thermal_stop_heater(struct thermal *self)
{
  hrtimer_cancel(&self->heater_pwm_timer);
  gpio_set_value(self->gpios[PIN_WATER_HEATER], 0);
}



#pragma mark -

int thermal_probe(struct platform_device *pdev)
{
  struct thermal *self;
  int ret = 0;
  if (!thermal_enabled) { dev_info(&pdev->dev, "%s: disabled, skipping", __func__); return 0; }
  dev_info(&pdev->dev, "%s: started", __func__);

  /* Allocate driver data */
  self = devm_kzalloc(&pdev->dev, sizeof(*self), GFP_KERNEL);
  if (!self) {
    return -ENOMEM;
  }
  self->dev = &pdev->dev;
  platform_set_drvdata(pdev, self);

  spin_lock_init(&self->heater_lock);
  INIT_WORK(&self->heater_lapse_work, heater_lapse_work_fn);
  self->heater_lease_ms = heater_lease_ms_held(READ_ONCE(heater_lease_ms));
  if (self->heater_lease_ms != READ_ONCE(heater_lease_ms)) {
    dev_warn(&pdev->dev, "heater_lease_ms=%u is out of range; the lease is %u ms",
             READ_ONCE(heater_lease_ms), self->heater_lease_ms);
    /* The parameter reads back as the lease in use. */
    WRITE_ONCE(heater_lease_ms, self->heater_lease_ms);
  }
  hrtimer_init(&self->heater_pwm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  self->heater_pwm_timer.function = heater_pwm_timer_cb;

  /* Set up GPIOs */
  ret = io_init_gpios(pdev->dev.of_node, thermal_pin_configs, self->gpios, THERMAL_NUM_GPIO_PINS);
  if (ret) {
    goto failed_io_init;
  }

  /* Set up PWMs */
  ret = io_init_pwms(&pdev->dev, thermal_pwm_configs, self->pwms, THERMAL_NUM_PWM_CHANNELS);
  if (ret) {
    goto failed_pwm_init;
  }

  /* Set up tach interrupts */
  ret = thermal_init_tachs(self);
  if (ret) {
    goto failed_tach_init;
  }

  /* Create sysfs attributes */
  ret = sysfs_create_group(&pdev->dev.kobj, &thermal_attr_group);
  if (ret < 0) {
    dev_err(&pdev->dev, "failed to register attribute group");
    goto failed_create_group;
  }

  /* Add a link in /sys/glowforge */
  ret = sysfs_create_link(glowforge_kobj, &pdev->dev.kobj, THERMAL_GROUP_NAME);
  if (ret) {
    goto failed_create_link;
  }

  /* Add deadman switch notifier */
  self->dms_notifier.notifier_call = thermal_dms_handler;
  dms_notifier_chain_register(&dms_notifier_list, &self->dms_notifier);

  /* Set heater off, call timer immediately */
  thermal_set_heater_duty_fraction(self, 0);
  hrtimer_start(&self->heater_pwm_timer, ktime_zero, HRTIMER_MODE_REL);

  dev_info(&pdev->dev, "%s: done", __func__);
  return 0;

failed_create_link:
  sysfs_remove_group(&pdev->dev.kobj, &thermal_attr_group);
failed_create_group:
  thermal_teardown_tachs(self);
failed_tach_init:
  io_release_pwms(self->pwms, THERMAL_NUM_PWM_CHANNELS);
failed_pwm_init:
  io_release_gpios(self->gpios, THERMAL_NUM_GPIO_PINS);
failed_io_init:
  return ret;
}


void thermal_remove(struct platform_device *pdev)
{
  struct thermal *self = platform_get_drvdata(pdev);
  if (!thermal_enabled) { return; }
  dev_info(&pdev->dev, "%s: started", __func__);
  /* The attributes go first, so no write can turn the heater on again
   * between the safe state and the timer's end; the pin is driven low once
   * the timer has stopped, whatever its last edge was. */
  sysfs_remove_link(glowforge_kobj, THERMAL_GROUP_NAME);
  sysfs_remove_group(&pdev->dev.kobj, &thermal_attr_group);
  thermal_make_safe(self);
  dms_notifier_chain_unregister(&dms_notifier_list, &self->dms_notifier);
  thermal_stop_heater(self);
  cancel_work_sync(&self->heater_lapse_work);
  io_release_pwms(self->pwms, THERMAL_NUM_PWM_CHANNELS);
  thermal_teardown_tachs(self);
  io_release_gpios(self->gpios, THERMAL_NUM_GPIO_PINS);
  dev_info(&pdev->dev, "%s: done", __func__);
  return;
}


/**
 * Halt, power-off and reboot: the heater and the TEC go off. The heater's
 * lease lives in its timer, and timers stop with the system, so the pin
 * would otherwise keep its last level; with the timer stopped here the
 * heater stays off. The same safe state as thermal_remove(), in the same
 * order; the device stays bound, so the attributes, the dead man's switch
 * notifier and the pins are left as they are. The pump and the fans are
 * not touched (thermal_make_safe()).
 */
void thermal_shutdown(struct platform_device *pdev)
{
  struct thermal *self = platform_get_drvdata(pdev);
  if (!thermal_enabled) { return; }
  dev_info(&pdev->dev, "%s: started", __func__);
  thermal_make_safe(self);
  thermal_stop_heater(self);
  dev_info(&pdev->dev, "%s: done", __func__);
}
