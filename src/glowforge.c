// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * Glowforge kernel module
 *
 * Copyright (C) 2020-2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
 * Copyright (C) 2015-2021 Glowforge, Inc. <opensource@glowforge.com>
 * Written by Matt Sarnoff with contributions from Taylor Vaughn.
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
 * "Please, don't think of yourself as a computer scientist who happens to
 * be a human being. Think of yourself as a human being who happens to have
 * the tremendous, precious, rare skills of a computer scientist--
 * with which you can help all the other human beings."
 *   Dr. Randy Pausch (1960-2008)
 *   Carnegie Mellon University School of Computer Science Commencement speech
 *   May 18, 2008
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/notifier.h>

#include "io.h"
#include "pic.h"
#include "cnc.h"
#include "head.h"
#include "thermal.h"
#include "uapi/glowforge.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Scott Wiederhold <s.e.wiederhold@gmail.com>");
MODULE_AUTHOR("Glowforge, Inc. <opensource@glowforge.com>");
MODULE_DESCRIPTION("Glowforge Driver");
MODULE_VERSION("dev");

/** Module parameters */
int cnc_enabled = 1;
int pic_enabled = 1;
int thermal_enabled = 1;
int head_enabled = 1;
/** How long the CPU stays busy before every sensor-PIC transaction, in
 *  microseconds (pic.c); 0 turns it off. Writable at runtime. */
unsigned int pic_settle_us = 500;

module_param(cnc_enabled, int, 0);
module_param(pic_enabled, int, 0);
module_param(thermal_enabled, int, 0);
module_param(head_enabled, int, 0);
module_param(pic_settle_us, uint, 0644);
MODULE_PARM_DESC(pic_settle_us, "CPU kept busy before every sensor-PIC transaction, microseconds (0 = none)");

/** kobject that provides /sys/glowforge */
struct kobject *glowforge_kobj;

BLOCKING_NOTIFIER_HEAD(dms_notifier_list);

static const struct of_device_id head_dt_ids[] = {
  { .compatible = "glowforge,head" },
  {},
};

static struct i2c_device_id head_idtable[] = {
  { "glowforge,head", 0 },
  {},
};

static struct i2c_driver head = {
  .probe =  head_probe,
  .remove = head_remove,
  .id_table = head_idtable,
  .driver = {
    .name =   "glowforge_head",
    .owner =  THIS_MODULE,
    .of_match_table = of_match_ptr(head_dt_ids),
  },
};

static const struct of_device_id pic_dt_ids[] = {
  { .compatible = "glowforge,pic" },
  {},
};

/* The SPI core matches the device-name half of the compatible ("pic") against
 * this table as well as the OF table and warns at probe when it is missing. */
static const struct spi_device_id pic_spi_ids[] = {
  { "pic", 0 },
  {},
};
MODULE_DEVICE_TABLE(spi, pic_spi_ids);


static struct spi_driver pic_driver = {
  .probe =  pic_probe,
  .remove = pic_remove,
  .id_table = pic_spi_ids,
  .driver = {
    .name =   "glowforge_pic",
    .owner =  THIS_MODULE,
    .of_match_table = of_match_ptr(pic_dt_ids),
  },
};


static struct of_device_id cnc_dt_ids[] = {
  { .compatible = "glowforge,cnc" },
  {}
};


static struct platform_driver cnc = {
  .probe  = cnc_probe,
  .remove = cnc_remove,
  .driver = {
    .name = "glowforge_cnc",
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(cnc_dt_ids)
  }
};


static struct of_device_id thermal_dt_ids[] = {
  { .compatible = "glowforge,thermal" },
  {}
};


static struct platform_driver thermal = {
  .probe  = thermal_probe,
  .remove = thermal_remove,
  .driver = {
    .name = "glowforge_thermal",
    .owner = THIS_MODULE,
    .of_match_table = of_match_ptr(thermal_dt_ids)
  }
};


static int __init glowforge_init(void)
{
  int status = 0;

  pr_info("%s: started\n", __func__);

  /* Create /sys/glowforge */
  glowforge_kobj = kobject_create_and_add(ROOT_KOBJ_NAME, NULL);
  if (!glowforge_kobj) {
    return -ENOMEM;
  }

  /* Initialize the PIC */
  status = spi_register_driver(&pic_driver);
  if (status < 0) {
    pr_err("failed to initialize PIC driver\n");
    goto failed_pic_init;
  }

  /* Initialize the head */
  status = i2c_add_driver(&head);
  if (status < 0) {
    pr_err("failed to initialize head\n");
    goto failed_head_init;
  }

  /* Initialize the thermal subsystem */
  status = platform_driver_register(&thermal);
  if (status < 0) {
    pr_err("failed to initialize thermal controller\n");
    goto failed_thermal_init;
  }

  /* Initialize the stepper driver */
  status = platform_driver_register(&cnc);
  if (status < 0) {
    pr_err("failed to initialize stepper driver\n");
    goto failed_cnc_init;
  }

  pr_info("%s: done\n", __func__);
  return 0;

failed_cnc_init:
  platform_driver_unregister(&thermal);
failed_thermal_init:
  i2c_del_driver(&head);
failed_head_init:
  spi_unregister_driver(&pic_driver);
failed_pic_init:
  kobject_put(glowforge_kobj);
  return status;
}
module_init(glowforge_init);


static void __exit glowforge_exit(void)
{
  pr_info("%s: started\n", __func__);
  platform_driver_unregister(&cnc);
  platform_driver_unregister(&thermal);
  i2c_del_driver(&head);
  spi_unregister_driver(&pic_driver);
  kobject_put(glowforge_kobj);
  pr_info("%s: done\n", __func__);
}
module_exit(glowforge_exit);


/** For autoloading */
static struct of_device_id glowforge_dt_ids[] = {
  { .compatible = "glowforge,cnc" },
  { .compatible = "glowforge,head" },
  { .compatible = "glowforge,pic" },
  { .compatible = "glowforge,thermal" },
  {}
};
MODULE_DEVICE_TABLE(of, glowforge_dt_ids);
