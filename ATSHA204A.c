/*
 * I2C slave mode EEPROM simulator
 *
 * Copyright (C) 2014 by Wolfram Sang, Sang Engineering <wsa@sang-engineering.com>
 * Copyright (C) 2014 by Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; version 2 of the License.
 *
 * Because most IP blocks can only detect one I2C slave address anyhow, this
 * driver does not support simulating EEPROM types which take more than one
 * address. It is prepared to simulate bigger EEPROMs with an internal 16 bit
 * pointer, yet implementation is deferred until the need actually arises.
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>



MODULE_AUTHOR("xiao zhao <xiao@minieye.cc>");
MODULE_DESCRIPTION("I2C ATSHA204A");
MODULE_LICENSE("GPL v2");
