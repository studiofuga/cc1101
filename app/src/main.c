/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "cc1101.h"

LOG_MODULE_REGISTER(main);

void main(void)
{
	int ret;
	const struct device *cs;

	cs = DEVICE_DT_GET_ANY(ti_cc1101);
	if(!cs) {
		printk("cc1101 not in devicetree\n");
		return;		
	}

	if (!device_is_ready(cs)) {
		printk("cc1101 device is not ready\n");
		return;
	}


	uint8_t chipVer = cc1101_find_chip(cs);
    if (chipVer > 0) {
        printk("Chip Version: %02x ", chipVer);
        switch (chipVer) {
        case CC1101_VERSION_CURRENT:
        	printk("current\n");
        	break;
        case CC1101_VERSION_CLONE:
        	printk("clone\n");
        	break;
        case CC1101_VERSION_LEGACY:
        	printk("legacy\n");
        	break;
        default:
        	printk("unknown/invalid\n");
        	break;
        }
    } else {
        printk("Error reading chip version: %d", chipVer);
    }

}
