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

	printk("Device ready");
}
