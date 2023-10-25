/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_cc1101

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <app/drivers/cc1101.h>

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#error"cc1101 series coder is not defined in DTS"
#endif

LOG_MODULE_REGISTER(main);

void rxcallback(const struct device *dev, struct cc1101_event *evt, void *)
{
    printk("Recv: %d ", evt->len);
    for (int i = 0; i < evt->len; ++i) {
        printk ("%02x ", evt->rx[i]);
    }
    printk("\n");
}

void main(void)
{
    printk("booting\n");
	int ret;
	const struct device *cs;

    k_msleep(1000);

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

    cc1101_add_cb(cs, rxcallback, NULL);

    printk("Registers dump:");

    int err;
    uint8_t reg, v;
    for (reg = 0; reg < 0x30; ++reg) {
    	err = cc1101_get_reg(cs, reg, &v);
    	if (err) {
    		printk ("Error: cannot read register %08x, %d\n", reg, v);
    		return;
    	}

    	if ((reg % 8) == 0){
    		k_msleep(1);
	    	printk("\n%02x: ", reg);	
	    }
	    printk ("%02x ", v);
    }

    printk("\n");

    while (1) {
        /*
   		char buffer[] = "Hello World!";
    	err = cc1101_tx(cs, buffer, strlen(buffer));
    	if (err < 0) {
    		printk("Error transmitting: %d\n", err);
    	}*/
    	k_msleep(1000);
    }
}
