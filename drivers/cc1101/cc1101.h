/*
 * This code is based on RadioLib by Jan Gromeš
 * https://github.com/jgromes/RadioLib
 */

#ifndef DRIVERS_CC1101_H
#define DRIVERS_CC1101_H

#include <stdint.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include "cc1101consts.h"

int cc1101_find_chip(const struct device *dev);
//int cc1101_reg_read(const struct device *dev, uint8_t reg, uint16_t *val);


#endif //DRIVERS_CC1101_H
