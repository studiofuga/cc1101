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

enum Cc1101SyncType {
    NoSync = 0, Sync15_16, Sync16_16, Sync30_32,
    CarrierSense, CarrierSense15_16, CarrierSense16_16, CarrierSense30_32
};

enum Cc1101PreambleTypes {
    Bytes2 = 0, Bytes3, Bytes4, Bytes6, Bytes8, bytes12, Bytes16, Bytes28
};

enum Cc1101Modulation {
    FSK2, GFSK, ASK_OOK, FSK4, MFSK
};

enum Cc1101SignalDirection {
    Change = 1, Falling = 1, Rising = 2
};


int cc1101_find_chip(const struct device *dev);

int cc1101_set_frequency(const struct device *dev, float freq);


#endif //DRIVERS_CC1101_H
