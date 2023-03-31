/*
 * This code is based on RadioLib by Jan Gromeš
 * https://github.com/jgromes/RadioLib
 */

#ifndef DRIVERS_CC1101_H
#define DRIVERS_CC1101_H

#include <stdint.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include "cc1101_const.h"

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


int cc1101_get_reg(const struct device *dev, uint8_t reg, uint8_t *data);

int cc1101_find_chip(const struct device *dev);

int cc1101_set_frequency(const struct device *dev, float freq);

int cc1101_tx (const struct device *dev, uint8_t *packet, int packetlen);


struct cc1101_data {
    const struct device *dev;
    struct gpio_callback gpio_cb;

    float frequency;
    float bitrate;
    float power;

    bool variable_length;
    uint8_t fixed_packet_length;

    enum Cc1101Modulation modulation;
};

struct cc1101_config {
    const struct spi_dt_spec spi;
    const struct gpio_dt_spec gdo0, gdo2;
    const struct cc1101_data data;
};

#endif //DRIVERS_CC1101_H
