#ifndef DRIVERS_CC1101_TXRX_H
#define DRIVERS_CC1101_TXRX_H

#include "cc1101.h"

int cc1101_tx (const struct device *dev, uint8_t *packet, int packetlen);

#endif
