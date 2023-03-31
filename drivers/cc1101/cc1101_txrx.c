#include "cc1101_txrx.h"
#include "cc1101_const.h"
#include "cc1101_spi.h"

#include <zephyr/logging/log.h>

#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(cc1101_txrx);


int cc1101_tx (const struct device *dev, uint8_t *packet, int packetlen)
{
    if (packetlen == 0)
        return 0;

    // check packet length -- in case of Variable Packet length, length must be accounted
    if(packetlen > CC1101_MAX_PACKET_LENGTH-1) {
        LOG_ERR("Transmit packet too long");
        return -EINVAL;
    }

    _idle(dev);

    const struct cc1101_config *config = dev->config;
    struct cc1101_data *data = dev->data;

    cc1101_strobe(dev, CC1101_CMD_FLUSH_TX);

    int err;
    uint8_t dataSent = 0;

    if (data->variable_length) {
        err = cc1101_set_reg(dev, CC1101_REG_FIFO, packetlen);
        if (err < 0) return err;
        dataSent += 1;
    }

    // We don't handle addresses, in case we should handle this.
/*
    uint8_t filter = CC1101_REG_PKTCTRL1, 1, 0);
    if(filter != CC1101_ADR_CHK_NONE) {
        err = cc1101_set_reg(dev, CC1101_REG_FIFO, addr);
        dataSent += 1;
    }
*/

    // fill the FIFO.
    uint8_t initialWrite = Z_MIN((uint8_t)packetlen, (uint8_t)(CC1101_FIFO_SIZE - dataSent));
    err = cc1101_set_regs(dev, CC1101_REG_FIFO, packet, initialWrite);
    if (err < 0) return err;
    dataSent += initialWrite;

    err = cc1101_strobe(dev, CC1101_CMD_TX);
    if (err < 0) return err;

    while (dataSent < packetlen) {
        uint8_t bytesInFIFO;
        err = cc1101_get_reg_field(dev, CC1101_REG_TXBYTES, &bytesInFIFO, 0b01111111);

        if (bytesInFIFO < CC1101_FIFO_SIZE) {
            uint8_t bytesToWrite = Z_MIN((uint8_t)(CC1101_FIFO_SIZE - bytesInFIFO), (uint8_t)(packetlen - dataSent));
            err = cc1101_set_regs(dev, CC1101_REG_FIFO, &packet[dataSent], bytesToWrite);
            if (err < 0) return err;
            dataSent += bytesToWrite;
        } else {
            k_usleep(250);
        }
    }

    return dataSent;
}

