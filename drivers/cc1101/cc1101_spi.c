#include "cc1101_spi.h"
#include "cc1101_const.h"

#include <zephyr/logging/log.h>

#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(cc1101_spi);

/* in master mode, spi_transfer returns 0 if ok, <0 if error */
int cc1101_txrx(const struct device *dev, uint8_t reg, uint8_t *txb, uint8_t txlen, uint8_t *rxb, uint8_t rxlen)
{
    const struct cc1101_config *config = dev->config;

    int err = 0;

    uint8_t tx_buf[256] = {reg};

    if (txb != 0 && txlen > 0)
        memcpy (tx_buf +1, txb, txlen);

    struct spi_buf tx_spibuf[1];
    
    if (rxlen > txlen)
        txlen = rxlen;

    tx_spibuf[0].buf = tx_buf;
    tx_spibuf[0].len = txlen +1;

    const struct spi_buf_set tx_set = { tx_spibuf, 1 };

    if (rxlen > 0 && rxb != 0) {
        uint8_t rx_buf[256] = { 0 };
        struct spi_buf rx_spibuf[1];
        
        rx_spibuf[0].buf = rx_buf;
        rx_spibuf[0].len = rxlen + 1;

        const struct spi_buf_set rx_set = { rx_spibuf, 1 };

        err = spi_transceive_dt(&config->spi, &tx_set, &rx_set);

        if (err == 0) {
            memcpy (rxb, rx_buf +1, rxlen);
        }
    } else {
        err = spi_transceive_dt(&config->spi, &tx_set, NULL);
    }

    if (err < 0)
        LOG_ERR("spi_transceve_dt error %d", err);
    return err;
}

int cc1101_strobe(const struct device *dev, uint8_t reg)
{
    return cc1101_txrx (dev, (reg & CC1101_CMD_MASK), NULL, 0, NULL, 0);
}

int cc1101_set_reg(const struct device *dev, uint8_t reg, uint8_t data)
{
    LOG_DBG("Set Reg %02x = %02x", reg, data);
    return cc1101_txrx (dev, (reg & CC1101_CMD_MASK) | CC1101_CMD_BURST | CC1101_CMD_WRITE, &data, 1, NULL, 0);
}

int cc1101_set_regs(const struct device *dev, uint8_t reg, uint8_t *values, uint8_t len)
{
    return cc1101_txrx (dev, (reg & CC1101_CMD_MASK) | CC1101_CMD_BURST | CC1101_CMD_WRITE, values, len, NULL, 0);
}

int cc1101_get_reg(const struct device *dev, uint8_t reg, uint8_t *data)
{
    return cc1101_txrx (dev, (reg & CC1101_CMD_MASK) | CC1101_CMD_BURST | CC1101_CMD_READ, NULL, 0, data, 1);
}

int cc1101_set_reg_field (const struct device *dev, uint8_t reg, uint8_t data, uint8_t mask)
{
    uint8_t v;
    int ret;

    ret = cc1101_get_reg(dev, reg, &v);
    if (ret < 0) {
        return ret;
    }
    v = v &(~mask);
    v = v | data;
    ret = cc1101_set_reg(dev, reg, v);
    return ret;
}

int cc1101_get_reg_field (const struct device *dev, uint8_t reg, uint8_t *data, uint8_t mask)
{
    uint8_t v;
    int ret;

    ret = cc1101_get_reg(dev, reg, &v);
    if (ret < 0) {
        return ret;
    }
    *data = v & mask;
    return ret;
}

int _idle(const struct device *dev)
{
    return cc1101_strobe(dev,CC1101_CMD_IDLE);
}
