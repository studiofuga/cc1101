/*
 * This code is based on RadioLib by Jan Gromeš
 * https://github.com/jgromes/RadioLib
 */

#define DT_DRV_COMPAT ti_cc1101

#include "cc1101.h"
#include "cc1101_const.h"
#include "cc1101_spi.h"
#include "cc1101_config.h"

#include <zephyr/logging/log.h>

#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(cc1101);

int cc1101_find_chip(const struct device *dev)
{
    uint8_t v;

    int err = cc1101_get_reg(dev, CC1101_REG_VERSION, &v);
    if (err < 0)
        return err;
    return v;
}


static int cc1101_init(const struct device *dev)
{
    const struct cc1101_config *config = dev->config;
    struct cc1101_data *data = dev->data;
    int err;

    /* Get the SPI device */
    if (!device_is_ready(config->spi.bus)) {
        LOG_ERR("Bus device is not ready");
        return -ENODEV;
    }


    if (config->gdo0.port != 0) {
        if (!gpio_is_ready_dt(&config->gdo0)) {
            LOG_ERR("GDO0 device is not ready");
            return -ENODEV;
        }
    
        if (gpio_pin_configure_dt(&config->gdo0, GPIO_INPUT) < 0) {
            LOG_ERR("GDO0 configuration invalid");
            return -ENODEV;        
        }
    }

    if (config->gdo2.port != 0) {
        if (!gpio_is_ready_dt(&config->gdo2)) {
            LOG_ERR("GDO2 device is not ready");
            return -ENODEV;
        }

        if (gpio_pin_configure_dt(&config->gdo2, GPIO_INPUT) < 0) {
            LOG_ERR("GDO2 configuration invalid");
            return -ENODEV;        
        }
    }

    err = cc1101_strobe(dev, CC1101_CMD_RESET);

    // configure 

    _idle(dev);

    cc1101_set_reg_field(dev,CC1101_REG_MCSM0, CC1101_FS_AUTOCAL_IDLE_TO_RXTX, 0b00110000);
    cc1101_set_reg_field(dev,CC1101_REG_PKTCTRL1, CC1101_CRC_AUTOFLUSH_OFF | CC1101_APPEND_STATUS_ON | CC1101_ADR_CHK_NONE, 0b00001111);
    cc1101_set_reg_field(dev,CC1101_REG_PKTCTRL0, CC1101_WHITE_DATA_OFF | CC1101_PKT_FORMAT_NORMAL, 0b01110000);
    cc1101_set_reg_field(dev,CC1101_REG_PKTCTRL0, CC1101_CRC_ON | CC1101_LENGTH_CONFIG_VARIABLE, 0b00000111);
    cc1101_set_reg_field(dev,CC1101_REG_FIFOTHR, 0x0d, 0b00001111);

// other defaults here

    cc1101_strobe(dev, CC1101_CMD_FLUSH_RX);
    cc1101_strobe(dev, CC1101_CMD_FLUSH_TX);


    return 0;
}


#define CC1101_DEFINE(inst)                                         \
    static struct cc1101_data cc1101_data_##inst;                   \
    static struct cc1101_config cc1101_config_##inst = {              \
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 150),        \
        .gdo0 = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, int_gpios, 0), \
        .gdo2 = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, int_gpios, 1), \
    };                                                              \
    DEVICE_DT_INST_DEFINE(inst, cc1101_init, NULL,     \
                  &cc1101_data_##inst, &cc1101_config_##inst, POST_KERNEL,  \
                  99, NULL);       \


DT_INST_FOREACH_STATUS_OKAY(CC1101_DEFINE)
