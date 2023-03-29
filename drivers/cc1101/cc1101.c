/*
 * This code is based on RadioLib by Jan Gromeš
 * https://github.com/jgromes/RadioLib
 */

#define DT_DRV_COMPAT ti_cc1101

#include "cc1101.h"
#include "cc1101consts.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cc1101);

#define CC1101_GET_CONF(x, cfg) ((struct cc1101_config *)(x->config))->reg_def.cfg

struct cc1101_data {
    const struct device *dev;
    struct gpio_callback gpio_cb;
};

struct cc1101_config {
    const struct spi_dt_spec spi;
    const struct gpio_dt_spec gdo0, gdo2;
    const struct cc1101_data data;
};

static int cc1101_init(const struct device *dev)
{
    const struct cc1101_config *config = dev->config;
    struct cc1101_data *data = dev->data;
    uint16_t id;

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


    return 0;
}


#define CC1101_DEFINE(inst)                                         \
    static struct cc1101_data cc1101_data_##inst;                   \
    static struct cc1101_config cc1101_config_##inst = {              \
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 10),        \
        .gdo0 = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, int_gpios, 0), \
        .gdo2 = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, int_gpios, 1), \
    };                                                              \
    DEVICE_DT_INST_DEFINE(inst, cc1101_init, NULL,     \
                  &cc1101_data_##inst, &cc1101_config_##inst, POST_KERNEL,  \
                  99, NULL);       \


DT_INST_FOREACH_STATUS_OKAY(CC1101_DEFINE)
