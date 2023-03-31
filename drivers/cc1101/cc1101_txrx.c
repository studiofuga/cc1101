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

void _cc1101_rx_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct cc1101_data *data = CONTAINER_OF(cb, struct cc1101_data, rx_cback);

    // if not receiving, signal.
    /*
    if (atomic_get(&data->rx) == 1) {
        k_sem_give(&data->rx_lock);
        atomic_set(&data->rx, 0);
    } else {    // if receiving, flag
        atomic_set(&data->rx, 1);
    }*/

            k_sem_give(&data->rx_lock);

}

static int get_status(const struct device *dev)
{
    return 0;
}

static int get_rx_fifo_bytes(const struct device *dev)
{
    uint8_t nb;
    int err = cc1101_get_reg_field(dev, CC1101_REG_RXBYTES, &nb, 0b01111111);
    if (!err){
        //LOG_INF("rx %d", nb);
        return nb;
    } else {
        LOG_ERR("error rx %d", err);
        return err;
    }    
}

static int get_rx_bytes(const struct device *dev, uint8_t *byte, uint8_t n) 
{
    return cc1101_set_regs (dev, CC1101_REG_FIFO, byte, n);
}

void _cc1101_rx_thread(void *arg)
{
    const struct device *dev = arg;
    struct cc1101_data *data = dev->data;
    struct net_pkt *pkt;
    uint8_t bytes_avail, pkt_len;

    uint8_t rxbuffer[256];
    uint8_t rxptr = 0;

    while (1) {
        pkt = NULL;

        k_sem_take(&data->rx_lock, K_FOREVER);

        // TODO check the status of the fifo. If error, flush it.

        bytes_avail = get_rx_fifo_bytes(dev);

        if (data->variable_length) {
            get_rx_bytes(dev, &pkt_len, 1);
            --bytes_avail;
        } else {
            pkt_len = data->fixed_packet_length;
        }

        while ((bytes_avail = get_rx_fifo_bytes(dev))> 0) {
            get_rx_bytes(dev, rxbuffer + rxptr, bytes_avail);
            rxptr += bytes_avail;
        }

        if (data->callback.callback) {
            struct cc1101_event e;
            e.rx = rxbuffer;
            e.len = rxptr;
            data->callback.callback(dev, &e, data->callback.user_data);
            rxptr = 0;
        }
        cc1101_strobe(dev, CC1101_CMD_RX);
    }
}


void cc1101_trigger_rx(const struct device *dev)
{
    cc1101_strobe(dev, CC1101_CMD_FLUSH_RX);
    cc1101_strobe(dev, CC1101_CMD_RX);
}

void cc1101_enable_intr_rx(const struct device *dev) 
{
    cc1101_set_reg(dev, CC1101_REG_IOCFG0, CC1101_GDOX_SYNC_WORD_SENT_OR_RECEIVED);
}

int cc1101_add_cb (const struct device *dev, cc1101_callback_t callback, void *user_data)
{
    struct cc1101_data *data = dev->data;
    data->callback.callback = callback;
    data->callback.user_data = user_data;

    return 0;
}
