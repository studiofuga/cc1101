#ifndef DRIVERS_CC1101_CONFIG_H
#define DRIVERS_CC1101_CONFIG_H

#include "cc1101.h"

int cc1101_set_output_power(const struct device *dev, int8_t power);
int cc1101_set_frequency(const struct device *dev, uint32_t freq);
int cc1101_set_bitrate(const struct device *dev, uint32_t br);
int cc1101_set_bw(const struct device *dev, float rxBw);
int cc1101_set_deviation(const struct device *dev, float freqDev);
int cc1101_set_sync_type(const struct device *dev, enum Cc1101SyncType type);
int cc1101_set_preamble_length(const struct device *dev, enum Cc1101PreambleTypes type);
int cc1101_set_modulation(const struct device *dev, enum Cc1101Modulation modulation);
int cc1101_set_variable_length_packet(const struct device *dev);
int cc1101_set_sync_words(const struct device *dev, uint8_t w1, uint8_t w2);
int cc1101_enable_crc(const struct device *dev);
int cc1101_enable_whitening(const struct device *dev);
int cc1101_set_maximum_packet_length(const struct device *dev,uint8_t max);


#endif 