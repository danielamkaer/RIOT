/*
 * Copyright (C) 2015 Daniel Amkaer Sorensen
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    drivers_nrf905 NRF905 Driver Interface
 * @ingroup     drivers_netdev
 *
 * @brief       Low-level driver for nrf905 transceiver
 *
 * @{
 * @file
 *
 * @author      Daniel Amkaer Sorensen <daniel.amkaer@gmail.com>
 *
 */

#ifndef NRF905_H
#define NRF905_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "periph/gpio.h"
#include "periph/spi.h"

/**
 * @brief Structure that represents the hardware setup of the nrf905 transceiver.
 */
typedef struct {
    spi_t spi;      /**< SPI device to initialize */
    gpio_t ce;      /**< GPIO pin to initialize as chip enable */
    gpio_t txen;    /**< GPIO pin to initialize as tx enable */
    gpio_t pwr;     /**< GPIO pin to initialize as pwr_up */
    gpio_t csn;      /**< GPIO pin to initialize as chip select */
    gpio_t am;      /**< GPIO pin to initialize as address match */
    gpio_t dr;      /**< GPIO pin to initialize as data ready */
    gpio_t cd;      /**< GPIO pin to initialize as carrier detect */
    int listener;   /**< Place to store an ID in */
} nrf905_t;

/**
 * @brief Defines the address width of the nrf24l01+ transceiver.
 */
typedef enum {
    NRF905_AW_1BYTE, /**< address width is 1 Byte */
    NRF905_AW_2BYTE, /**< address width is 2 Byte */
    NRF905_AW_3BYTE, /**< address width is 3 Byte */
    NRF905_AW_4BYTE  /**< address width is 4 Byte */
} nrf905_aw_t;

/**
 * @brief Defines the RF power level.
 */
typedef enum {
    NRF905_PWR_N10DBM = 0,   /**< power is -10dBm */
    NRF905_PWR_N2DBM,        /**< power is -2dBm  */
    NRF905_PWR_6DBM,         /**< power is  6dBm  */
    NRF905_PWR_10DBM         /**< power is  10dBm */
} nrf905_pwr_t;

/**
 * @brief Defines the error detection encoding scheme for the nrf905 transceiver.
 */
typedef enum {
    NRF905_CRC_1BYTE = 0,    /**< encoding scheme generates 1 Byte redundancy */
    NRF905_CRC_2BYTE,        /**< encoding scheme generates 2 Bytes redundancy */
} nrf905_crc_t;

/**
* @brief Initialize the nrf905 transceiver.
*
* @ note
* This function initializes the transceiver so that it is ready to use.
*
* @param[in] dev    Transceiver device to use.
* @param[in] spi    SPI device to use.
* @param[in] ce     GPIO pin to use for chip enable.
* @param[in] csn    GPIO pin to use for chip select.
* @param[in] txen   GPIO pin to use for tx/rx selection.
* @param[in] pwr    GPIO pin to use for power up.
* @param[in] am     GPIO pin to use for address match.
* @param[in] dr     GPIO pin to use for data ready.
* @param[in] cd     GPIO pin to use for carrier detect.
*
* @return           1 on success.
* @return           -1 on error.
*/
int nrf905_init(nrf905_t *dev, spi_t spi, gpio_t ce, gpio_t csn, gpio_t txen, gpio_t pwr, gpio_t am, gpio_t dr, gpio_t cd);

/**
* @brief Power off the nrf905 transceiver.
*
* @param[in] dev    Transceiver device to use.
*
* @return           0 on success.
* @return           -1 on error.
*/
int nrf905_off(nrf905_t *dev);

/**
* @brief Power on the nrf905 transceiver in transmit mode.
*
* @param[in] dev    Transceiver device to use.
*
* @return           0 on success.
* @return           -1 on error.
*/
int nrf905_on_receive(nrf905_t *dev);

/**
* @brief Power on the nrf905 transceiver in transmit mode.
*
* @param[in] dev    Transceiver device to use.
*
* @return           0 on success.
* @return           -1 on error.
*/
int nrf905_on_transmit(nrf905_t *dev);

/**
* @brief Read one register of the nrf905 transceiver.
*
* @param[in] dev    Transceiver device to use.
* @param[in] reg    Register address to read from.
* @param[in] answer Byte to read.
*
* @return           1 on success.
* @return           -1 on error.
*/
int nrf905_read_reg(nrf905_t *dev, char reg, char *answer);

/**
* @brief Write one register to the nrf905 transceiver.
*
* @param[in] dev    Transceiver device to use.
* @param[in] reg    Register address to write to.
* @param[in] write  Byte to write.
*
* @return           1 on success.
* @return           -1 on error.
*/
int nrf905_write_reg(nrf905_t *dev, char reg, char write);

int nrf905_set_channel(nrf905_t *dev, uint16_t chan);
int nrf905_set_auto_retran(nrf905_t *dev, bool enabled);
int nrf905_set_rx_reduced_power(nrf905_t *dev, bool enabled);
int nrf905_set_tx_power(nrf905_t *dev, nrf905_pwr_t power);
int nrf905_set_hfreq(nrf905_t *dev, bool enabled);
int nrf905_set_tx_address_width(nrf905_t *dev, nrf905_aw_t width);
int nrf905_set_rx_address_width(nrf905_t *dev, nrf905_aw_t width);
int nrf905_set_tx_payload_width(nrf905_t *dev, uint8_t width);
int nrf905_set_rx_payload_width(nrf905_t *dev, uint8_t width);
int nrf905_set_crc_en(nrf905_t *dev, bool enabled);
int nrf905_set_crc_len(nrf905_t *dev, nrf905_crc_t len);

#endif /* NRF905_H */
/** @} */
