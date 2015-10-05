/*
 * Copyright (C) 2015 Daniel Amkaer Sorensen
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_nrf905
 * @{
 *
 * @file
 * @brief       Low-level driver for nrf905 transceiver
 *
 * @author      Daniel Amkaer Sorensen <daniel.amkaer@gmail.com>
 *
 * @}
 */

#ifndef NRF905_SETTINGS_H
#define NRF905_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_R_REGISTER          0x00
#define CMD_W_REGISTER          0x10
#define CMD_R_RX_PAYLOAD        0x28
#define CMD_W_TX_PAYLOAD        0x20
#define CMD_NOP                 0x00

#define REGISTER_MASK           0x0F

#define REG_CH_NO 				0x00
#define REG_CONFIG0				0x01
#define REG_AFW 				0x02
#define REG_RX_PW 				0x03
#define REG_TX_PW 				0x04
#define REG_RX_ADDRESS 			0x05
#define REG_CONFIG1 			0x09

#define AUTO_RETRAN 			0x05
#define RX_RED_PWR 				0x04
#define PA_PWR 					0x02
#define PA_PWR_MASK  			0x0C
#define HFREQ_PLL 				0x01
#define CH_NO8 					0x00
#define TX_AFW 					0x04
#define TX_AFW_MASK 			0x70
#define RX_AFW 					0x00
#define RX_AFW_MASK 			0x07
#define UP_CLK_FREQ 			0x00
#define UP_CLK_EN 				0x02
#define XOF 					0x03
#define CRC_EN 					0x06
#define CRC_MODE 				0x07

#ifdef __cplusplus
}
#endif

#endif /* NRF905_SETTINGS_H */
