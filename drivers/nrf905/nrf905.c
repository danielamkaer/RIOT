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
 * @author      Daniel Amkaer Sorensen <daniel.amkaer@gmail.com>
 * @}
 */
#include "nrf905.h"
#include "include/nrf905_settings.h"
#include "mutex.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "xtimer.h"
#include "thread.h"
#include "msg.h"


#define ENABLE_DEBUG (0)
#include "debug.h"

int nrf905_init(nrf905_t *dev, spi_t spi, gpio_t ce, gpio_t csn, gpio_t txen, gpio_t pwr, gpio_t am, gpio_t dr, gpio_t cd) {

	int status;

	dev->spi = spi;
	dev->ce = ce;
	dev->csn = csn;
	dev->txen = txen;
	dev->pwr = pwr;
	dev->am = am;
	dev->dr = dr;
	dev->cd = cd;
	dev->listener = KERNEL_PID_UNDEF;

	gpio_init(dev->ce, GPIO_DIR_OUT, GPIO_NOPULL);
	gpio_init(dev->csn, GPIO_DIR_OUT, GPIO_NOPULL);
	gpio_init(dev->txen, GPIO_DIR_OUT, GPIO_NOPULL);
	gpio_init(dev->pwr, GPIO_DIR_OUT, GPIO_NOPULL);
	gpio_init(dev->am, GPIO_DIR_IN, GPIO_NOPULL);
	gpio_init(dev->dr, GPIO_DIR_IN, GPIO_NOPULL);
	gpio_init(dev->cd, GPIO_DIR_IN, GPIO_NOPULL);

	gpio_set(dev->csn);
	gpio_clear(dev->ce);
	gpio_clear(dev->pwr);
	gpio_clear(dev->txen);

	spi_poweron(dev->spi);
	spi_acquire(dev->spi);
	status = spi_init_master(dev->spi, SPI_CONF_FIRST_RISING, SPI_SPEED_400KHZ);
	spi_release(dev->spi);

	if (status < 0) {
		return status;
	}
	
	status = nrf905_set_channel(dev, 0b001101100);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_auto_retran(dev, false);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_rx_reduced_power(dev, false);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_hfreq(dev, false);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_xof(dev, NRF905_XOF_16MHZ);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_tx_address_width(dev, NRF905_AW_4BYTE);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_rx_address_width(dev, NRF905_AW_4BYTE);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_tx_payload_width(dev, 32);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_rx_payload_width(dev, 32);
	if (status < 0) {
		return status;
	}
	
	status = nrf905_set_tx_power(dev, NRF905_PWR_N10DBM);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_crc_en(dev, true);
	if (status < 0) {
		return status;
	}

	status = nrf905_set_crc_len(dev, NRF905_CRC_2BYTE);
	if (status < 0) {
		return status;
	}

	return 0;

}

int nrf905_transmit(nrf905_t *dev) {
	gpio_set(dev->txen);
	xtimer_spin(2);
	gpio_set(dev->ce);
	return 0;
}

int nrf905_receive(nrf905_t *dev) {
	gpio_clear(dev->txen);
	xtimer_spin(2);
	gpio_set(dev->ce);
	return 0;
}

int nrf905_stop(nrf905_t *dev) {
	gpio_clear(dev->ce);
	return 0;
}

int nrf905_on(nrf905_t *dev) {
	gpio_set(dev->pwr);
	return 0;
}

int nrf905_off(nrf905_t *dev) {
	gpio_clear(dev->pwr);
	return 0;
}

int nrf905_read_reg(nrf905_t *dev, char reg, char *answer) {
	int status;

	spi_acquire(dev->spi);
	gpio_clear(dev->csn);
	xtimer_spin(2);
	status = spi_transfer_reg(dev->spi, (CMD_R_REGISTER | (REGISTER_MASK & reg)), 0, answer);
	xtimer_spin(2);
	gpio_set(dev->csn);
	spi_release(dev->spi);

	return status;

}

int nrf905_write_reg(nrf905_t *dev, char reg, char write) {
	int status;
	char reg_content;

	spi_acquire(dev->spi);
	gpio_clear(dev->csn);
	xtimer_spin(2);
	status = spi_transfer_reg(dev->spi, (CMD_W_REGISTER | (REGISTER_MASK & reg)), write, &reg_content);
	xtimer_spin(2);
	gpio_set(dev->csn);
	spi_release(dev->spi);

	return status;

}

int nrf905_set_channel(nrf905_t *dev, uint16_t chan) {
	int status;
	uint8_t chanh = (chan&0x100)>>8;
	uint8_t chanl = (chan&0xff);

	status = nrf905_write_reg(dev, REG_CH_NO, chanl);
	if (status < 0) {
		return status;
	}

	char config0;
	status = nrf905_read_reg(dev, REG_CONFIG0, &config0);
	if (status < 0) {
		return status;
	}

	config0 &= ~(1<<CH_NO8);
	config0 |= chanh;
	status = nrf905_write_reg(dev, REG_CONFIG0, config0);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_auto_retran(nrf905_t *dev, bool enabled) {
	int status;
	char config0;

	status = nrf905_read_reg(dev, REG_CONFIG0, &config0);
	if (status < 0) {
		return status;
	}

	config0 &= ~(1<<AUTO_RETRAN);
	if (enabled) {
		config0 |= (1<<AUTO_RETRAN);
	}
	status = nrf905_write_reg(dev, REG_CONFIG0, config0);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_rx_reduced_power(nrf905_t *dev, bool enabled) {
	int status;
	char config0;

	status = nrf905_read_reg(dev, REG_CONFIG0, &config0);
	if (status < 0) {
		return status;
	}

	config0 &= ~(1<<RX_RED_PWR);
	if (enabled) {
		config0 |= (1<<RX_RED_PWR);
	}
	status = nrf905_write_reg(dev, REG_CONFIG0, config0);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_tx_power(nrf905_t *dev, nrf905_pwr_t power) {
	int status;
	char config0;

	status = nrf905_read_reg(dev, REG_CONFIG0, &config0);
	if (status < 0) {
		return status;
	}

	config0 &= ~PA_PWR_MASK;
	switch (power) {
		case NRF905_PWR_N2DBM:
			config0 |= (0b01) << PA_PWR;
			break;
		case NRF905_PWR_6DBM:
			config0 |= (0b10) << PA_PWR;
			break;
		case NRF905_PWR_10DBM:
			config0 |= (0b11) << PA_PWR;
			break;
		default:
			/* Default is -10 dBm */
			break;
	}

	status = nrf905_write_reg(dev, REG_CONFIG0, config0);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_xof(nrf905_t *dev, nrf905_xof_t freq) {
	int status;
	char config1;

	status = nrf905_read_reg(dev, REG_CONFIG1, &config1);
	if (status < 0) {
		return status;
	}

	config1 &= ~(XOF_MASK << XOF);
	switch (freq) {
		case NRF905_XOF_8MHZ:
			config1 |= (1<<XOF);
			break;
		case NRF905_XOF_12MHZ:
			config1 |= (0b10<<XOF);
			break;
		case NRF905_XOF_16MHZ:
			config1 |= (0b11<<XOF);
			break;
		case NRF905_XOF_20MHZ:
			config1 |= (0b100<<XOF);
			break;
		default:
			// 4 Mhz
			break;
	}
	status = nrf905_write_reg(dev, REG_CONFIG1, config1);
	if (status < 0) {
		return status;
	}
	return 0;

}

int nrf905_set_hfreq(nrf905_t *dev, bool enabled) {
	int status;
	char config0;

	status = nrf905_read_reg(dev, REG_CONFIG0, &config0);
	if (status < 0) {
		return status;
	}

	config0 &= ~(1<<HFREQ_PLL);
	if (enabled) {
		config0 |= (1<<HFREQ_PLL);
	}
	status = nrf905_write_reg(dev, REG_CONFIG0, config0);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_tx_address_width(nrf905_t *dev, nrf905_aw_t width) {
	int status;
	char afw;

	status = nrf905_read_reg(dev, REG_AFW, &afw);
	if (status < 0) {
		return status;
	}

	afw &= ~TX_AFW_MASK;
	switch (width) {
		case NRF905_AW_1BYTE:
			afw |= (0b001 << TX_AFW);
			break;
		case NRF905_AW_2BYTE:
			afw |= (0b010 << TX_AFW);
			break;
		case NRF905_AW_3BYTE:
			afw |= (0b011 << TX_AFW);
			break;
		case NRF905_AW_4BYTE:
			afw |= (0b100 << TX_AFW);
			break;
	}
	status = nrf905_write_reg(dev, REG_AFW, afw);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_rx_address_width(nrf905_t *dev, nrf905_aw_t width) {
	int status;
	char afw;

	status = nrf905_read_reg(dev, REG_AFW, &afw);
	if (status < 0) {
		return status;
	}

	afw &= ~RX_AFW_MASK;
	switch (width) {
		case NRF905_AW_1BYTE:
			afw |= (0b001 << RX_AFW);
			break;
		case NRF905_AW_2BYTE:
			afw |= (0b010 << RX_AFW);
			break;
		case NRF905_AW_3BYTE:
			afw |= (0b011 << RX_AFW);
			break;
		case NRF905_AW_4BYTE:
			afw |= (0b100 << RX_AFW);
			break;
	}
	status = nrf905_write_reg(dev, REG_AFW, afw);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_tx_payload_width(nrf905_t *dev, uint8_t width) {
	int status;
	if (width > 32) {
		return -1;
	}

	status = nrf905_write_reg(dev, REG_TX_PW, width);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_rx_payload_width(nrf905_t *dev, uint8_t width) {
	int status;
	if (width > 32) {
		return -1;
	}

	status = nrf905_write_reg(dev, REG_RX_PW, width);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_crc_en(nrf905_t *dev, bool enabled) {
	int status;
	char config1;

	status = nrf905_read_reg(dev, REG_CONFIG1, &config1);
	if (status < 0) {
		return status;
	}

	config1 &= ~(1<<CRC_EN);
	if (enabled) {
		config1 |= (1<<CRC_EN);
	}
	status = nrf905_write_reg(dev, REG_CONFIG1, config1);
	if (status < 0) {
		return status;
	}

	return 0;
}

int nrf905_set_crc_len(nrf905_t *dev, nrf905_crc_t len) {
	int status;
	char config1;

	status = nrf905_read_reg(dev, REG_CONFIG1, &config1);
	if (status < 0) {
		return status;
	}

	config1 &= ~(1<<CRC_MODE);
	if (len == NRF905_CRC_2BYTE) {
		config1 |= (1<<CRC_MODE);
	}
	status = nrf905_write_reg(dev, REG_CONFIG1, config1);
	if (status < 0) {
		return status;
	}

	return 0;
}

bool nrf905_data_ready(nrf905_t *dev) {
	return (gpio_read(dev->dr) > 0);
}

bool nrf905_carrier_detect(nrf905_t *dev) {
	return (gpio_read(dev->cd) > 0);
}

bool nrf905_address_match(nrf905_t *dev) {
	return (gpio_read(dev->am) > 0);
}

int nrf905_write_payload(nrf905_t *dev, char *payload, int len) {
	int status;

	spi_acquire(dev->spi);
	gpio_clear(dev->csn);
	xtimer_spin(2);
	status = spi_transfer_regs(dev->spi, CMD_W_TX_PAYLOAD, payload, NULL, len);
	xtimer_spin(2);
	gpio_set(dev->csn);
	spi_release(dev->spi);

	return status;

}

int nrf905_read_payload(nrf905_t *dev, char *answer, int len) {
	int status;

	spi_acquire(dev->spi);
	gpio_clear(dev->csn);
	xtimer_spin(2);
	status = spi_transfer_regs(dev->spi, CMD_R_RX_PAYLOAD, NULL, answer, len);
	xtimer_spin(2);
	gpio_set(dev->csn);
	spi_release(dev->spi);

	return status;

}

int nrf905_set_rx_address(nrf905_t *dev, char *address, int len) {

	int status;

	spi_acquire(dev->spi);
	gpio_clear(dev->csn);
	xtimer_spin(2);
	status = spi_transfer_regs(dev->spi, (CMD_W_REGISTER | (REG_RX_ADDRESS)), address, NULL, len);
	xtimer_spin(2);
	gpio_set(dev->csn);
	spi_release(dev->spi);

	return status;

}

int nrf905_set_tx_address(nrf905_t *dev, char *address, int len) {

	int status;

	spi_acquire(dev->spi);
	gpio_clear(dev->csn);
	xtimer_spin(2);
	status = spi_transfer_regs(dev->spi, CMD_W_TX_ADDRESS, address, NULL, len);
	xtimer_spin(2);
	gpio_set(dev->csn);
	spi_release(dev->spi);

	return status;

}
