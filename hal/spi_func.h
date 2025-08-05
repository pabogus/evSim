#ifndef SPI_FUNC_H
#define SPI_FUNC_H
/*
 * This file is part of evSim
 *
 * Copyright (C) 2025 pabogus <https://github.com/pabogus>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * spi_func.h
 * Provides functions to exchange data via a SPI interface on a Attiny 1 series device
 *
 * History
 * 26.09.2018 17:08:31 Initial version
 */

#include <avr/io.h>
#include "config.h"

/**
 * Initialize SPI0.
 * 
 * @param prescaler System Clock divided parameter, e.g. SPI_PRESC_DIV128_gc
 */
void SPI0_init(const uint8_t prescaler);

/**
 * Send and receive a byte using SPI0.
 * 
 * @param data Byte to send
 * @return Response byte from SPI
 */
uint8_t SPI0_exchangeData(uint8_t data);

/**
 * Initialize SPI0 as master.
 * 
 * @param prescaler System Clock divided parameter, e.g. SPI_PRESC_DIV128_gc
 * @param clk2x Enable double-speed, e.g. SPI_CLK2X_bm or 0
 */
void SPI0_init_master(const uint8_t prescaler, const uint8_t clk2x);

/**
 * Set SS pin value to LOW.
 */
void slaveSelect(void);

/**
 * Set SS pin value to HIGH.
 */
void slaveDeselect(void);

#endif /* SPI_FUNC_H */
