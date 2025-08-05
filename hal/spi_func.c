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
 * spi_func.c
 * Provides functions to exchange data via a SPI interface on a Attiny 1 series device
 *
 * History
 * 26.09.2018 17:08:31 Initial version
 */


#include <avr/io.h>
#include "config.h"



/*
//--------------------------------------------------------------------
// configure SPI functions
//--------------------------------------------------------------------
*/

/****************************************************************
https://github.com/MicrochipTech/TB3215_Getting_Started_with_SPI
----------------------------------------------------------------
*/
void SPI0_init_master(const uint8_t prescaler, const uint8_t clk2x)
{
    PORTA.DIR |= PIN1_bm;     /* Set MOSI pin direction to output */
    PORTA.DIR &= ~PIN2_bm;    /* Set MISO pin direction to input */
    PORTA.DIR |= PIN3_bm;     /* Set SCK pin direction to output */
    PORTA.DIR |= PIN4_bm;     /* Set SS pin direction to output */
    PORTA_PIN2CTRL = PORTA_PIN2CTRL | 0x08; /* Set MI pin pull-up */

    SPI0.CTRLA = clk2x           /* Enable double-speed, e.g. SPI_CLK2X_bm or 0 */
//             | SPI_DORD_bm     /* LSB is transmitted first */
               | SPI_ENABLE_bm   /* Enable module */
               | SPI_MASTER_bm   /* SPI module in Master mode */
               | prescaler;      /* System Clock divided parameter, e.g. SPI_PRESC_DIV128_gc */
//    SPI0.CTRLB = 1;
}

uint8_t SPI0_exchangeData(uint8_t data)
{
    SPI0.DATA = data;
    /* Waits until data is exchanged */
    while (!(SPI0.INTFLAGS & SPI_IF_bm)) {;}
    return SPI0.DATA;
}

void slaveSelect(void)
{
    PORTA.OUT &= ~PIN4_bm; // Set SS pin value to LOW
}

void slaveDeselect(void)
{
    PORTA.OUT |= PIN4_bm; // Set SS pin value to HIGH
}

