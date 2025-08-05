#ifndef I2C_FUNC_H
#define I2C_FUNC_H
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
 * I2C_func_v1.c
 * 
 * Provides functions to use I2C on an Atmel series 1 device
 *
 * History
 * 24.06.2023 15:14:21 Initial version
 */


#include "config.h"
#define I2C_BAUD_100kHz  1
#define I2C_BAUD_400kHz  4
#define I2C_BAUD_1MHz   10

enum {
    I2C_OK = 0,
    I2C_ERROR,
    I2C_ADR_SENT,
    I2C_ADR_ACK,
    I2C_ADR_NACK
};

#define I2C_WRITE 0
#define I2C_READ  1

/**
 * Init I2C as Master with the specified baud rate.
 */
void I2C_init_master(const uint8_t baud);

// REVIEW: Unterschied zw. start und start_condition?
/**
 * Start an I2C communication.
 * @param address I2C bus address
 * @param rw R/W mode, one of I2C_READ, I2C_WRITE
 * @return I2C status code
 */
uint8_t I2C_start(uint8_t address, uint8_t rw);
uint8_t I2C_start_condition(uint8_t address, uint8_t rw);

/**
 * Stop the current I2C communication.
 */
uint8_t I2C_stop(void);

/**
 * Write a single byte to the I2C bus.
 */
uint8_t I2C_write(uint8_t byte);
/**
 * Read a single byte from the I2C bus.
 */
uint8_t I2C_read(void);
/**
 * Read a single byte from the I2C bus and send NAK to mark 
 * that no more bytes will be read.
 */
uint8_t I2C_read_last(void);
/**
 * Create an I2C start condition, read a 16 bit word and stop.
 * @param address I2C bus address
 * @param word Destination word
 * @return I2C status code
 */
uint8_t I2C_read_word(uint8_t address, uint16_t* word);
/**
 * Read bytes from the I2C bus into a buffer, including start and stop.
 * This call is blocking.
 * @param address I2C bus address
 * @param buffer Destination buffer
 * @param size Number of bytes to read
 * @return I2C status code
 */
uint8_t I2C_read_buffer(uint8_t address, uint8_t* buffer, uint8_t size);
/**
 * Write bytes from a buffer into the I2C bus, including start and stop.
 * This call is blocking.
 * @param address I2C bus address
 * @param buffer Source buffer
 * @param size Number of bytes to write
 * @return I2C status code
 */
uint8_t I2C_write_buffer(uint8_t address, uint8_t* buffer, uint8_t size);

/**
 * Scan for a responding I2C address.
 * @param start_addr Start address for the scan
 * @return The found address, or 0 if no address was found.
 */
uint8_t I2C_addr_scan(uint8_t start_addr);

/**
 * Get string representation for I2C return codes.
 */
const char* I2C_getReturnString(uint8_t value);
/**
 * Get string representation for I2C Mstatus codes.
 */
const char* I2C_getMstatusString(uint8_t value);


/**
 * Init I2C as slave with the specified address.
 * @param addr I2C bus address to use for this device.
 */
void I2C_init_slave(uint8_t addr);

extern volatile uint8_t I2C_received;
extern volatile uint8_t I2C_send;

/** @defgroup I2C_ReloadEndMode_definition
  * @{
  */
#define  I2C_Reload_Mode                I2C_CR2_RELOAD
#define  I2C_AutoEnd_Mode               I2C_CR2_AUTOEND
#define  I2C_SoftEnd_Mode               ((uint32_t)0x00000000)


#define IS_RELOAD_END_MODE(MODE)        (((MODE) == I2C_Reload_Mode) || \
                                         ((MODE) == I2C_AutoEnd_Mode) || \
                                         ((MODE) == I2C_SoftEnd_Mode))



/** @defgroup I2C_StartStopMode_definition
  * @{
  */
#define  I2C_No_StartStop                 ((uint32_t)0x00000000)
#define  I2C_Generate_Stop                I2C_CR2_STOP
#define  I2C_Generate_Start_Read          (uint32_t)(I2C_CR2_START | I2C_CR2_RD_WRN)
#define  I2C_Generate_Start_Write         I2C_CR2_START


#define IS_START_STOP_MODE(MODE)        (((MODE) == I2C_Generate_Stop) || \
                                         ((MODE) == I2C_Generate_Start_Read) || \
                                         ((MODE) == I2C_Generate_Start_Write) || \
                                         ((MODE) == I2C_No_StartStop))


#endif /* I2C_FUNC_H */
