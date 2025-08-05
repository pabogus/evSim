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
 * Input: https://www.bitbanging.space/posts/attiny-1-series-i2c-library
 *        http://www.technoblogy.com/show?2QYB
 *        http://www.technoblogy.com/show?3UF0
 *        https://github.com/MicrochipTech/TB3215_Getting_Started_with_SPI
 *
 * History
 * 24.06.2023 15:14:21 Initial version
 */


#include <avr/io.h>
#include "config.h"
#include "I2C_func.h"

/*
//--------------------------------------------------------------------
// configure TWI functions
//--------------------------------------------------------------------
*/
//#define USE_I2C_MASTER
//#define USE_I2C_MASTER_STR
//#define USE_I2C_MASTER_SUPPORT
//#define USE_I2C_SLAVE
//#define I2C_SLAVE_SEND_CALLBACK twi_send
//#define I2C_SLAVE_RECEIVED_CALLBACK(byte) twi_received=byte

#ifdef USE_I2C_MASTER

#ifdef USE_ALTRNATIVE_PINS
void I2C_init_master(const uint8_t baudSet)
{
  uint8_t baud = 0;
  if (baudSet == I2C_BAUD_100kHz) baud = (uint8_t)((((float) F_CPU / (float)  100000) - 10)/2);
  if (baudSet == I2C_BAUD_400kHz) baud = (uint8_t)((((float) F_CPU / (float)  400000) - 10)/2);
  if (baudSet == I2C_BAUD_1MHz)   baud = (uint8_t)((((float) F_CPU / (float) 1000000) - 10)/2);

  //TWI pins as outputs
  PORTMUX.CTRLB |= 0x10;

  PORTA.DIR |= _BV(2);     /* Set SCL pin direction to output */
  PORTA.DIR |= _BV(1);     /* Set SDA pin direction to output */

	//Activate internal pull-up resistors
	PORTA_PIN2CTRL = PORTA_PIN2CTRL | 0x08;
	PORTA_PIN1CTRL = PORTA_PIN1CTRL | 0x08;

  /* Master Baud Rate Control */
  TWI0.MBAUD = baud;

  /* Enable TWI */
  TWI0.MCTRLA = TWI_ENABLE_bm;

  /* Set bus state idle */
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

#else // USE_ALTRNATIVE_PINS

void I2C_init_master(const uint8_t baudSet)
{
  uint8_t baud = 0;
  if (baudSet == I2C_BAUD_100kHz) baud = (uint8_t)((((float) F_CPU / (float)  100000) - 10)/2);
  if (baudSet == I2C_BAUD_400kHz) baud = (uint8_t)((((float) F_CPU / (float)  400000) - 10)/2);
  if (baudSet == I2C_BAUD_1MHz)   baud = (uint8_t)((((float) F_CPU / (float) 1000000) - 10)/2);

  // TWI pins as outputs
  PORTB.DIR |= _BV(0);     /* Set SCL pin direction to output */
  PORTB.DIR |= _BV(1);     /* Set SDA pin direction to output */

	// Activate internal pull-up resistors
	PORTB_PIN0CTRL = PORTB_PIN0CTRL | 0x08;
	PORTB_PIN1CTRL = PORTB_PIN1CTRL | 0x08;

  /* Master Baud Rate Control */
  TWI0.MBAUD = baud;

  /* Enable TWI */
  TWI0.MCTRLA = TWI_ENABLE_bm;

  /* Set bus state idle */
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

#endif // USE_ALTRNATIVE_PINS


uint8_t I2C_start(uint8_t address, uint8_t rw)
{
  uint8_t state = 0;
  TWI0.MADDR = (address << 1) | rw;
  do
  {
    if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
    {
      state = I2C_ADR_SENT;
    }
    else if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
    {
      /* get here only in case of bus error or arbitration lost */
      state = I2C_ERROR;
    }
  } while (state == I2C_OK);
  // REVIEW: Should check for I2C_ERROR here?
  // Actual status of the line; 0 means ACK - 1 means NACK.
  if ((TWI0.MSTATUS & TWI_RXACK_bm) == 0)
  {
    state = I2C_OK;
  }
  else
  {
    // No ACK received send stop
    // REVIEW: return I2C_ERROR or sth specific?
    I2C_stop();
  }
  return state;
}


uint8_t I2C_start_condition(uint8_t address, uint8_t rw)
{
  uint8_t state = 0;
  TWI0.MADDR = (address << 1) | rw;
  do
  {
    if (TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm))
    {
      state = I2C_ADR_SENT;
    }
    else if (TWI0.MSTATUS & (TWI_BUSERR_bm | TWI_ARBLOST_bm))
    {
      /* get here only in case of bus error or arbitration lost */
      state = I2C_ERROR;
    }
  } while (state == I2C_OK);
  // Actual status of the line O means ACK - 1 means NACK.
  if ((TWI0.MSTATUS & TWI_RXACK_bm) == 0)
  {
    state = I2C_ADR_ACK;
  }
  return state;
}


uint8_t I2C_stop(void)
{
  TWI0.MCTRLB |= TWI_MCMD_STOP_gc;                                // Send STOP
  while (!(TWI0.MSTATUS & TWI_BUSSTATE_IDLE_gc)) {;}              // Wait for bus to return to idle state
  return I2C_OK;
}


uint8_t I2C_write(uint8_t  byte)
{
  TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;                            // Prime transaction
  TWI0.MDATA = byte;                                              // Send data
  while (!(TWI0.MSTATUS & TWI_WIF_bm)) {;}                        // Wait for write to complete
  if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm))
    return I2C_ERROR;                                             // Fails if bus error or arblost
  if (!(TWI0.MSTATUS & TWI_RXACK_bm))                             // Returns true if slave gave an ACK
  {
    return I2C_ADR_ACK;
  }
  else
  {
    return I2C_ERROR;
  }
}

uint8_t I2C_read(void)
{
  while (!(TWI0.MSTATUS & TWI_RIF_bm)) {;}  // Wait for read interrupt flag
  uint8_t data = TWI0.MDATA;
  TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;      // ACK = more bytes to read
  return data;
}

uint8_t I2C_read_last(void) {
  while (!(TWI0.MSTATUS & TWI_RIF_bm)) {;}  // Wait for read interrupt flag
  uint8_t data = TWI0.MDATA;
  TWI0.MCTRLB = TWI_ACKACT_NACK_gc;         // Send NAK
  return data;
}


uint8_t I2C_read_word(uint8_t address, uint16_t* word)
{
  uint8_t lsb, msb, ret;

  ret = I2C_start_condition(address, I2C_READ);
  if (ret != I2C_ADR_ACK)
    return ret;

  lsb = I2C_read();
  msb = I2C_read_last();
  *word = msb<<8 | lsb;

  return I2C_stop();
}


uint8_t I2C_read_buffer(uint8_t address, uint8_t* buffer, uint8_t size)
{
  uint8_t ret;

  ret = I2C_start_condition(address, I2C_READ);
  if (ret != I2C_ADR_ACK)
    return ret;

  while (size--)
  {
    while (!(TWI0.MSTATUS & TWI_RIF_bm)) {;}  // Wait for read interrupt flag
    *(buffer++) = TWI0.MDATA;
    if (size == 0)
      TWI0.MCTRLB = TWI_ACKACT_NACK_gc;       // last byte = Send NAK
    else
      TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;    // ACK = more bytes to read
  }

  return I2C_stop();
}


uint8_t I2C_write_buffer(uint8_t address, uint8_t* buffer, uint8_t size)
{
  uint8_t ret;
  ret = I2C_start_condition(address, I2C_WRITE);
  if (ret != I2C_ADR_ACK)
    return ret;

  while (size--)
  {
    ret = I2C_write(*(buffer++));
    if (ret != I2C_ADR_ACK)
      return ret;
  }

  return I2C_stop();
}


#ifdef USE_I2C_MASTER_STR
#include <string.h>
const char* I2C_getReturnString(uint8_t value)
{
  if (value == TWI_OK) return "Ok";
  if (value == TWI_ERROR) return "Err";
  if (value == TWI_ADR_SENT) return "Asnt";
  if (value == TWI_ADR_ACK) return "Aack";
  return "?";
}

const char* I2C_getMstatusString(uint8_t value)
{
  static char text[16];
  strcpy(text, "MS=");
  if (TWI0.MSTATUS & TWI_WIF_bm ) strcat(text, "W:");
  if (TWI0.MSTATUS & TWI_RIF_bm ) strcat(text, "R:");
  if (TWI0.MSTATUS & TWI_BUSERR_bm ) strcat(text, "E:");
  if (TWI0.MSTATUS & TWI_ARBLOST_bm ) strcat(text, "Al:");
  if (TWI0.MSTATUS & TWI_RXACK_bm ) strcat(text, "Ack:");
  if (TWI0.MSTATUS & TWI_CLKHOLD_bm ) strcat(text, "CH:");
  if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == 1 ) strcat(text, "Idl");
  if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == 2 ) strcat(text, "Own");
  if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == 3 ) strcat(text, "Bsy");

  return text;
}
#endif // USE_I2C_MASTER_STR

#ifdef USE_I2C_MASTER_SUPPORT
uint8_t I2C_addr_scan(uint8_t start_addr)
{
  uint8_t ret;
  while (start_addr < 128)
  {
    ret = I2C_start(start_addr, I2C_WRITE);
    if (ret == I2C_OK)
      return start_addr;
    start_addr++;
  }
  // REVIEW return 0; ?
  return I2C_OK;
}
#endif // USE_I2C_MASTER_SUPPORT

#endif // USE_I2C_MASTER


#ifdef USE_I2C_SLAVE

#ifndef I2C_SLAVE_SEND_CALLBACK
  #define I2C_SLAVE_SEND_CALLBACK I2C_send
#endif

#ifndef I2C_SLAVE_RECEIVED_CALLBACK
  #define I2C_SLAVE_RECEIVED_CALLBACK(byte) I2C_received=byte
#endif



/*
 * https://www.avrfreaks.net/s/topic/a5C3l000000UaV1EAK/t154097
 */
#include <avr/interrupt.h>


volatile uint8_t I2C_received = 0x00;
volatile uint8_t I2C_send = 0x00;
void I2C_init_slave(uint8_t addr)
{
  // TWI pins as outputs
  //  PORTB.DIR |= _BV(0);     /* Set SCL pin direction to output */
  //  PORTB.DIR |= _BV(1);     /* Set SDA pin direction to output */
  // Activate internal pull-up resistors
  PORTB_PIN0CTRL = PORTB_PIN0CTRL | 0x08;
  PORTB_PIN1CTRL = PORTB_PIN1CTRL | 0x08;

  TWI0.SADDR = (addr << 1) | 0x01;		//Slave address & enable general call
  TWI0.SCTRLA = TWI_ENABLE_bm |	    //Enable slave peripheral
                TWI_APIEN_bm |		//Enable address match interrupt
                TWI_PIEN_bm |			//Enable stop interrupt
                TWI_DIEN_bm |			//Enable data interrupt
                TWI_SMEN_bm;			//Enable smart mode
}


ISR(TWI0_TWIS_vect)
{
  if (TWI0.SSTATUS & TWI_APIF_bm)					//Address match/stop interrupt
  {
    if (TWI0.SSTATUS & TWI_COLL_bm)
    {
      TWI0.SSTATUS |= TWI_COLL_bm;			//Clear Collision flag
      TWI0_SCTRLB = TWI_SCMD_COMPTRANS_gc;	//complete transaction
      return;
    }
    if (TWI0.SSTATUS & TWI_AP_bm)
      TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;		//Send ACK after address match
    else
      TWI0_SCTRLB = TWI_SCMD_COMPTRANS_gc;	//complete transaction after Stop
  }

  if (TWI0.SSTATUS & TWI_DIF_bm)					//Data interrupt
  {
    if (TWI0.SSTATUS & TWI_DIR_bm)
    {
      TWI0.SDATA = TWI_SLAVE_SEND_CALLBACK;	//Transmit data for Master to read
      TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;
    }
    else
    {
      TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;
      TWI_SLAVE_RECEIVED_CALLBACK(TWI0.SDATA);	//Receive data written by Master
    }
  }
}

#endif // USE_I2C_SLAVE
