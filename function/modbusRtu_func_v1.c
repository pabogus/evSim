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
 * modbusRtu_func_v1.c
 * Provides basic functions to handle modbus RTU messages
 * 
 * History
 * 13.01.2024 Initial version
 * 29.08.2024 Some corrections
 * 
 */

#include "config.h"
#include "modbusRtu_func.h"
#include "rs232.h"


#define TRANSMIT_ENABLE_PORT PORTC_OUT
#define TRANSMIT_ENABLE_PIN  0

#define MAX_FRAME_BUFFER_LEN 20


uint16_t analogOutput[NR_OF_AO_REGISTERS]; // Func 0x06 / 0x16
uint16_t analogInput[NR_OF_AI_REGISTERS];

uint8_t frame[MAX_FRAME_BUFFER_LEN];
uint8_t ownAdr = 0;
uint8_t frameGapTicks = 10;


/*
//--------------------------------------------------------------------
// configure modbus RTU functions
//--------------------------------------------------------------------
*/
// #define NR_OF_AO_REGISTERS 16
// #define NR_OF_AI_REGISTERS 16
// #define RTU_TIMOUT_COUNTER TCA0_SINGLE_CNT


uint16_t calculate_crc(uint8_t* data, uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t pos = 0; pos < len; pos++)
  {
    crc ^= (uint16_t)data[pos];          // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--)
    {
      // Loop over each bit
      if ((crc & 0x0001) != 0)
      {
        // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}


void mbrtu_init(uint8_t adr)
{
  ownAdr = adr;
}


void mbrtu_set_parameter(uint8_t ticks_per_ms, uint8_t baud)
{
  switch (baud)
  {
    case BR_115200:
    case BR_57600:
    case BR_38400:
      frameGapTicks = ticks_per_ms*1 + (ticks_per_ms >> 1);
      break;
    case BR_19200:
    case BR_9600:
    default:
      frameGapTicks = ticks_per_ms*3 + (ticks_per_ms >> 1);
      break;
  }
}

uint8_t mbrtu_evaluate_frame(uint8_t* data, uint8_t len, rtu_request_t* request)
{
  // Ensure minimum data length
  if (len < 8)
    return FRAME_INVALID;

  request->address =   data[0];
  request->function =  data[1];
  request->regOffset = data[2] << 8 | data[3];
  request->len =       data[4] << 8 | data[5];
  request->crc =       data[len-2] << 8 | data[len-1];

  if (request->address != ownAdr)
    return FRAME_INVALID_ADR;

  if (FUNC_WRITE_ANALOG_OUTPUT == request->function)
  {
    request->crc = 1;
    request->data[0] = data[4] << 8 | data[5];
  }
  if (FUNC_WRITE_DISCRETE_OUTPUT == request->function)
  {
    request->crc = 1;
    request->data[0] = data[4] & 0x01;
  }
  return FRAME_OK;
}

void mbrtu_generate_confirmation(rtu_request_t* request)
{
  uint16_t crc;
  frame[0] = request->address;
  frame[1] = request->function;

  frame[2] = request->regOffset >> 8;
  frame[3] = request->regOffset;

  frame[4] = request->len >> 8;
  frame[5] = request->len;

  crc = calculate_crc(frame, 6);

  frame[6] = crc;
  frame[7] = crc >> 8;
  mbrtu_send_frame(frame, 8);
}

uint8_t mbrtu_generate_frame(uint8_t function, uint16_t* data, uint8_t nrWords)
{
  uint16_t crc;
  uint8_t len = nrWords * 2;
  // Check buffer size
  if (len + 5 > MAX_FRAME_BUFFER_LEN)
    return FRAME_INVALID;
  frame[0] = ownAdr;
  frame[1] = function;
  frame[2] = len;
  for (uint8_t i = 0; i < nrWords; i++)
  {
    frame[i*2 + 3] = (uint8_t)(data[i] >> 8);
    frame[i*2 + 4] = (uint8_t)(data[i]);
  }
  crc = calculate_crc(frame, len + 3);

  frame[len + 3] = (uint8_t)crc;
  frame[len + 4] = (uint8_t)(crc >> 8);
  mbrtu_send_frame(frame, len + 5);
  return FRAME_OK;
}


void mbrtu_send_frame(uint8_t* data, uint8_t len)
{
  TRANSMIT_ENABLE_PORT = TRANSMIT_ENABLE_PORT | _BV(TRANSMIT_ENABLE_PIN);
  _delay_us(10);
  USART0.STATUS = USART0.STATUS | USART_TXCIF_bm;
  for(uint8_t idx = 0; idx < len; idx++)
  {
    USART0_sendChar(data[idx]);
  }
  while (!(USART0.STATUS & USART_TXCIF_bm)) {;}
  _delay_us(10);
  TRANSMIT_ENABLE_PORT = TRANSMIT_ENABLE_PORT & ~_BV(TRANSMIT_ENABLE_PIN);
}


void mbrtu_store_confirm(rtu_request_t* request)
{
  uint16_t crc;

  analogOutput[request->regOffset] = request->data[0];

  frame[0] = request->address;
  frame[1] = request->function;

  frame[2] = request->regOffset >> 8;
  frame[3] = request->regOffset;

  frame[4] = request->len >> 8;
  frame[5] = request->len;

  crc = calculate_crc(frame, 6);

  frame[6] = crc;
  frame[7] = crc >> 8;
  mbrtu_send_frame(frame, 8);
}


uint8_t mbrtu_check_buffer(rtu_request_t* request)
{
  uint8_t ret = FRAME_IDLE;
  uint8_t frameLen;
  frameLen = nr_bytes_in_buffer();
  if (frameLen != 0 && RTU_TIMOUT_COUNTER > frameGapTicks)
  {
    read_UART_buffer(frame, MAX_FRAME_BUFFER_LEN);
    ret = mbrtu_evaluate_frame(frame, frameLen, request);
  }
  return ret;
}

uint16_t get_AI_register(uint8_t offset)
{
  return analogInput[offset];
}

uint8_t get_AI_register_lo(uint8_t offset)
{
  return analogInput[offset] & 0xff;
}

uint8_t get_AI_register_hi(uint8_t offset)
{
  return analogInput[offset] >> 8;
}

uint16_t get_AO_register(uint8_t offset)
{
  return analogOutput[offset];
}

uint8_t get_AO_register_lo(uint8_t offset)
{
  return analogOutput[offset] & 0xff;
}

uint8_t get_AO_register_hi(uint8_t offset)
{
  return analogOutput[offset] >> 8;
}
