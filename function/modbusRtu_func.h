#ifndef __MODBUS_RTU_FUNC__
#define __MODBUS_RTU_FUNC__
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
 * modbusRtu_func.h
 *
 * History
 * 13.01.2024 Initial version
 * 
 */

#define FRAME_MAX_DATA_LEN 4

#ifndef NR_OF_AO_REGISTERS
  #define NR_OF_AO_REGISTERS 16
#endif
#ifndef NR_OF_AI_REGISTERS
  #define NR_OF_AI_REGISTERS 16
#endif


typedef struct
{
  uint8_t  address;
  uint8_t  function;
  uint16_t regOffset;
  uint16_t len;
  uint16_t data[FRAME_MAX_DATA_LEN];
  uint16_t crc;
} rtu_request_t;

enum
{
  FRAME_IDLE,
  FRAME_OK,
  FRAME_INVALID_ADR,
  FRAME_CRC_ERROR,
  FRAME_INVALID
};


#define FUNC_READ_DISCRETE_OUTPUT        0x01
#define FUNC_READ_DISCRETE_INPUT         0x02
#define FUNC_READ_ANALOG_OUTPUT          0x03
#define FUNC_READ_ANALOG_INPUT           0x04
#define FUNC_WRITE_DISCRETE_OUTPUT       0x05
#define FUNC_WRITE_ANALOG_OUTPUT         0x06
#define FUNC_WRITE_DISCRETE_OUTPUT_MULTI 0x15
#define FUNC_WRITE_ANALOG_OUTPUT_MULTI   0x16

extern uint16_t analogOutput[]; // Read/Write -> Func 0x06 / 0x16
extern uint16_t analogInput[];  // Read Only  ->


uint16_t calculate_crc(uint8_t* data, uint8_t len);
void mbrtu_init(uint8_t adr);
void mbrtu_set_parameter(uint8_t ticks_per_ms, uint8_t baud);
uint8_t mbrtu_evaluate_frame(uint8_t* data, uint8_t len, rtu_request_t* request);
void mbrtu_generate_confirmation(rtu_request_t* request);
uint8_t mbrtu_generate_frame(uint8_t function, uint16_t* data, uint8_t nrWords);
void mbrtu_send_frame(uint8_t* data, uint8_t len);
void mbrtu_store_confirm(rtu_request_t* request);
uint8_t mbrtu_check_buffer(rtu_request_t* request);
uint16_t get_AI_register(uint8_t offset);
uint8_t get_AI_register_lo(uint8_t offset);
uint8_t get_AI_register_hi(uint8_t offset);
uint16_t get_AO_register(uint8_t offset);
uint8_t get_AO_register_lo(uint8_t offset);
uint8_t get_AO_register_hi(uint8_t offset);


#endif // __MODBUS_RTU_FUNC__
