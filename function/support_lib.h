#ifndef _SUPPORT_LIB_H_
#define _SUPPORT_LIB_H_
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
 * Provides support functions to create strings and output to displays
 * 
 * History
 * 04.07.2019 Initial version
 * 28.05.2023 Some more functions added
 */

extern volatile int8_t  turn_cnt;
extern volatile uint8_t turn_event;
extern int8_t rotenc_load;

/** Append a char to a string */
void strcat_char(char* buffer, char c);

/**
 * Convert uint16 to its string representation.
 * @param buffer Destination buffer
 * @param value Value to convert
 * @param base Base to use
 * @param size Length of the created string. Digits will be right-aligned
 *  and filled with spaces.
 */
void utostr(char* buffer, uint16_t value, char base, signed char size);
/**
 * Convert uint16 to its string representation and append it to another
 *  string.
 * @param buffer Destination buffer
 * @param value Value to convert
 * @param base Base to use
 * @param size Length of the created string. Digits will be right-aligned
 *  and filled with spaces.
 */
void strcat_utoa(char* buffer, uint16_t value, char base, signed char size);
/** Append a uint to a string using the default base 10 representation. */
void strcat_uint(char* buffer, uint16_t value);


void itostr(char* buffer, int value, char base, signed char size);
void strcat_itoa(char* buffer, int value, char base, signed char size);
void strcat_itoa_fixpoint(char* buffer, int value, uint8_t decimals);

void bittostr(char* buffer, uint8_t value);
void strcat_bittoa(char* buffer, int value);

/**
 * Convert 2 character hex string into the corresponding byte value.
 * The correct format is not checked. Supplying a string other than
 * the format [0-9a-fA-F]{2} will result in undefined behaviour.
 */
uint8_t getValueFromHex(char* string);
void strcat_hex(char* string, uint8_t val);
char nibble2hex(uint8_t val);
void strcat_hex_array(char* string, uint8_t* val, uint8_t len);
void strcat_hex_array_space(char* string, uint8_t* val, uint8_t len);


void handle_display_output(void);
void write_display_buffer(char* str);
void write_display_buffer_XY(uint8_t x, uint8_t y);

#define display_version() \
  display_str(__DATE__"\n");  \
  display_str(__BASE_FILE__)

#endif // _SUPPORT_LIB_H_
