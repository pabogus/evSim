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
 * support_lib_v02.c
 *
 * Provides support functions to create strings and output to displays
 * 
 * History
 * 04.07.2019 Initial version
 * 28.05.2023 Some more functions added
 */

#include <avr/io.h>
#include <string.h>
#include "config.h"
#include "support_lib.h"

/*
//--------------------------------------------------------------------
// configure support functions for strings
//--------------------------------------------------------------------
*/
//#define USE_BIT_TO_STR_FUNCTION
//#define USE_U_TO_STR_FUNCTION
//#define USE_I_TO_STR_FUNCTION
//#define USE_HEX_TO_STR_FUNCTION
//#define USE_DISPLAY_BUFFER
//#define SIZE_OF_DISPLAY_BUFFER 20


#define VAL2STR_MAX_DIGITS 20

void strcat_char(char* buffer, char c)
{
  /* search end of string */
  while (*buffer != 0)
    buffer++;
  *buffer++=c;
  *buffer=0;
}


#ifdef USE_BIT_TO_STR_FUNCTION
void bittostr(char* buffer, uint8_t value)
{
  uint8_t n;

  for (n = 0; n < 8; n++)
  {
    if (0 == (value & 0x80))
      *buffer = '0';
    else
      *buffer = '1';
    value <<= 1;
    buffer++;
  }
  *buffer = 0;
}

void strcat_bittoa(char* buffer, int value)
{
  /* search end of string */
  while (*buffer != 0)
    buffer++;
  bittostr(buffer, value);
}
#endif /* USE_BIT_TO_STR_FUNCTION */


#ifdef USE_U_TO_STR_FUNCTION
void utostr(char* buffer, uint16_t value, char base, signed char size)
{
  char digit[VAL2STR_MAX_DIGITS];
  char *digit_ptr;
  uint16_t r;

  digit_ptr = digit;
  *digit_ptr++ = 0;

  do {
    r = 0;
    while (value >= base)
    {
      value -= base;
      r++;
    }
    *digit_ptr = value + '0';
    if (*digit_ptr > '9')
      *digit_ptr += 'a' - '9' - 1;
    digit_ptr++;
    value = r;
    size--;
  } while (value);
  // Fill empty space to the left with space
  while (size-- > 0)
    *buffer++ = ' ';
  // Write digits reversed into output buffer
  while ((*buffer = *--digit_ptr))
    buffer++;
}

void strcat_utoa(char* buffer, uint16_t value, char base, signed char size)
{
  /* search end of string */
  while (*buffer != 0)
    buffer++;
  utostr(buffer, value, base, size);
}

void strcat_uint(char* buffer, uint16_t value)
{
  /* search end of string */
  while (*buffer != 0)
    buffer++;
  utostr(buffer, value, 10, 1);
}

#ifndef USE_HEX_TO_STR_FUNCTION
void strcat_hex(char* buffer, uint8_t value)
{
  /* search end of string */
  while (*buffer != 0)
    buffer++;
  utostr(buffer, value, 16, 2);
}
#endif // USE_HEX_TO_STR_FUNCTION

#endif // USE_U_TO_STR_FUNCTION


#ifdef USE_I_TO_STR_FUNCTION
void itostr(char* buffer, int value, char base, signed char size)
{
  char digit[VAL2STR_MAX_DIGITS];
  char *digit_ptr;
  int8_t neg=0;
  uint16_t r;

  /* search end of string */
  while (*buffer != 0)
    buffer++;
  digit_ptr = digit;
  *digit_ptr++ = 0;
  /* negative values turned into positive values*/
  if (value < 0)
  {
    neg = 1;
    value = -value;
  }
  /* conversion loop */
  do
  {
    r = 0;
    while (value >= base)
    {
      value -= base;
      r++;
    }
    *digit_ptr = value + '0';
    if (*digit_ptr > '9')
      *digit_ptr += 'a' - '9' - 1;
    digit_ptr++;
    value = r;
    size--;
  } while (value);
  /* add sign */
  if (neg)
  {
    *digit_ptr++ = '-';
    size--;
  }
  /* fill until size and add termination*/
  while (size-- > 0)
    *buffer++ = ' ';
  /* reorder numbers */
  while ((*buffer = *--digit_ptr))
    buffer++;
}

void strcat_itoa(char* buffer, int value, char base, signed char size)
{
  /* search end of string */
  while (*buffer != 0)
    buffer++;
  itostr(buffer, value, base, size);
}

void strcat_itoa_fixpoint(char* buffer, int value, uint8_t decimals)
{
  uint8_t len;
  strcat_itoa(buffer, value, 10, 1);
  len = strlen(buffer);
  buffer[len+1] = 0;
  while (decimals)
  {
    buffer[len] = buffer[len-1];
    len--;
    decimals--;
  }
  buffer[len] = '.';
}
#endif /* USE_I_TO_STR_FUNCTION */


#ifdef USE_HEX_TO_STR_FUNCTION
/**
 * Convert hex char of [0-9a-fA-F] into the byte value it represents.
 * Chars of other formats are undefined.
 */
static uint8_t getNibbleFromHexChar(char c)
{
  uint8_t x;
  if (c >= 'a')
    x = c - ('a' - 10);
  else if (c >= 'A')
    x = c - ('A' - 10);
  else
    x = c - '0';
  return x & 0x0f;
}

uint8_t getValueFromHex(char* string)
{
  return getNibbleFromHexChar(string[0]) << 4
    + getNibbleFromHexChar(string[1]);
}

char nibble2hex(uint8_t val)
{
  char c;
  val = val & 0x0f;
  if (val < 10)
    return val + '0';
  else
    return val - 10 + 'A';
}

void strcat_hex(char* string, uint8_t val)
{
  while (*string != 0)
    string++;
  *string++ = nibble2hex(val >> 4);
  *string++ = nibble2hex(val & 0x0f);
  *string = 0;
}

void strcat_hex_array(char* string, uint8_t* val, uint8_t len)
{
  uint8_t n;
  while (*string != 0)
    string++;
  for (n = 0; n < len; n++)
  {
    *string++ = nibble2hex(val[n] >> 4);
    *string++ = nibble2hex(val[n] & 0x0f);
  }
  *string = 0;
}

void strcat_hex_array_space(char* string, uint8_t* val, uint8_t len)
{
  uint8_t n;
  while (*string != 0)
    string++;
  for (n = 0; n < len; n++)
  {
    *string++ = nibble2hex(val[n] >> 4);
    *string++ = nibble2hex(val[n] & 0x0f4);
    *string++ = ' ';
  }
  *string = 0;
}
#endif /* USE_HEX_TO_STR_FUNCTION */


#ifdef USE_DISPLAY_BUFFER
static char display_buffer[SIZE_OF_DISPLAY_BUFFER];
static uint8_t d_read_idx, d_write_idx;

char get_char_from_buffer(void)
{
  char c = 0;
  if (d_read_idx != d_write_idx)
  {
    c = display_buffer[d_read_idx];
    d_read_idx++;
    if (d_read_idx == SIZE_OF_DISPLAY_BUFFER)
      d_read_idx = 0;
  }
  return c;
}

uint8_t handle_esc_sequence(void)
{
//  static uint8_t state=0;
  char c;
  char ret = 1;
  static uint8_t par[3];
  static uint8_t idx = 0;
//  char str[20];

  do
  {
    c = get_char_from_buffer();
    switch(c)
    {
      case '[':
        idx = 0;
        par[0] = 0;
        par[1] = 0;
        break;
      case 'H':
        ret = 0;
        gotoXY(par[0], par[1]);
//        gotoXY(0, 0); utostr(str, idx, 10, 2);
//        strcat_utoa(str, par[0], 10, 3); strcat_utoa(str, par[1], 10, 3); display_str(str);
        break;
      case ';':
        idx++;
        break;
      default:
        if ((c >= '0') && (c <= '9'))
          par[idx] = par[idx]*10 + c - '0';
        if ((c >= 'A' && c <= 'X') ||
            (c >= 'a' && c <= 'x'))
          ret = 0;
    }
    if (ret == 0)
      c = 0;
  } while (c != 0);
  return ret;
}

void handle_display_output(void)
{
  static uint8_t esc_sequence = 0;
  char c;

  if (esc_sequence == 0)
  {
    c = get_char_from_buffer();
    if (c == 27)
      esc_sequence = 1;
    else if (c != 0)
      display_char(c);
  }
  else
    esc_sequence = handle_esc_sequence();
}

void write_display_buffer(char* str)
{
  char c;
  while ((c = *str++) != 0)
  {
    display_buffer[d_write_idx] = c;
    d_write_idx++;
    if (d_write_idx == SIZE_OF_DISPLAY_BUFFER)
      d_write_idx = 0;
  }
}

void write_display_buffer_XY(uint8_t x, uint8_t y)
{
  char str[10];
  str[0] = 27;
  str[1] = '[';
  str[2] = 0;
  strcat_utoa(str, x, 10, 0);
  write_display_buffer(str);
  str[0] = ';';
  str[1] = 0;
  strcat_utoa(str, y, 10, 0);
  write_display_buffer(str);
  write_display_buffer("H");
}
#endif /* USE_DISPLAY_BUFFER */
