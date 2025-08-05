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
 * rs232.c
 * Provides functions to send and recive data via UART on a Attiny 1 series device
 * Implements a ring buffer and an interrupt routine.

 *
 * History
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "rs232.h"

/*
//--------------------------------------------------------------------
// configure USART Settings
//--------------------------------------------------------------------
*/
//#define USE_UART
//#define UART_RX_BUFFER_LEN 32
//#define UART_MAX_LINE_LENGTH 16
//#define USART_USE_RX_INTERRUPT
//#define USART_USE_RX_INTERRUPT_ECHO
//#define uart_rx_idx GPIOR1
//#define RESET_TIMEOUT TCNT=0

#ifdef USE_UART


#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#ifndef uart_rx_idx
  static uint8_t uart_rx_idx;
#endif

#ifndef UART_RX_BUFFER_LEN
  #define UART_RX_BUFFER_LEN 32
#endif

#ifndef RESET_TIMEOUT
  #define RESET_TIMEOUT
#endif

#ifdef USART_USE_RX_INTERRUPT
  char uart_rx_buffer[UART_RX_BUFFER_LEN];
  ISR(USART0_RXC_vect)
  {
    uart_rx_idx = (uart_rx_idx+1) & (UART_RX_BUFFER_LEN-1);
    uart_rx_buffer[uart_rx_idx] = USART0.RXDATAL;
  #ifdef USART_USE_RX_INTERRUPT_ECHO
    USART0.TXDATAL = uart_rx_buffer[uart_rx_idx];
  #endif
    RESET_TIMEOUT;
  }
#endif // USART_USE_RX_INTERRUPT

#ifndef UART_MAX_LINE_LENGTH
  #define UART_MAX_LINE_LENGTH 16
#endif


static uint8_t read_idx = 0;


void USART0_init(uint8_t baud)
{
  switch (baud)
  {
    case BR_115200: USART0.BAUD = (uint16_t)USART0_BAUD_RATE(115200); break;
    case BR_57600:  USART0.BAUD = (uint16_t)USART0_BAUD_RATE(57600); break;
    case BR_38400:  USART0.BAUD = (uint16_t)USART0_BAUD_RATE(38400); break;
    case BR_19200:  USART0.BAUD = (uint16_t)USART0_BAUD_RATE(19200); break;
    default:        USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600);
  }
  // if (baud == BR_115200) USART0.BAUD = (uint16_t)USART0_BAUD_RATE(115200);
  // else if (baud == BR_57600) USART0.BAUD = (uint16_t)USART0_BAUD_RATE(57600);
  // else if (baud == BR_38400) USART0.BAUD = (uint16_t)USART0_BAUD_RATE(38400);
  // else if (baud == BR_19200) USART0.BAUD = (uint16_t)USART0_BAUD_RATE(19200);
  // else USART0.BAUD = (uint16_t)USART0_BAUD_RATE(9600);
  USART0.CTRLB |= USART_TXEN_bm;
  USART0.CTRLB |= USART_RXEN_bm;
  PORTB.DIR |=  PIN2_bm; // Output Tx
  PORTB.DIR &= ~PIN3_bm; // Input  Rx

#ifdef USART_USE_RX_INTERRUPT
  uart_rx_idx = 0;
  USART0.CTRLA |= USART_RXCIE_bm;
#endif /* USART_USE_RX_INTERRUPT */
}

uint8_t USART0_read()
{
  while (!(USART0.STATUS & USART_RXCIF_bm)) {;}
  return USART0.RXDATAL;
}

void USART0_sendChar(char c)
{
  while (!(USART0.STATUS & USART_DREIF_bm)) {;}
  USART0.TXDATAL = c;
}

void USART0_sendString(char* str)
{
  while (*str != 0)
  {
    USART0_sendChar(*str);
    str++;
  }
}

char* read_UART_buffer_line(const char eol)
{
  static char linebuffer[UART_MAX_LINE_LENGTH];
  static uint8_t read_idx = 0;
  static uint8_t line_idx = 0;
  char c;
  while (read_idx != uart_rx_idx)
  {
    read_idx = (read_idx+1) & (UART_RX_BUFFER_LEN-1);
    c = uart_rx_buffer[read_idx];
    linebuffer[line_idx++] = c;
    if (read_idx == UART_RX_BUFFER_LEN)
      read_idx = 0;
    if (line_idx == UART_MAX_LINE_LENGTH - 1)
      line_idx = UART_MAX_LINE_LENGTH - 2;
    if (c == eol)
    {
      linebuffer[line_idx] = 0;
      line_idx = 0;
      return linebuffer;
    }
  }
  return 0;
}


uint8_t read_UART_buffer(uint8_t* buffer, uint8_t max_nr_bytes)
{
  uint8_t line_idx = 0;
  char c;
  while (read_idx != uart_rx_idx)
  {
    read_idx = (read_idx + 1) & (UART_RX_BUFFER_LEN - 1);
    c = uart_rx_buffer[read_idx];
    buffer[line_idx++] = c;
    if (read_idx == UART_RX_BUFFER_LEN)
      read_idx = 0;
    if (line_idx == max_nr_bytes)
      break;
  }
  return line_idx;
}


uint8_t nr_bytes_in_buffer(void)
{
  if (read_idx > uart_rx_idx)
    return (uart_rx_idx + UART_RX_BUFFER_LEN - read_idx);
  else
    return (uart_rx_idx - read_idx);
}

#endif // USE_UART
