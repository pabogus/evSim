#ifndef RS232_H
#define RS232_H
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
 * rs232.h
 * Provides functions to send and recive data via UART on a Attiny 1 series device
 * Implements a ring buffer and an interrupt routine.

 *
 * History
 */

#define BR_9600    1  // 1 Bit = 104,16666 us => 1Byte ca. 1ms
#define BR_19200   2
#define BR_38400   4
#define BR_57600   6
#define BR_115200 12  // 1 Bit =   8,68055 us !! only possible with Clock >= 5MHz

#define EOL_CHAR_CR 13
#define EOL_CHAR_LF 10


extern char uart_rx_buffer[];


/**
 * Initialize USART0 with the specified baud rate.
 * 
 * @param baud Baud rate
 */
void USART0_init(uint8_t baud);

/**
 * Receive a single byte of data from USART.
 */
uint8_t USART0_read(void);

/**
 * Write a single byte of data to USART.
 */
void USART0_sendChar(char c);

/**
 * Write a 0-terminated string of multiple characters to USART.
 */
void USART0_sendString(char* str);

/**
 * Read a line from the UART. Non-blocking.
 * 
 * @param eol EOL character (End Of Line) that terminates the line
 * @returns The read string. This static buffer will be overriden 
 *  each time the function gets called.
 *  Returns NULL if the end of the current UART stream was reached
 *  before an EOL character was found.
 */
char* read_UART_buffer_line(const char eol);

/**
 * Read from UART into a buffer. Non-blocking.
 * 
 * @param buffer The destination buffer to read into
 * @param max_nr_bytes Maximum number of bytes to read
 * @returns Actual number of bytes read
 */
uint8_t read_UART_buffer(uint8_t* buffer, uint8_t max_nr_bytes);

/**
 * Return the number of bytes that are currently ready in the UART buffer.
 */
uint8_t nr_bytes_in_buffer(void);

#endif /* RS232_H */
