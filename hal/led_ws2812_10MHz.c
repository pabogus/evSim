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
 * led_ws2812_10MHz.c
 * Provides bit banging functions to drive a chain of ws2812 compatible LEDs
 * To achive the protocol the stream output function is written in
 * assembly language and expects the AVR device running with 10MHz.
 *
 * History
 * 26.09.2018 17:08:31 Initial version
 */

/*
avrdude -c xplainedmini_updi -F -C $ATMEL_DIR/avrdude.conf -p t1614 -U flash:v:main.hex
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "config.h"
#include "led_ws2812.h"


/*
//--------------------------------------------------------------------
// configure led_ws2812
//--------------------------------------------------------------------
*/
//#define LED_VPORT_OUT VPORTB_OUT
//#define LED_STIPE_DATA_PIN_NR
//#define USE_FAST_HSV2RGB_FUNCTION
//#define LED_DIM_NR_SHIFTS; /* 1=1/2 2=1/4 3=1/8


/*
 * 1= __________________  0.6us
 *     0.7us           |______________
 *
 * 0= _________  0.8us
 *     0.35us |______________________
 *
 */
void send_rgb_stream_10MHz(uint8_t* data, uint16_t len, uint8_t pin_number)
{
  uint8_t byte;
  volatile uint8_t cnt;
  uint8_t out_lo, out_hi;

  out_lo = LED_VPORT_OUT & ~_BV(pin_number);
  out_hi = out_lo | _BV(pin_number);
  byte = *data++;
  cnt = 6;
  do
  {
    asm volatile (
"SEND_BIT_%=: "  "out   %2, %3" "\n\t" /* Set output to High */
                 "sbrc %0, 7"   "\n\t" /* Skip if bit clear, Test if Bit 7 = 0*/
                 "rjmp SEND_ONE_%="  "\n\t"
"SEND_ZERO_%=: " "nop"     "\n\t"
                 "out  %2, %4"  "\n\t" /* Set output to Low after 0,35us. Wait 0 Low time, including jump to loop start */
                 "rjmp .+0"     "\n\t"
                 "nop"          "\n\t"
                 "add  %0,%0"   "\n\t"
                 "subi %1,1"    "\n\t"
                 "brcc SEND_BIT_%="  "\n\t"
                 "rjmp SEND_END_%="  "\n\t"
"SEND_ONE_%=: "  "nop"          "\n\t"
                 "rjmp .+0"     "\n\t"
                 "add  %0,%0"   "\n\t"
                 "out  %2, %4"  "\n\t" /* Set output to Low after 0,7us. Wait 0 Low time, including jump to loop start */
                 "nop"          "\n\t"
                 "nop"          "\n\t"
                 "subi %1,1"    "\n\t"
                 "brcc SEND_BIT_%="  "\n\t"
                 "rjmp SEND_END_%="  "\n\t"
"SEND_END_%=: "  "out  %2, %3"  "\n\t"
    :: "r" (byte), "r" (cnt), "I" (_SFR_IO_ADDR(LED_VPORT_OUT)), "r" (out_hi), "r" (out_lo) );

    /* send the last bit, the output is already HIGH */
    if (byte & 0x80) {
      /* wait 1 HIGH time, also prepare for the next byte */
      asm volatile ("nop" "\n\t");
      byte = *data++;
      len--;
      cnt = 6;
      LED_VPORT_OUT = out_lo;
      /* wait 1 LOW time, which is just long enough to start the loop for the next byte. */
    }
    else
    {
      /* wait 0 HIGH time */
//        asm volatile ("nop"      "\n\t");
      LED_VPORT_OUT = out_lo;
      /* wait 1 LOW time, also prepare for the next byte and jump back to the loop for the next byte. */
//        asm volatile ("nop"      "\n\t");
      byte = *data++;
      len--;
      cnt = 6;
    }

  } while (len != 0);
  _delay_us(100);
}

void send_rgb_stream(uint8_t* data, uint16_t len, uint8_t pin_number)
{
  uint8_t tmp;
  if (len == 0)
    return;
  /* attiny1614 runs with 20Mhz internal Clock
   * with default devider by 6 => 3,33 MHz. */
  tmp = CLKCTRL_MCLKCTRLB;
  CPU_CCP = 0xD8;
  CLKCTRL_MCLKCTRLB = 0x1; /* divider 2 Clock = 10MHz => one Tick = 100ns*/
  cli();
  send_rgb_stream_10MHz(data, len, pin_number);
  sei();
  CPU_CCP = 0xD8;
  CLKCTRL_MCLKCTRLB = tmp; /* Reset clock to original value*/
}
