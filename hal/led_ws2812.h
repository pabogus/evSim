#ifndef _LED_WS2812_H_
#define _LED_WS2812_H_
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
 * led_ws2812.h
 * Provides functions to send a data stream to ws2812 compatible LEDs
 *
 * History
 * 26.09.2018 17:08:31 Initial version
 */

/*
avrdude -c xplainedmini_updi -F -C $ATMEL_DIR/avrdude.conf -p t1614 -U flash:v:main.hex
*/

#include <avr/io.h>
#include "color_func.h"


/* H: The hue as an angle H on the color wheel (e.g., 0° = Red, 120° = Green, 240° = Blue)
 * Since a byte is used for this, the angle must be multiplied by 255/360 = 0.708333..
 */
#define H_VAL_RED            0
#define H_VAL_ORANGE        10
#define H_VAL_SAFFRON       25
#define H_VAL_YELLOW        42
#define H_VAL_YELLOW_GREEN  64
#define H_VAL_LIME          74
#define H_VAL_GREEN         85
#define H_VAL_BLUE_GREEN   106
#define H_VAL_CYAN         127
#define H_VAL_GREEN_BLUE   149
#define H_VAL_BLUE         170
#define H_VAL_INDIGO       181
#define H_VAL_VIOLET       191
#define H_VAL_MAGENTA      212
#define H_VAL_BLUE_RED     234

typedef union
{
  uint8_t data[3];
  ColorRGB_t pixel;
} pixel_array_t;

/**
 * Send a stream of RGB data to the WS2812 LED strip using the specified pin.
 * 
 * @param data RGB data to send
 * @param len Length of the RGB data buffer
 * @param pin_number GPIO pin to use
 */
void send_rgb_stream(uint8_t* data, uint16_t len, uint8_t pin_number);

#endif // _LED_WS2812_H_
