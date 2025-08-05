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
 * oled1306.h
 *
 * Implementation of functions to use a graphical OLED display
 * based on ssd1306 and sh1106 
 *
 * History
 * 2023.05.27 pabogus initial version crated
 * 2023.06.xx pabogus functions to display bitmap data
 * 2024.02.xx pabogus functions to support a vertical display usage added
 *                    draft handling of font data with last character info added
 *
 */

#ifndef OLED_H
#define OLED_H

#include "config.h"
#include "bitmap_frame.h"
#include <inttypes.h>


//#define SH1106	// or SSD1306, check datasheet of your display
//#define SSD1306

#define OLED_I2C_ADDR	0x3c	// refer lcd-manual, 0x78 for 8-bit-adressmode, 0x3c for 7-bit-adressmode

#define OLED_DISP_OFF	0xAE
#define OLED_DISP_ON	0xAF

#define OLED_ROT_TOP	0x00
#define OLED_ROT_BOT	0x01


#ifdef SSD1306_64x48
  #define SCREEN_WIDTH_PXL  64
  #define SCREEN_HIGHT_PXL  48
  #define SCREEN_HIGHT_PAGE  6
#else
  #define SCREEN_WIDTH_PXL 128
  #define SCREEN_HIGHT_PXL  64
  #define SCREEN_HIGHT_PAGE  8
#endif

#define FRAME_BUFFER_SIZE (SCREEN_WIDTH_PXL*8)

#define FONT_TYPE_FIXED_WIDTH             0
#define FONT_TYPE_FIXED_WIDTH_LAST_INFO   1

extern const uint8_t ssd1306xled_font6x8data[];
extern const uint8_t ssd1306xled_font8x16data[];
extern uint8_t screenBuffer[];

void oled_init(uint8_t dispAttr);
uint8_t oled_get_curr_page(void);
uint8_t oled_get_curr_col(void);
void oled_set_font(const uint8_t* font_data);
void oled_home(void);
void oled_put_bitmap_p(const uint8_t* data, uint8_t width, uint8_t height);
void oled_put_bitmap(const uint8_t* data, uint8_t width, uint8_t height);
void oled_put_frame_p(const framesDefinition* frames, uint8_t frameNr);
void oled_gotoPageCol(uint8_t page, uint8_t col);
void oled_draw_pixel(uint8_t x, uint8_t y, uint16_t color);
void oled_command(uint8_t cmd);				    // transmit command to display
void oled_data(uint8_t data);				      // transmit data to display
void oled_clrscr(void);						        // clear screen
void oled_clrpage(uint8_t page);    		  // clear specific page
void oled_invert(uint8_t invert);			    // invert display
void oled_rotation(uint8_t rot);          // flip screen
void oled_set_contrast(uint8_t contrast);	// set contrast for display

//horizontal screen usage
void oled_gotoxy(uint8_t x, uint8_t y);		// set curser at pos x, y. x means character, y means line (page, refer lcd manual)
void oled_putc(char c);						        // print character on screen
void oled_puts(const char* s);				    // print string, \n-terminated, from ram on screen
void oled_puts_p(const char* progmem_s);	// print string from flash on screen

//vertical screen usage
void oled_gotoxy_v(uint8_t x, uint8_t y);	// set curser at pos x, y. x means character, y means line (page, refer lcd manual)
void oled_putc_v(char c);					        // print character on screen
void oled_puts_v(const char* s);			    // print string, \n-terminated, from ram on screen
void oled_put_bitmap_v(const uint8_t* data, uint8_t width, uint8_t height);
void oled_put_bitmap_v_p(const uint8_t* data, uint8_t width, uint8_t height);

//GFX functions, using a frame buffer
void oled_pixel_color(uint8_t x, uint8_t y, uint16_t color);
void oled_refresh_screen(void);
void oled_clear_screen(void);

#endif /*  OLED_H  */
