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
 * oled1306.c
 *
 * Implementation of functions to use a graphical OLED display
 * based on ssd1306 and sh1106
 * Infos to set up the display taken from chatgpt.com and 
 * https://www.mikrocontroller.net/topic/415651#new
 *
 * History
 * 27.05.2023 pabogus initial version crated
 * xx.06.2023 pabogus functions to display bitmap data
 * xx.02.2024 pabogus functions to support a vertical display usage added
 *                    draft handling of font data with last character info added
 *
 */

#include "oled1306.h"
#include "font6x8.h"
#include "font8x16.h"
#include "I2C_func.h"


/*
//--------------------------------------------------------------------
// configure oled functions
//--------------------------------------------------------------------
*/
/* select ONE display type */
//#define SH1106
//#define SH1106_OFFSET1
//#define SSD1306
//#define SSD1306_64x48

/* select display functions */
//#define USE_OLED_GRAPHIC_SUPPORT
//#define USE_OLED_VERTICAL
//#define OLED_FLIP_HORIZONTAL

/* select basic fonts */
//#define USE_OLED_FONT_6x8
//#define USE_OLED_FONT_8x16

#ifdef SH1106_OFFSET1
  #define D_OFF 0x01 // Set display offset. 00 = no offset
  #define SH1106
#else
  #define D_OFF 0x00 // Set display offset. 00 = no offset
#endif



/*
 * local function declaration
 */
void _oled_set_pixel(uint8_t x, uint8_t y);
void _oled_clear_pixel(uint8_t x, uint8_t y);

const uint8_t noFont[] PROGMEM = {
    0x00, 0x02, 0x01, 0x51, 0x09, 0x06, // just space as ?
};

#if defined USE_OLED_FONT_6x8
const uint8_t* pattern_base = ssd1306oled_font6x8data;
uint8_t  pattern_size_bytes =  6;
uint8_t  pattern_width_pixel = 6;
uint8_t  pattern_height_bytes = 1;
uint8_t  pattern_first_c = 32;
#elif defined USE_OLED_FONT_8x16
const uint8_t* pattern_base = ssd1306xled_font8x16data;
uint8_t  pattern_size_bytes = 16;
uint8_t  pattern_width_pixel = 8;
uint8_t  pattern_height_bytes = 2;
uint8_t  pattern_first_c = 32;
#else
const uint8_t* pattern_base =  noFont;
uint8_t  pattern_size_bytes =  6;
uint8_t  pattern_width_pixel = 6;
uint8_t  pattern_height_bytes = 1;
uint8_t  pattern_first_c = 32;
#endif



/******************************************************
 * Base functions
 *****************************************************
 */
static uint8_t curr_page, curr_col;

/* Init sequence based on chatgpt.com with own modifications
 */
const uint8_t ssd1306_init_sequence[] = {
    0xAE,             // Display OFF (sleep mode)
    0xD5, 0x80,       // Set display clock divide ratio/oscillator frequency
    0xA8, 0x3F,       // Set multiplex ratio (1 to 64)
    0xD3, D_OFF,      // Set display offset
    0x40,             // Set display start line to 0
    0x8D, 0x14,       // Enable charge pump
    0x20, 0x00,       // Memory addressing mode: horizontal
#ifdef OLED_FLIP_HORIZONTAL
    0xA1,             // Segment re-map (column address 127 mapped to SEG0)
    0xC8,             // COM output scan direction remapped (scan from COM[N-1] to COM0)
#else
    0xA0,             // Segment re-map (column address 127 mapped to SEG0)
    0xC0,             // COM output scan direction remapped (scan from COM[N-1] to COM0)
#endif
    0xDA, 0x12,       // Set COM pins hardware configuration
    0x81, 0xCF,       // Set contrast control
    0xD9, 0xF1,       // Set pre-charge period
    0xDB, 0x40,       // Set V_COMH deselect level
    0xA4,             // Disable entire display ON (resume to RAM content display)
    0xA6,             // Set normal display (not inverted)
    0xAF              // Display ON
};

void oled_command(uint8_t cmd)
{
  uint8_t data[2];
  data[0] = 0x00;	// 0x00 for command, 0x40 for data
  data[1] = cmd;
  I2C_write_buffer(OLED_I2C_ADDR, data, 2);
}

void oled_data(uint8_t byte)
{
  uint8_t data[2];
  data[0] = 0x40;	// 0x00 for command, 0x40 for data
  data[1] = byte;
  I2C_write_buffer(OLED_I2C_ADDR, data, 2);
}

void oled_init(uint8_t dispAttr)
{
  for (uint8_t i = 0; i < sizeof(ssd1306_init_sequence); i++)
    oled_command(pgm_read_byte(&ssd1306_init_sequence[i]));
  oled_command(dispAttr);
  oled_clrscr();
}

#ifdef SSD1306_64x48
void oled_gotoPageCol(uint8_t page, uint8_t col)
{
  uint8_t data[10];
  uint8_t idx=0;
  if (col > 127 || page > 5)
    return; // Out of display
  data[idx++] = 0x00;	// 0x00 for command, 0x40 for data
  // Set column address range
  data[idx++] = 0x21;
  data[idx++] = 32 + col;
  data[idx++] = 95;
  // Set page address range
  data[idx++] = 0x22;
  data[idx++] = 2 + page;
  data[idx++] = 7;
  I2C_write_buffer(OLED_I2C_ADDR, data, idx);
  curr_page = page;
  curr_col = col;
}

#else // SSD1306_64x48

void oled_gotoPageCol(uint8_t page, uint8_t col)
{
  uint8_t data[10];
  uint8_t idx = 0;
  if (col > 127 || page > 7)
    return; // Out of display
  data[idx++] = 0x00;	       // 0x00 for command, 0x40 for data
  data[idx++] = 0xb0 + page; // Set page start to y
  data[idx++] = 0x21;	       // Set column start to x
#if defined SSD1306
  data[idx++] = col;
#elif defined SH1106
  data[idx++] = 0x00 +  ((2+col) & 0x0f);          // lowbyte
  data[idx++] = 0x10 + (((2+col) & 0xf0) >> 4);   // highbyte
#endif
  data[idx++] = 0x7f;	       // Set column end to 127
  I2C_write_buffer(OLED_I2C_ADDR, data, idx);
  curr_page = page,
  curr_col = col;
}
#endif //SSD1306_64x48

uint8_t oled_get_curr_page(void)
{
  return curr_page;
}

uint8_t oled_get_curr_col(void)
{
  return curr_col;
}

void oled_clrpage(uint8_t page)
{
  oled_gotoPageCol(page, 0);
  if (I2C_start(OLED_I2C_ADDR, I2C_WRITE) != I2C_OK)
    return;
  I2C_write(0x40);	// 0x00 for command, 0x40 for data
  // Write zeros into page ram
  for (uint8_t i = 0; i < 128; i++)
    I2C_write(0);
  I2C_stop();
  oled_home();
}


void oled_clrscr(void)
{
  oled_clrpage(0);
  oled_clrpage(1);
  oled_clrpage(2);
  oled_clrpage(3);
  oled_clrpage(4);
  oled_clrpage(5);
  oled_clrpage(6);
  oled_clrpage(7);
  oled_gotoPageCol(0, 0);
}

void oled_home(void)
{
  oled_gotoPageCol(0, 0);
}



/******************************************************
 * Display command functions
 *****************************************************
 */

void oled_invert(uint8_t invert)
{
  if (invert)
    oled_command(0xA7); // Set display inverted mode
  else
    oled_command(0xA6); // Set display normal mode
}

void oled_rotation(uint8_t rot)
{
  if (OLED_ROT_TOP == rot)
  {
    oled_command(0xA1);
    oled_command(0xC8);
  }
  if (OLED_ROT_BOT == rot)
  {
    oled_command(0xA0);
    oled_command(0xC0);
  }
}

void oled_set_contrast(uint8_t contrast)
{
  uint8_t data[3];
  data[0] = 0x00;	    // 0x00 for command, 0x40 for data
  data[1] = 0x81;	    // Set display contrast
  data[2] = contrast; // to contrast
  I2C_write_buffer(OLED_I2C_ADDR, data, 3);
}



/******************************************************
 * Common Graphic output functions
 *****************************************************
 */
void oled_set_font(const uint8_t* font_data)
{
  uint8_t type = pgm_read_byte(&font_data[0]);
  uint8_t width = pgm_read_byte(&font_data[2]);
  pattern_width_pixel = pgm_read_byte(&font_data[1]);
  pattern_first_c = pgm_read_byte(&font_data[3]);

  if (width < 9)
    pattern_height_bytes = 1;
  else if (width < 17)
    pattern_height_bytes = 2;
  pattern_size_bytes = pattern_width_pixel * pattern_height_bytes;
  if (type == FONT_TYPE_FIXED_WIDTH_LAST_INFO)
  {
    pattern_base = &font_data[5];
  }
  else
  {
    pattern_base = &font_data[4];
  }
}



/******************************************************
 * Graphic output functions normal orientation
 *****************************************************
 */
void oled_gotoxy(uint8_t x, uint8_t y)
{
  x = x * pattern_width_pixel; // One char with width according to font
  y = y * pattern_height_bytes;
  oled_gotoPageCol(y, x);
}

void oled_put_bitmap_p(const uint8_t* data, uint8_t width, uint8_t height)
{
  while (height--)
  {
    if (I2C_start(OLED_I2C_ADDR, I2C_WRITE) != I2C_OK)
      return;
    I2C_write(0x40);	// 0x00 for command, 0x40 for data
    for (uint8_t i = 0; i < width; i++)
      I2C_write(pgm_read_byte(data++));	// Print font to ram, print 6 columns
    I2C_stop();
    oled_gotoPageCol(curr_page + 1, curr_col);
  }
}

void oled_put_bitmap(const uint8_t* data, uint8_t width, uint8_t height)
{
  while (height--)
  {
    if (I2C_start(OLED_I2C_ADDR, I2C_WRITE) != I2C_OK)
      return;
    I2C_write(0x40);	// 0x00 for command, 0x40 for data
    for (uint8_t i = 0; i < width; i++)
      I2C_write(*data++);	// print font to ram, print 6 columns
    I2C_stop();
    oled_gotoPageCol(curr_page + 1, curr_col);
  }
}

void oled_put_frame_p(const framesDefinition* frames, uint8_t frameNr)
{
  uint8_t** bitmap_Array = (uint8_t**)frames->data;
  const uint8_t* frame;
  if (frames->type == BMP_TYPE_PICTURE)
    frame = frames->data;
  else
    frame = bitmap_Array[frameNr];
  oled_put_bitmap_p(frame, frames->nrPixlX, frames->nrPixlY / 8);
}

void oled_putc(char c)
{
  uint8_t tmp_page = curr_page; ;
  uint8_t tmp_height = pattern_height_bytes;

  if (c == 0x0A)
  {
    oled_gotoPageCol(tmp_page + tmp_height, 0);
    return;
  }

  if ((c > 127) || (c < 32))
    return;
  c -= 32;

  const uint8_t* pattern = &pattern_base[pattern_size_bytes * c];
  oled_put_bitmap_p(pattern, pattern_width_pixel, tmp_height);
  oled_gotoPageCol(tmp_page, curr_col + pattern_width_pixel);
}


void oled_puts(const char* s)
{
  while (*s)
    oled_putc(*s++);
}

void oled_puts_p(const char* progmem_s)
{
  register uint8_t c;
  while ((c = pgm_read_byte(progmem_s++)))
    oled_putc(c);
}



/******************************************************
 * Graphic output functions vertical orientation
 *****************************************************
 */
#ifdef USE_OLED_VERTICAL
void oled_gotoxy_v(uint8_t x, uint8_t y)
{
  uint8_t col,page;
  col = 128 - (y * pattern_height_bytes * 8) - 8;
  page = x * pattern_width_pixel / 8;
  oled_gotoPageCol(page, col);
}

void oled_put_bitmap_v_p(const uint8_t* data, uint8_t width, uint8_t height)
{
  uint8_t in[8];
  uint8_t out[8];
  const uint8_t* pattern;

  for (uint8_t w = 0; w < width/8; w++)
  {
    for (uint8_t h = 0; h < height; h++)
    {
      pattern = &data[h*width + w*8];
      for (uint8_t i = 0; i < 8; i++)
        in[i] = pgm_read_byte(pattern++);
      uint8_t bit = 0x80;
      for (uint8_t n = 0; n < 8; n++)
      {
        out[n] = 0;
        if (in[0] & bit) out[n] = out[n] | 0x01;
        if (in[1] & bit) out[n] = out[n] | 0x02;
        if (in[2] & bit) out[n] = out[n] | 0x04;
        if (in[3] & bit) out[n] = out[n] | 0x08;
        if (in[4] & bit) out[n] = out[n] | 0x10;
        if (in[5] & bit) out[n] = out[n] | 0x20;
        if (in[6] & bit) out[n] = out[n] | 0x40;
        if (in[7] & bit) out[n] = out[n] | 0x80;
        bit = bit >> 1;
      }
      if (I2C_start(OLED_I2C_ADDR, I2C_WRITE) != I2C_OK)
        return;
      I2C_write(0x40);	// 0x00 for command, 0x40 for data
      for (uint8_t i = 0; i < 8; i++)
        I2C_write(out[i]);	// print font to ram, print 6 columns
      I2C_stop();
      oled_gotoPageCol(curr_page, curr_col - 8);
    }
    oled_gotoPageCol(curr_page + 1, curr_col + height*8);
  }
}

void oled_put_bitmap_v(const uint8_t* data, uint8_t width, uint8_t height)
{
  uint8_t in[8];
  uint8_t out[8];
  const uint8_t* pattern;
  uint8_t base_col;

  base_col = curr_col;
  for (uint8_t w = 0; w < width/8; w++)
  {
    for (uint8_t h = 0; h < height; h++)
    {
      pattern = &data[h*width + w*8];
      for (uint8_t i = 0; i < 8; i++)
        in[i] = *pattern++;
      uint8_t bit = 0x80;
      for (uint8_t n = 0; n < 8; n++)
      {
        out[n] = 0;
        if (in[0] & bit) out[n] = out[n] | 0x01;
        if (in[1] & bit) out[n] = out[n] | 0x02;
        if (in[2] & bit) out[n] = out[n] | 0x04;
        if (in[3] & bit) out[n] = out[n] | 0x08;
        if (in[4] & bit) out[n] = out[n] | 0x10;
        if (in[5] & bit) out[n] = out[n] | 0x20;
        if (in[6] & bit) out[n] = out[n] | 0x40;
        if (in[7] & bit) out[n] = out[n] | 0x80;
        bit = bit >> 1;
      }
      if (I2C_start(OLED_I2C_ADDR, I2C_WRITE) != I2C_OK)
        return;
      I2C_write(0x40);	// 0x00 for command, 0x40 for data
      for (uint8_t i = 0; i < 8; i++)
      {
        I2C_write(out[i]);	// print font to ram, print 6 columns
      }
      I2C_stop();
      oled_gotoPageCol(curr_page, base_col - 8 + h*8);
    }
  oled_gotoPageCol(curr_page + 1, curr_col);
  }
}

void oled_putc_v(char c)
{
  uint8_t tmp_page = curr_page;
  uint8_t tmp_col  = curr_col;
  uint8_t tmp_height = pattern_height_bytes;

  if (c == 0x0A)
  {
    oled_gotoPageCol(tmp_page + tmp_height, 0);
    return;
  }

  if ((c > 127) || (c < 32))
    return;
  c -= 32;

  const uint8_t* pattern = &pattern_base[pattern_size_bytes * c];
  oled_put_bitmap_v_p(pattern, pattern_width_pixel, tmp_height);
  oled_gotoPageCol(tmp_page + pattern_width_pixel/8, tmp_col);
}

void oled_puts_v(const char* s)
{
  while (*s)
    oled_putc_v(*s++);
}
#endif // USE_OLED_VERTICAL



/******************************************************
 * Gfx functions using screen buffer
 *****************************************************
 */
#ifdef USE_OLED_GRAPHIC_SUPPORT
uint8_t screenBuffer[128 * 8];
/*
 * color of 16bit
 * |   5bit red   |    6bit green   |   5bit blue  |
 * |15|14|13|12|11|10|09|08|07|06|05|04|03|02|01|00|
 */
void oled_draw_pixel(uint8_t x, uint8_t y, uint16_t color)
{
  if (color != 0)
    _oled_set_pixel(x, y);
  else
    _oled_clear_pixel(x, y);
}

void _oled_set_pixel(uint8_t x, uint8_t y)
{
  uint8_t page = y >> 3;
  uint8_t pixelIdx = y & 0x07;
  uint8_t pattern = 1 << pixelIdx;
  uint16_t idx = x + page*128;
  screenBuffer[idx] = screenBuffer[idx] | pattern;
}

void _oled_clear_pixel(uint8_t x, uint8_t y)
{
  uint8_t page = y >> 3;
  uint8_t pixelIdx = y & 0x07;
  uint8_t pattern = 1 << pixelIdx;
  uint16_t idx = x + page*128;
  screenBuffer[idx] = screenBuffer[idx] & ~pattern;
}

void oled_refresh_screen(void)
{
  uint8_t* framebuffer = screenBuffer;
  oled_home();
#if defined SSD1306
  if (I2C_start(OLED_I2C_ADDR, I2C_WRITE) != I2C_OK)
    return;
  I2C_write(0x40);	// 0x00 for command, 0x40 for data
  for (uint16_t n = 0; n < FRAME_BUFFER_SIZE; n++)
  {
    I2C_write(*framebuffer++); // write value from framebuffer
  }
  I2C_stop();
#elif defined SH1106
    for (uint8_t j = 0; j < 8; j++)
    {
      if (I2C_start(OLED_I2C_ADDR, I2C_WRITE) != I2C_OK)
        return;
      I2C_write(0x40);	// 0x00 for command, 0x40 for data
      for (uint8_t i = 0; i < 128 ; i++)
      {
        I2C_write(*framebuffer++);	//write value from framebuffer
      }
      I2C_stop();
      oled_gotoPageCol(j + 1, 0);
    }
#endif
}

void oled_clear_screen(void)
{
  uint8_t* framebuffer = screenBuffer;
  for (uint16_t n = 0; n < FRAME_BUFFER_SIZE; n++)
  {
    *framebuffer++ = 0;
  }
}

#endif // USE_OLED_GRAPHIC_SUPPORT
