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
 * evSimDisplay.c
 * 
 * Provides functions to display satus information
 * about the ev-simulator on an OLED display.
 *
 * History
 * Version  Date       Change
 *    1     30.12.2023 Initial version of the functions inside main.c
 *    2     28.01.2024 Functions moved to own file.
 *                     Add functions to support different screen orientations
 *    3     29.08.2024 Code Review and Cleanup by Kueper
 */


#include <string.h>
#include "config.h"
#include "support_lib.h"
#include "oled1306.h"
#include "modbusRtu_func.h"
#include "evSim.h"

extern const uint8_t ssd1306xled_font6x8data[];
extern const uint8_t ssd1306xled_font8x16data[];

/**************************************************************
 * HW Setup configuration
 * ***********************************************************/



/**************************************************************
 * Display functions
 * ***********************************************************/
#define UPDATE_STATE_A (1<<0)
#define UPDATE_STATE_B (1<<1)
#define UPDATE_PWM_A   (1<<2)
#define UPDATE_PWM_B   (1<<3)

#define SELECT_A 1
#define SELECT_B 3

#define BAR_WIDTH 51


uint8_t horizontal = 1;
extern char string[];


void display_status(uint8_t select)
{
  uint8_t iso;
  uint8_t mode;
  uint8_t line;

  if (SELECT_A == select)
  {
    iso  = analogOutput[AO_REG_OFFSET_ISO_A];
    mode = analogOutput[AO_REG_OFFSET_MOD_A];
    line = 0;
  }
  else
  {
    iso  = analogOutput[AO_REG_OFFSET_ISO_B];
    mode = analogOutput[AO_REG_OFFSET_MOD_B];
    line = 2;
  }

  strcpy(string, "ISO:");
  switch (iso)
  {
    case REGVALUE_ISOSTATE_A:  strcat(string, "A "); break;
    case REGVALUE_ISOSTATE_B:
    case REGVALUE_ISOSTATE_1B: strcat(string, "B "); break;
    case REGVALUE_ISOSTATE_C:  strcat(string, "C "); break;
    default: strcat(string, "I "); break;
  }
  strcat(string, " MOD:");
  switch (mode)
  {
    case REGVAL_MODE_MAX: strcat(string, "Max_A "); break;
    case REGVAL_MODE_13A: strcat(string, "13_A  "); break;
    case REGVAL_MODE_ERR: strcat(string, "ERR   "); break;
    default: strcat(string, "I "); break;
  }
  oled_gotoxy(0, line); oled_puts(string);
}


void display_status_v(uint8_t select)
{
  uint8_t iso;
  uint8_t mode;
  uint8_t line;

  if (SELECT_A == select)
  {
    iso  = analogOutput[AO_REG_OFFSET_ISO_A];
    mode = analogOutput[AO_REG_OFFSET_MOD_A];
    line = 0;
  }
  else
  {
    iso  = analogOutput[AO_REG_OFFSET_ISO_B];
    mode = analogOutput[AO_REG_OFFSET_MOD_B];
    line = 4;
  }

  strcpy(string, "ISO:");
  switch (iso)
  {
    case REGVALUE_ISOSTATE_A:  strcat(string, "A "); break;
    case REGVALUE_ISOSTATE_B:
    case REGVALUE_ISOSTATE_1B: strcat(string, "B "); break;
    case REGVALUE_ISOSTATE_C:  strcat(string, "C "); break;
    default: strcat(string, "I "); break;
  }
  oled_gotoxy_v(0, line); oled_puts_v(string);
  strcpy(string, "MOD:");
  switch (mode)
  {
    case REGVAL_MODE_MAX: strcat(string, "Max "); break;
    case REGVAL_MODE_13A: strcat(string, "13A "); break;
    case REGVAL_MODE_ERR: strcat(string, "ERR "); break;
    default: strcat(string, "-I- "); break;
  }
  oled_gotoxy_v(0, line+1); oled_puts_v(string);
}

uint8_t bar[BAR_WIDTH*2] = {0};
void plot_bar(uint8_t value, uint8_t vertical)
{
  for (uint8_t n = 0; n < BAR_WIDTH; n++)
  {
      if(n < value/2)
      {
        bar[n] = 0xf0;
        bar[n+BAR_WIDTH] = 0x0f;
      }
      else
      {
        bar[n] = 0;
        bar[n+BAR_WIDTH] = 0;
      }
  }
  for (uint8_t n = 4; n < BAR_WIDTH; n += 5)
  {
    bar[n] = 0xfc;
    bar[n+BAR_WIDTH] = 0x3f;
  }
  if (vertical)
    oled_put_bitmap_v(bar, BAR_WIDTH, 2);
  else
    oled_put_bitmap(bar, BAR_WIDTH, 2);
}


void display_dc(uint8_t select)
{
  uint8_t dc,line;
  if (SELECT_A == select)
  {
    dc = get_AI_register_lo(AI_REG_OFFSET_PWM_A);
    line = 1;
  }
  else
  {
    dc = get_AI_register_lo(AI_REG_OFFSET_PWM_B);
    line = 3;
  }
  strcpy(string, "PWM:");
  strcat_uint(string, dc);
  strcat(string, "% ");
  oled_gotoxy(0, line); oled_puts(string);
  plot_bar(dc, 0);
}


void display_dc_v(uint8_t select)
{
  uint8_t dc,line;
  if (SELECT_A == select)
  {
    dc = get_AI_register_lo(AI_REG_OFFSET_PWM_A);
    line = 2;
  }
  else
  {
    dc = get_AI_register_lo(AI_REG_OFFSET_PWM_B);
    line = 6;
  }
  strcpy(string, "PWM:");
  strcat_uint(string, dc);
  strcat(string, "% ");
  oled_gotoxy_v(0, line); oled_puts_v(string);
  oled_gotoxy_v(0, line+1); plot_bar(dc, 1);
}


void update_display(uint8_t update)
{
  if (horizontal && (update & UPDATE_STATE_A))  display_status(SELECT_A);
  if (horizontal && (update & UPDATE_STATE_B))  display_status(SELECT_B);
  if (!horizontal && (update & UPDATE_STATE_A)) display_status_v(SELECT_A);
  if (!horizontal && (update & UPDATE_STATE_B)) display_status_v(SELECT_B);

  if (horizontal && (update & UPDATE_PWM_A))  display_dc(SELECT_A);
  if (horizontal && (update & UPDATE_PWM_B))  display_dc(SELECT_B);
  if (!horizontal && (update & UPDATE_PWM_A)) display_dc_v(SELECT_A);
  if (!horizontal && (update & UPDATE_PWM_B)) display_dc_v(SELECT_B);
}

void update_rotation(void)
{
  uint8_t tmp = get_AO_register_lo(AO_REG_OFFSET_CFG_ROTATION);
  if (tmp == ROTATION_HORIZONTAL || tmp == ROTATION_HORIZONTAL_FLIP)
  {
    horizontal = 1;
  }
  else
  {
    horizontal = 0;
  }
  if (tmp == ROTATION_VERTICAL_FLIP || tmp == ROTATION_HORIZONTAL_FLIP)
  {
    oled_rotation(OLED_ROT_BOT);
  }
  else
  {
    oled_rotation(OLED_ROT_TOP);
  }
  oled_clrscr();
  update_display(0xff);
}
