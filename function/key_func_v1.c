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
 * key_func_v1.c
 * 
 * implementation of functions to handle single key inputs
 *
 * History
 * Date       Change
 * 09.04.2020 Initial version
 * 29.08.2024 Review and Cleanup by Kueper
 * 
 */


//--------------------------------------------------------------------
// Configuration for key input functions
// Place needed defines into config.h
//--------------------------------------------------------------------
//#define _ENABLE_SIMPLE_KEY_INPUT
//#define TASTER1_PORT_IN PORTA_IN
//#define TASTER1_PIN 1
//#define TASTER2_PORT_IN PORTA_IN
//#define TASTER2_PIN 2
//#define TASTER3_PORT_IN PORTA_IN
//#define TASTER3_PIN 3
//#define TASTER4_PORT_IN PORTA_IN
//#define TASTER4_PIN 4
//#define TASTER5_PORT_IN PORTA_IN
//#define TASTER5_PIN 5

#include "config.h"
#include "key_func.h"

/***************************************************************
 * User input funktions
 ***************************************************************
 */

volatile uint8_t key_event_pressed = 0;
volatile uint8_t key_event_released = 0;
volatile uint8_t key_event_key = 0;
volatile uint8_t key_down = 0;
volatile uint8_t key_pressed_cnt;

void cyclic_check_taster(void)
{
  static uint8_t cnt;
  static uint8_t last_key_down = 0;
  uint16_t in_tmp;
  key_down = 0;
  in_tmp = TASTER1_PORT_IN;
  if (0 == (in_tmp & _BV(TASTER1_PIN)))
    key_down = 1;
#ifdef TASTER2_PORT_IN
  in_tmp = TASTER2_PORT_IN;
  if (0 == (in_tmp & _BV(TASTER2_PIN)))
    key_down = 2;
#endif
#ifdef TASTER3_PORT_IN
  in_tmp = TASTER2_PORT_IN;
  if (0 == (in_tmp & _BV(TASTER2_PIN)))
    key_down = 3;
#endif
#ifdef TASTER4_PORT_IN
  in_tmp = TASTER2_PORT_IN;
  if (0 == (in_tmp & _BV(TASTER2_PIN)))
    key_down = 4;
#endif
#ifdef TASTER5_PORT_IN
  in_tmp = TASTER5_PORT_IN;
  if (0 == (in_tmp & _BV(TASTER5_PIN)))
    key_down = 5;
#endif

  if (0 != key_down)
  {
    if (cnt < 250)
      cnt++;
    if (last_key_down == 0)
    {
      key_event_pressed = 1;
      key_event_key = key_down;
      cnt = 0;
    }
  }
  else
  {
    if (last_key_down != 0)
    {
      key_event_released = 1;
      key_pressed_cnt = cnt;
    }
  }
  last_key_down = key_down;
}

void key_event_done(void)
{
  key_event_pressed = 0;
  key_event_released = 0;
}
