#ifndef _input_functions_h
#define _input_functions_h
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


/*==============================================================*/
/* User Configuration for input_functions.c
 */

/***************************************************************
 * The following defines need to be set for simple key input
 * Each key is connected to one pin and switches to Gnd.
 * Only the defines are needed for which a key is connected
 *
 * set _ENABLE_SIMPLE_KEY_INPUT if this function is needed
 **************************************************************
 */
//#define _ENABLE_SIMPLE_KEY_INPUT
//#define TASTER1_PORT_IN PORTA_IN
//#define TASTER1_PIN 4
//#define TASTER2_PORT_IN PORTB_IN
//#define TASTER2_PIN 5
//#define TASTER3_PORT_IN PORTA_IN
//#define TASTER3_PIN 4
//#define TASTER4_PORT_IN PORTB_IN
//#define TASTER4_PIN 5
//#define TASTER4_PORT_IN PORTB_IN
//#define TASTER4_PIN 5


extern volatile uint8_t key_event_pressed;
extern volatile uint8_t key_event_released;
extern volatile uint8_t key_event_key;
extern volatile uint8_t key_down;
extern volatile uint8_t key_pressed_cnt;


void init_taster(void);
void cyclic_check_taster(void);
void key_event_done(void);


#endif /*_input_functions_h*/
