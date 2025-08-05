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
 * main.c
 *
 * Contains main() and the functions to switch output for relais to realice
 * the evse states signaled via CP and CS as well as functions to measure the 
 * duty cycle of the PWM provided by the outlet controller.
 *
 * History
 * 28.12.2023 Initial version
 *       2024 Review and corrections
 * 02.08.2025 Beatyfiing for upload to GitHub
 * 
 */

/* Notes and infos:

Commandlines to flash used under Linux:
---------------------------------------
avrdude -c xplainedmini_updi -F -C $ATMEL_DIR/avrdude.conf -p t1616 -U flash:v:main.hex
~/.local/bin/pyupdi -c /dev/ttyUSB0 -d tiny826 -f main.hex
pymcuprog write -d t1616 -t uart -u /dev/ttyUSB0 --erase --verify -f main.hex


Commandline examples to use mbpoll to issue modbus RTU messages under Linux
For testing this dongle was used:
<https://www.reichelt.de/de/de/shop/produkt/raspberry_pi_-_usb-rs485-schnittstelle_ch340c-242783>
-----------------------------------------------------
mbpoll -v -m rtu  -a 2 -b 9600 -t 4 /dev/ttyUSB0 0xA

Set ISO State A, B and C
mbpoll -v -m rtu  -a 2  -b 9600 -t 4 -1 /dev/ttyUSB0 0xa
mbpoll -v -m rtu  -a 2  -b 9600 -t 4 -1 /dev/ttyUSB0 0xb
mbpoll -v -m rtu  -a 2  -b 9600 -t 4 -1 /dev/ttyUSB0 0xc

Read ISO State B
mbpoll -v -m rtu  -a 2 -r 3 -b 9600 -t 4 -1 /dev/ttyUSB0

Read all States
mbpoll -v -m rtu  -a 2 -c 4 -b 9600 -t 4 -1 /dev/ttyUSB0

Read PWM and inputs
mbpoll -v -m rtu  -a 2 -c 5 -b 9600 -t 3:hex -1 /dev/ttyUSB0

Set & Read configuration
mbpoll -v -m rtu  -a 2 -r 11 -c 5 -b 9600 -t 4 -1 /dev/ttyUSB0
mbpoll -v -m rtu  -a 2 -r 11  -b 9600 -t 4 /dev/ttyUSB0 48
mbpoll -v -m rtu  -a 2 -r 12  -b 9600 -t 4 /dev/ttyUSB0 3

Read SW infos
mbpoll -v -m rtu  -a 2 -r 11 -c 5 -b 9600 -t 3:hex -1 /dev/ttyUSB0

Read PWM A
mbpoll -v -m rtu  -a 2  -b 9600 -t 3 -1 /dev/ttyUSB0

Read PWM A & PWM P
mbpoll -v -m rtu  -a 2 -c 2  -b 9600 -t 3 -1 /dev/ttyUSB0

Read configuration
mbpoll -v -m rtu  -a 2  -b 9600 -t 4 -1 /dev/ttyUSB0

*/


/* Pinout informationn

Testbuild / breadboard setup: 

             AtTiny1616
                _____
            Vcc-|   |-Gnd
      CS -- PA4-|   |-PA3 --SCLK
    TA_A -- PA5-|   |-PA2 --
    TA_B -- PA6-|   |-PA1 --MOSI
   PWM_A -- PA7-|   |-Reset/UPDI
   PWM_B --´PB5-|   |-PC3 --
         -- PB4-|   |-PC2 --
     RxD -- PB3-|   |-PC1 --LEDs
     TxD -- PB2-|   |-PC0 --RXen
     SDA -- PB1-|   |-PB0 --SCL
                -----

PCB setup: 
             AtTiny1616
                _____
            Vcc-|   |-Gnd
      CS -- PA4-|   |-PA3 --SCLK
    TA_A -- PA5-|   |-PA2 --
    TA_B -- PA6-|   |-PA1 --MOSI
   PWM_A -- PA7-|   |-Reset/UPDI
   PWM_B --´PB5-|   |-PC3 --Fin_A
   Fin_B -- PB4-|   |-PC2 --Aux_FET
     RxD -- PB3-|   |-PC1 --LEDs
     TxD -- PB2-|   |-PC0 --RXen
     SDA -- PB1-|   |-PB0 --SCL
                -----


              TPIC6B595N
                _____
             NC-|   |-NC
            VCC-|   |-Gnd
   MOSI --  Sin-|   |-Sout
        -- Out0-|   |-Out7 --ISO_B_1
        -- Out1-|   |-Out6 --ISO_C_1
        -- Out2-|   |-Out5 --MODE_13_A_1
        -- Out3-|   |-Out4 --MODE_ERROR
VCC/Cap -- /Clr-|   |-SCLK --SCLK
    GND --  /G -|   |-RCLK -CS
            GND-|   |-GND
                -----
*/


#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "config.h"
#include "spi_func.h"
#include "I2C_func.h"
#include "oled1306.h"
#include "key_func.h"
#include "support_lib.h"
#include "rs232.h"
#include "modbusRtu_func.h"
#include "led_ws2812.h"
#include "evSim.h"


/**************************************************************
 * HW Setup configuration
 * ***********************************************************/
#define REL_BIT_ISO_B_1 4
#define REL_BIT_ISO_C_1 5
#define REL_BIT_13_A_1 6
#define REL_BIT_ERR_1 7

#define REL_BIT_ISO_B_2 0
#define REL_BIT_ISO_C_2 1
#define REL_BIT_13_A_2 2
#define REL_BIT_ERR_2 3

#define SERIAL_PORT_CS_PIN 4
#define RELAIS_CS_PORT PORTA_OUT

#define AUX_PORT_PIN 2
#define AUX_PORT PORTC_OUT


#define CFG_DEFAULT_BRIGHTNESS 48
#define LONG_PRESSED_CNT 50


char string[20];


void IOinit(void)
{
  PORTA.PIN7CTRL = PORT_INVEN_bm;    // Invert input

  PORTA.PIN5CTRL = PORT_PULLUPEN_bm; // Enable Pullup for PA5
  PORTA.PIN6CTRL = PORT_PULLUPEN_bm; // Enable Pullup for PA6
}

/**************************************************************
 * Modbus related functions
 * ***********************************************************/
void init_modbus_registers(void)
{
  // Read/Write -> Func 0x06 / 0x16
  analogOutput[AO_REG_OFFSET_ISO_A] = REGVALUE_ISOSTATE_A;
  analogOutput[AO_REG_OFFSET_MOD_A] = REGVAL_MODE_MAX;
  analogOutput[AO_REG_OFFSET_ISO_B] = REGVALUE_ISOSTATE_A;
  analogOutput[AO_REG_OFFSET_MOD_B] = REGVAL_MODE_MAX;

  analogOutput[AO_REG_OFFSET_CFG_BRIGHTNESS] = CFG_DEFAULT_BRIGHTNESS;
  
  // Set the default orientation of the display as defined in config.h
  // For testing the other options can be commented in.
  analogOutput[AO_REG_OFFSET_CFG_ROTATION] = CFG_DEFAULT_ROTATION;
//  analogOutput[AO_REG_OFFSET_CFG_ROTATION] = ROTATION_HORIZONTAL;
//  analogOutput[AO_REG_OFFSET_CFG_ROTATION] = ROTATION_HORIZONTAL_FLIP;
//  analogOutput[AO_REG_OFFSET_CFG_ROTATION] = ROTATION_VERTICAL;
//  analogOutput[AO_REG_OFFSET_CFG_ROTATION] = ROTATION_VERTICAL_FLIP;



  // Read Only  ->
  // TODO set on real build date
  analogInput[AI_REG_OFFSET_SW_VERSION] = 0x0101;
  analogInput[AI_REG_OFFSET_SW_YEAR] = 0x2024;
  analogInput[AI_REG_OFFSET_SW_MONTHDAY] = 0x0116;
}


/**************************************************************
 * Timer PWM/Freq capture setup and handling
 * ***********************************************************/

volatile uint16_t PWM0_high = 0;
volatile uint16_t PWM0_dur = 0;
ISR(TCB0_INT_vect)
{
  PWM0_dur = TCB0.CNT;
  PWM0_high = TCB0.CCMP;
}

volatile uint16_t PWM1_high = 0;
volatile uint16_t PWM1_dur = 0;
ISR(TCB1_INT_vect)
{
  PWM1_dur = TCB1.CNT;
  PWM1_high = TCB1.CCMP;
}

void EVENT_SYSTEM_init(void)
{
  EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;
  EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_PORTA_PIN7_gc;

  EVSYS.ASYNCUSER11 = EVSYS_ASYNCUSER11_ASYNCCH1_gc;
  EVSYS.ASYNCCH1 = EVSYS_ASYNCCH1_PORTB_PIN5_gc;
}

void TCB_init(void)
{
  /* 3,33MHz Prescaler 1 > 0,3 us > 1kHz => 3333 Ticks
     3,33MHz Prescaler 2 > 0,6 us > 1kHz => 1666 Ticks
     10  MHz Prescaler 1 > 0,1 us > 1kHz => 10000 Ticks
     10  MHz Prescaler 2 > 0,2 us > 1kHz => 5000 Ticks
     20  MHz Prescaler 1 > 0,05us > 1kHz => 20000 Ticks
     20  MHz Prescaler 2 > 0,1 us > 1kHz => 10000 Ticks
   */
  TCB0.CTRLB = TCB_CNTMODE_FRQPW_gc; // Input capture pulse width measurement and frequency
  TCB0.EVCTRL = TCB_CAPTEI_bm;       // Enable the timer capture interrupt
  TCB0.INTCTRL = TCB_CAPT_bm;        // Enable the Capture Interrupt
  TCB0.CTRLA = TCB_ENABLE_bm;        // Select Pre scaler //Start Timmer

  TCB1.CTRLB = TCB_CNTMODE_FRQPW_gc; // Input capture pulse width measurement and frequency
  TCB1.EVCTRL = TCB_CAPTEI_bm;       // Enable the timer capture interrupt
  TCB1.INTCTRL = TCB_CAPT_bm;        // Enable the Capture Interrupt
  TCB1.CTRLA = TCB_ENABLE_bm;        // Select Pre scaler //Start Timmer
}

/**************************************************************
 * Calculation
 * ***********************************************************/

uint8_t dutyCycle_1(void)
{
  uint16_t tmp16;
  uint16_t val16;
  if (PWM0_dur == 0)
  {
    if (PORTA_IN & _BV(7))
      return 100;
    else
      return 0;
  }
  tmp16 = PWM0_dur / 100;
  val16 = PWM0_high / tmp16;
  PWM0_dur = 0;
  return val16;
}

uint8_t dutyCycle_2(void)
{
  uint16_t tmp16;
  uint16_t val16;
  if (PWM1_dur == 0)
  {
    if (PORTB_IN & _BV(5))
      return 100;
    else
      return 0;
  }
  tmp16 = PWM1_dur / 100;
  val16 = PWM1_high / tmp16;
  PWM1_dur = 0;
  return val16;
}


/**************************************************************
 * Switch pins and relays on shift register via SPI
 * ***********************************************************/

uint8_t serial_port = 0;
void set_serial_port(uint8_t pattern)
{
  PORTA_OUT = PORTA_OUT & ~_BV(SERIAL_PORT_CS_PIN);
  SPI0_exchangeData(pattern);
  PORTA_OUT = PORTA_OUT | _BV(SERIAL_PORT_CS_PIN);
  serial_port = pattern;
}

void set_serial_port_pin(uint8_t pin, uint8_t state)
{
  if (state)
    serial_port = serial_port | (1<<pin);
  else
    serial_port = serial_port & ~(1<<pin);
  set_serial_port(serial_port);
}

void set_relais_A(void)
{
  uint8_t iso;
  uint8_t mode;

  iso  = analogOutput[AO_REG_OFFSET_ISO_A];
  mode = analogOutput[AO_REG_OFFSET_MOD_A];

  serial_port = serial_port &
    ~(_BV(REL_BIT_ISO_B_1) | _BV(REL_BIT_ISO_C_1) | _BV(REL_BIT_13_A_1) | _BV(REL_BIT_ERR_1));

  switch (iso)
  {
    case REGVALUE_ISOSTATE_C:
      serial_port = serial_port | _BV(REL_BIT_ISO_C_1);
      break;
    case REGVALUE_ISOSTATE_1B:
    case REGVALUE_ISOSTATE_B:
      serial_port = serial_port | _BV(REL_BIT_ISO_B_1);
      break;
  }
  switch (mode)
  {
    case REGVAL_MODE_13A:
      serial_port = serial_port | _BV(REL_BIT_13_A_1);
      break;
    case REGVAL_MODE_ERR:
      serial_port = serial_port | _BV(REL_BIT_ERR_1);
      break;
  }
  set_serial_port(serial_port);
}

void set_relais_B(void)
{
  uint8_t iso;
  uint8_t mode;

  iso  = analogOutput[AO_REG_OFFSET_ISO_B];
  mode = analogOutput[AO_REG_OFFSET_MOD_B];

  serial_port = serial_port &
    ~(_BV(REL_BIT_ISO_B_2) | _BV(REL_BIT_ISO_C_2) | _BV(REL_BIT_13_A_2) | _BV(REL_BIT_ERR_2));

  switch(iso)
  {
    case REGVALUE_ISOSTATE_C:
      serial_port = serial_port | _BV(REL_BIT_ISO_C_2);
      break;
    case REGVALUE_ISOSTATE_1B:
    case REGVALUE_ISOSTATE_B:
      serial_port = serial_port | _BV(REL_BIT_ISO_B_2);
      break;
  }
  switch(mode)
  {
    case REGVAL_MODE_13A:
      serial_port = serial_port | _BV(REL_BIT_13_A_2);
      break;
    case REGVAL_MODE_ERR:
      serial_port = serial_port | _BV(REL_BIT_ERR_2);
      break;
  }
  set_serial_port(serial_port);
}


/**************************************************************
 * Show state information via a small array of ws2812 compatible 
 * LEDs. 
 * ***********************************************************/

pixel_array_t pixel_array[NR_OF_LEDS_ON_STRIP] = {0};
uint8_t pixel_data[NR_OF_LEDS_ON_STRIP] = {0};

// There are 10 LEDs with the following meaning: 
#define LED_ISO_A  0
#define LED_MOD_A  1
#define LED_PWM1_A 2
#define LED_PWM2_A 3
#define LED_PWM3_A 4
#define LED_ISO_B  5
#define LED_MOD_B  6
#define LED_PWM1_B 7
#define LED_PWM2_B 8
#define LED_PWM3_B 9

/* Three LEDs will be used to indicate the PWM duty cycle, 
   respectively the output current. The levels are according to
   specification. 

https://de.wikipedia.org/wiki/IEC_62196_Typ_2
Pulsweiten zur Anzeige der höchsten Stromlast[7] PWM 	SAE dauerhaft 	SAE kurzzeitig 	IEC 61851-1
97 %	80 A (EU)   1 White  LED
95 %	77,5 A (EU) 3 Violet LEDs
90 %	65 A (EU)   2 Violet LEDs
85 %	51 A (EU)   1 Violet LED
80 %	48 A (EU)   3 Red    LEDs
70 %	42 A (EU)   2 Red    LEDs
60 %	36 A (EU)   1 Red    LED
50 % 	30 A (EU)   2 Orange LEDs
40 % 	24 A (EU)   2 Orange LEDs
30 % 	18 A (EU)   1 Orange LED
25 % 	15 A (EU)   3 Yellow LED
16 %	9,6 A (EU)  2 Yellow LEDs
10 %	6 A (EU)    1 Yellow LED
                      OFF

Following colors are used for the ISO states and cable mode

ISO_A Green
ISO_B Cyan
ISO_C Blue

MODE_MAX Off
MODE_13A Magenta
MODE_ERR Red
*/

void set_led_array(uint8_t select)
{
  uint8_t iso;
  uint8_t mode;
  uint8_t offset;
  uint8_t brightness = get_AO_register_lo(AO_REG_OFFSET_CFG_BRIGHTNESS);

  if (SELECT_A == select)
  {
    iso  = analogOutput[AO_REG_OFFSET_ISO_A];
    mode = analogOutput[AO_REG_OFFSET_MOD_A];
    offset = 0;
  }
  else
  {
    iso  = analogOutput[AO_REG_OFFSET_ISO_B];
    mode = analogOutput[AO_REG_OFFSET_MOD_B];
    offset = LED_ISO_B;
  }

  switch (iso)
  {
    case REGVALUE_ISOSTATE_C:
      pixel_array[offset + LED_ISO_A].pixel = hsv_to_rgb(H_VAL_BLUE, 255, brightness);
      break;
    case REGVALUE_ISOSTATE_1B:
    case REGVALUE_ISOSTATE_B:
      pixel_array[offset + LED_ISO_A].pixel = hsv_to_rgb(H_VAL_BLUE_GREEN, 255, brightness/2);
      break;
    default:
      pixel_array[offset + LED_ISO_A].pixel = hsv_to_rgb(H_VAL_GREEN, 255, brightness);
      break;
  }
  switch (mode)
  {
    case REGVAL_MODE_13A:
      pixel_array[offset + LED_MOD_A].pixel = hsv_to_rgb(H_VAL_YELLOW_GREEN, 255, brightness);
      break;
    case REGVAL_MODE_ERR:
      pixel_array[offset + LED_MOD_A].pixel = hsv_to_rgb(H_VAL_RED, 255, brightness);
      break;
    default:
      pixel_array[offset + LED_MOD_A].pixel = hsv_to_rgb(0, 0, 0);
      break;
  }
}

void set_leds_color(pixel_array_t* leds, uint8_t nr, uint8_t h, uint8_t s, uint8_t v)
{
  while (nr--)
  {
    leds[2].pixel = hsv_to_rgb(h, s, v);
  }
}

void set_3_leds(pixel_array_t* leds, uint8_t nr, uint8_t bg_h, uint8_t bg_brightness, uint8_t set_h)
{
  uint8_t brightness = get_AO_register_lo(AO_REG_OFFSET_CFG_BRIGHTNESS);
  for(uint8_t n = 0; n < 3; n++)
    leds[n].pixel = hsv_to_rgb(bg_h, 255, bg_brightness);
  if (nr > 0)
    leds[0].pixel = hsv_to_rgb(set_h, 255, brightness);
  if (nr > 1)
    leds[1].pixel = hsv_to_rgb(set_h, 255, brightness);
  if (nr > 2)
    leds[2].pixel = hsv_to_rgb(set_h, 255, brightness);
}


/* Remember above :-)
97 %	80 A (EU)   1 White
95 %	77,5 A (EU) 3 Violet
90 %	65 A (EU)   2 Violet
85 %	51 A (EU)   1 Violet
80 %	48 A (EU)   3 Red
70 %	42 A (EU)   2 Red
60 %	36 A (EU)   1 Red
50 % 	30 A (EU)   2 Orange
40 % 	24 A (EU)   2 Orange
30 % 	18 A (EU)   1 Orange
25 % 	15 A (EU)   3 Yellow
16 %	9,6 A (EU)  2 Yellow
10 %	6 A (EU)    1 Yellow
                    OFF
*/

void set_led_array_pwm(pixel_array_t* leds, uint8_t dc)
{
  uint8_t brightness = get_AO_register_lo(AO_REG_OFFSET_CFG_BRIGHTNESS);
  if (dc == 100)
  {
    leds[0].pixel = hsv_to_rgb(0, 0, 0);
    leds[1].pixel = hsv_to_rgb(0, 0, 0);
    leds[2].pixel = hsv_to_rgb(H_VAL_RED, 0, brightness);
  }
  else if (dc >= 97) set_3_leds(leds, 3, H_VAL_VIOLET, brightness, H_VAL_INDIGO);
  else if (dc >= 90) set_3_leds(leds, 2, H_VAL_VIOLET, brightness, H_VAL_INDIGO);
  else if (dc >= 85) set_3_leds(leds, 1, H_VAL_VIOLET, brightness, H_VAL_INDIGO);
  else if (dc >= 80) set_3_leds(leds, 3, H_VAL_RED, brightness, H_VAL_VIOLET);
  else if (dc >= 70) set_3_leds(leds, 2, H_VAL_RED, brightness, H_VAL_VIOLET);
  else if (dc >= 60) set_3_leds(leds, 1, H_VAL_RED, brightness, H_VAL_VIOLET);
  else if (dc >= 50) set_3_leds(leds, 3, H_VAL_YELLOW, brightness, H_VAL_RED);
  else if (dc >= 40) set_3_leds(leds, 2, H_VAL_YELLOW, brightness, H_VAL_RED);
  else if (dc >= 30) set_3_leds(leds, 1, H_VAL_YELLOW, brightness, H_VAL_RED);
  else if (dc >= 25) set_3_leds(leds, 3, H_VAL_GREEN, 0, H_VAL_YELLOW);
  else if (dc >= 16) set_3_leds(leds, 2, H_VAL_GREEN, 0, H_VAL_YELLOW);
  else if (dc >= 10) set_3_leds(leds, 1, H_VAL_GREEN, 0, H_VAL_YELLOW);
  else set_3_leds(leds, 0, H_VAL_GREEN, 0, H_VAL_YELLOW);
}

void update_leds(uint8_t update)
{
  uint8_t dc;
  if (update & UPDATE_STATE_A)
    set_led_array(SELECT_A);
  if (update & UPDATE_STATE_B)
    set_led_array(SELECT_B);
  dc = get_AI_register_lo(AI_REG_OFFSET_PWM_A);
  if (update & UPDATE_PWM_A)
    set_led_array_pwm(&pixel_array[2], dc);
  dc = get_AI_register_lo(AI_REG_OFFSET_PWM_B);
  if (update & UPDATE_PWM_B)
    set_led_array_pwm(&pixel_array[7], dc);
}


/**************************************************************
 * Initialization and main loop
 * ***********************************************************/
int main(void)
{
//  uint8_t h,s,v;
//  uint8_t tmp;
  uint16_t tmp16;
  uint16_t val16;
  uint8_t update;

  CPU_CCP = 0xD8;
  CLKCTRL_MCLKCTRLB = 0x1; /* divider 2 Clock = 10MHz => one Tick = 100ns*/

  IOinit();

  SPI0_init_master(SPI_PRESC_DIV64_gc, 0);
  set_serial_port(0);

  I2C_init_master(I2C_BAUD_100kHz);
  _delay_ms(100);

  oled_init(OLED_DISP_ON);
  _delay_ms(100);
  oled_gotoxy(0,0); oled_putc(65);
  oled_gotoxy(1,1); oled_putc(66);
  oled_gotoxy(2,2); oled_putc(67);
  _delay_ms(1000);

  while (0)
  {
    PORTA_OUT = PORTA_OUT & ~_BV(SERIAL_PORT_CS_PIN);
    SPI0_exchangeData(0xaa);
    PORTA_OUT = PORTA_OUT | _BV(SERIAL_PORT_CS_PIN);
    _delay_ms(100);

    PORTA_OUT = PORTA_OUT & ~_BV(SERIAL_PORT_CS_PIN);
    SPI0_exchangeData(0x55);
    PORTA_OUT = PORTA_OUT | _BV(SERIAL_PORT_CS_PIN);
    _delay_ms(100);
  }

  while (0)
  {
    set_serial_port_pin(7, 1);
    _delay_ms(200);
    set_serial_port_pin(6, 1);
    _delay_ms(200);
    set_serial_port_pin(5, 1);
    _delay_ms(200);
    set_serial_port_pin(4, 1);
    _delay_ms(200);
    set_serial_port(0);
    _delay_ms(200);
  }

/*
  // Some test code for the OLED
  oled_command(0xae);_delay_ms(200);
  oled_command(0xc8);_delay_ms(200);
  oled_command(0xaf);_delay_ms(200);
  _delay_ms(100);
  oled_gotoxy(0,0);oled_putc(92);
  oled_gotoxy(1,1);oled_putc(92);
  oled_gotoxy(2,2);oled_putc(92);
  oled_gotoxy(3,3);oled_putc(92);
  _delay_ms(10000);
*/

  strcpy(string, __DATE__);
  strcat(string, "\n");
  oled_gotoxy(0, 0); oled_puts(string);
  strcpy(string, "EvSim Robo \n");
  oled_gotoxy(0, 1); oled_puts(string);

  PORTC_DIR = 0x03;
  uint8_t h, v, s;
  s = 255;
  v = 32;
  h = 0;


  //Some test code for the ws2812 LEDs
  while (0)
  {
    for (uint8_t n = 0; n < 5; n++)
    {
      pixel_array[0].pixel = hsv_to_rgb(h+5,  s, v);
      pixel_array[1].pixel = hsv_to_rgb(h+10, s, v);
      pixel_array[2].pixel = hsv_to_rgb(h+15, s, v);
      pixel_array[3].pixel = hsv_to_rgb(h+20, s, v);
      pixel_array[4].pixel = hsv_to_rgb(h+25, s, v);
    }
    h += 7;
    pixel_array[0].pixel = hsv_to_rgb(0,0,0);
    send_rgb_stream(pixel_array[0].data, 15, 1);
    _delay_ms(100);
  }


  //Initialize the LEDs
  for (uint8_t n = 0; n < 5; n++)
  {
    memset(pixel_array, 0, 5*3);
    pixel_array[n].pixel = hsv_to_rgb(n*16, s, v);
    send_rgb_stream(pixel_array[0].data, 15, 1);
    _delay_ms(200);
  }

  /* Initialize the peripherals */

  EVENT_SYSTEM_init();
  TCB_init();

  USART0_init(BR_9600);
  USART0.CTRLC |= (0x2 << 4);

  /*
   * F_CPU 3,3 MHz Div256(0x6) => 1000ms = 3,3/256 =12890
   *                           =>    1ms =         =12
   * F_CPU 3,3 MHz Div64(0x5)  => 1000ms =  3,3/64 =51562
   *                           =>    1ms =         =51
   * F_CPU  10 MHz Div256(0x6) => 1000ms =  10/256 =39062
   *                           =>    1ms =         =39
   * F_CPU  10 MHz Div64(0x5)  => 1000ms =  10/64  =156250
   *                           =>    1ms =         =156
   */
  TCA0_SINGLE_CTRLA = (0x06 << TCA_SINGLE_CLKSEL_0_bp) | TCA_SINGLE_ENABLE_bm;
  mbrtu_init(2);
  mbrtu_set_parameter(39, BR_9600);
  init_modbus_registers();

  sei();

  PORTC_DIR = 0x03;
  PORTC_OUT = 0x00;
  uint8_t frameLen = 0;
  uint8_t frame[20];
  uint8_t cnt = 0;
  uint8_t tmp;
  rtu_request_t rtuFrame;

  //Some test code for UART output
  while (0)
  {
    USART0_sendChar(0x0f);
    USART0_sendChar(0xaa);
    _delay_ms(5);
  }

  //Some test code for UART output
  while (0)
  {
    PORTC_OUT = 1;
    _delay_ms(1);
    USART0_sendChar(65);
    USART0_sendChar(66);
    USART0_sendChar(67);
    _delay_ms(5);
    PORTC_OUT = 0;
    _delay_ms(500);
  }

  //Some test code for OLED and counter
  while (0)
  {
    strcpy(string, "tim=");
    strcat_uint(string, TCA0_SINGLE_CNT);
    TCA0_SINGLE_CNT = 0;
    strcat(string, " ");
    strcat_uint(string, cnt++);
    strcat(string, "-");
    oled_gotoxy(0, 0);
    oled_puts(string);
    _delay_ms(950);
  }


  //Some test code to see that CRC is calculation works
  uint8_t txFrame[20];
  txFrame[0] = 0x02;
  txFrame[1] = 0x06;
  txFrame[2] = 0x00;
  txFrame[3] = 0x01;
  txFrame[4] = 0x00;
  txFrame[5] = 0x0E;
  txFrame[6] = 0x59;
  txFrame[7] = 0xFD;
  tmp16 = calculate_crc(txFrame, 6);
  strcpy(string, "CRC=");
  strcat_hex(string, tmp16);
  strcat_hex(string, tmp16 >> 8);
  strcat(string, "   ");
  oled_gotoxy(0, 0); oled_puts(string);

  //<02><03><02><AA><BB><00><C0>
  //ERROR CRC received 0xC0 != CRC calculated 0xC297
  txFrame[0] = 0x02;
  txFrame[1] = 0x03;
  txFrame[2] = 0x02;
  txFrame[3] = 0xaa;
  txFrame[4] = 0xbb;
  tmp16 = calculate_crc(txFrame, 5);
  strcpy(string, "CRC=");
  strcat_hex(string, tmp16);
  strcat_hex(string, tmp16 >> 8);
  strcat(string, "   ");
  oled_gotoxy(0, 1);
  oled_puts(string);
  tmp16 = calculate_crc(txFrame, 5);
  strcpy(string, "CRC=");
  strcat_hex(string, tmp16);
  strcat_hex(string, tmp16 >> 8);
  strcat(string, "   ");
  oled_gotoxy(0, 2); oled_puts(string);
  _delay_ms(1000);
  while (0) {;}

  // mbpoll -v -m rtu  -a 2 -r 2 -b 9600 -t 4 /dev/ttyUSB0 0x0e
  txFrame[0] = 0x02;
  txFrame[1] = 0x06;
  txFrame[2] = 0x00;
  txFrame[3] = 0x01;
  txFrame[4] = 0x00;
  txFrame[5] = 0x0E;
  txFrame[6] = 0x59;
  txFrame[7] = 0xFD;

  //Test loop for UART reception
  while (0)
  {
    frameLen = nr_bytes_in_buffer();
    if (frameLen != 0 && TCA0_SINGLE_CNT > 100)
    {
      read_UART_buffer(frame, 20);
//        if(frameLen==8)
      {
//         _delay_ms(1);
        PORTC_OUT = 1;
        _delay_ms(1);
        for (uint8_t i = 0; i < 8; i++)
          USART0_sendChar(txFrame[i]);
//          USART0_sendChar(0x52);
        _delay_ms(5);
        PORTC_OUT = 0;

        tmp16 = calculate_crc(frame, 6);

        strcpy(string, "FrLen=");
        strcat_uint(string, frameLen);
        strcat(string, " ");
//          strcat_hex(string,cnt++);
        strcat_hex(string, tmp16);
        strcat_hex(string, tmp16 >> 8);

        strcat(string, "-");
        oled_gotoxy(0, 0); oled_puts(string);
        strcpy(string, ":");
        strcat_uint(string, frame[0]); strcat(string, " ");
        strcat_uint(string, frame[1]); strcat(string, " ");
        strcat_uint(string, frame[2]); strcat(string, " ");
        oled_gotoxy(0, 1); oled_puts(string);
        strcpy(string, ":");
        strcat_uint(string, frame[3]); strcat(string, " ");
        strcat_uint(string, frame[4]); strcat(string, " ");
        strcat_uint(string, frame[5]); strcat(string, " ");
        oled_gotoxy(0, 2); oled_puts(string);
        strcpy(string, ":");
        strcat_uint(string, frame[6]); strcat(string, " ");
        strcat_uint(string, frame[7]); strcat(string, " ");
        strcat_uint(string, frame[8]); strcat(string, " ");
        oled_gotoxy(0, 3); oled_puts(string);
      }
    }
    _delay_ms(10);
  }

  //Test loop for modbus RTU reception
  while (0)
  {
    tmp = mbrtu_check_buffer(&rtuFrame);
    if (tmp != FRAME_IDLE)
    {
      strcpy(string, ":");
      strcat_uint(string, cnt++);
      strcat(string, " Ad=");
      strcat_uint(string, rtuFrame.address); strcat(string, " Fk=");
      strcat_uint(string, rtuFrame.function); strcat(string, " ");
      oled_gotoxy(0,0);oled_puts(string);

      strcpy(string, "ro=");
      strcat_uint(string, rtuFrame.regOffset);
      strcat(string, " len=");
      strcat_uint(string, rtuFrame.len); strcat(string," ");
      oled_gotoxy(0, 1); oled_puts(string);

      strcpy(string, "data:");
      strcat_hex(string, rtuFrame.data[0]); strcat(string," ");
      strcat_hex(string, rtuFrame.data[1]); strcat(string,"  ");
      oled_gotoxy(0, 2); oled_puts(string);

      strcpy(string, "CRC=");
      strcat_hex(string, rtuFrame.crc); strcat(string, "  ");
      oled_gotoxy(0, 3); oled_puts(string);
    }
    _delay_ms(5);
  }

  //Test loop for PWM measurement
  while (0)
  {
    strcpy(string,"P");
    strcat_uint(string, PWM0_high);
    strcat(string, " D");
    strcat_uint(string, PWM0_dur);
    strcat(string, " C");
    tmp16 = PWM0_dur / 100;
    val16 = PWM0_high / tmp16;
    strcat_uint(string, val16);
    strcat(string, " ");
    oled_gotoxy(0, 2); oled_puts(string);

    strcpy(string, "P");
    strcat_uint(string, PWM1_high);
    strcat(string, " D");
    strcat_uint(string, PWM1_dur);
    strcat(string, " C");
    tmp16 = PWM1_dur / 100;
    val16 = PWM1_high / tmp16;
    strcat_uint(string, val16);
    strcat(string, " ");
    oled_gotoxy(0, 3); oled_puts(string);
    _delay_ms(100);
  }

  /**********************************************
  * Now here happens the real action !
  ***********************************************/
  uint8_t dc_A = 0;
  uint8_t dc_A_prev = 1;
  uint8_t dc_B = 0;
  uint8_t dc_B_prev = 1;
  update = 0;
  oled_clrscr();
  update_rotation();
  while (1)
  {
    //TODO use RTC and tick_cnt instead of _delay_ms
    _delay_ms(10);
    cyclic_check_taster();

    // Handle key input
    //----------------------------------------------------- 
    if (key_event_released == 1)
    {
      if (key_event_key == 1)
      {
        if (key_pressed_cnt > LONG_PRESSED_CNT)
        {
          if (analogOutput[AO_REG_OFFSET_MOD_A] == REGVAL_MODE_MAX)
            analogOutput[AO_REG_OFFSET_MOD_A] = REGVAL_MODE_13A;
          else if (analogOutput[AO_REG_OFFSET_MOD_A] == REGVAL_MODE_13A)
            analogOutput[AO_REG_OFFSET_MOD_A] = REGVAL_MODE_ERR;
          else if (analogOutput[AO_REG_OFFSET_MOD_A] == REGVAL_MODE_ERR)
            analogOutput[AO_REG_OFFSET_MOD_A] = REGVAL_MODE_MAX;
        }
        else
        {
          if (analogOutput[AO_REG_OFFSET_ISO_A] == REGVALUE_ISOSTATE_A)
            analogOutput[AO_REG_OFFSET_ISO_A] = REGVALUE_ISOSTATE_B;
          else if (analogOutput[AO_REG_OFFSET_ISO_A] == REGVALUE_ISOSTATE_B)
            analogOutput[AO_REG_OFFSET_ISO_A] = REGVALUE_ISOSTATE_C;
          else if (analogOutput[AO_REG_OFFSET_ISO_A] == REGVALUE_ISOSTATE_C)
            analogOutput[AO_REG_OFFSET_ISO_A] = REGVALUE_ISOSTATE_1B;
          else if (analogOutput[AO_REG_OFFSET_ISO_A] == REGVALUE_ISOSTATE_1B)
            analogOutput[AO_REG_OFFSET_ISO_A] = REGVALUE_ISOSTATE_A;
          else
            analogOutput[AO_REG_OFFSET_ISO_A] = REGVALUE_ISOSTATE_A;
        }
      }
      else if (key_event_key == 2)
      {
        if (key_pressed_cnt > LONG_PRESSED_CNT)
        {
          if (analogOutput[AO_REG_OFFSET_MOD_B] == REGVAL_MODE_MAX)
            analogOutput[AO_REG_OFFSET_MOD_B] = REGVAL_MODE_13A;
          else if (analogOutput[AO_REG_OFFSET_MOD_B] == REGVAL_MODE_13A)
            analogOutput[AO_REG_OFFSET_MOD_B] = REGVAL_MODE_ERR;
          else if (analogOutput[AO_REG_OFFSET_MOD_B] == REGVAL_MODE_ERR)
            analogOutput[AO_REG_OFFSET_MOD_B] = REGVAL_MODE_MAX;
        }
        else
        {
          if (analogOutput[AO_REG_OFFSET_ISO_B] == REGVALUE_ISOSTATE_A)
            analogOutput[AO_REG_OFFSET_ISO_B] = REGVALUE_ISOSTATE_B;
          else if (analogOutput[AO_REG_OFFSET_ISO_B] == REGVALUE_ISOSTATE_B)
            analogOutput[AO_REG_OFFSET_ISO_B] = REGVALUE_ISOSTATE_C;
          else if (analogOutput[AO_REG_OFFSET_ISO_B] == REGVALUE_ISOSTATE_C)
            analogOutput[AO_REG_OFFSET_ISO_B] = REGVALUE_ISOSTATE_1B;
          else if (analogOutput[AO_REG_OFFSET_ISO_B] == REGVALUE_ISOSTATE_1B)
            analogOutput[AO_REG_OFFSET_ISO_B] = REGVALUE_ISOSTATE_A;
          else
            analogOutput[AO_REG_OFFSET_ISO_B] = REGVALUE_ISOSTATE_A;
        }
      }
      key_event_done();
      update = update | UPDATE_STATE_A;
    }


    // Handle modbus RTU messages
    //----------------------------------------------------- 
    tmp = mbrtu_check_buffer(&rtuFrame);
    if (tmp == FRAME_OK)
    {
      if (FUNC_WRITE_ANALOG_OUTPUT == rtuFrame.function && AO_REG_OFFSET_ISO_A == rtuFrame.regOffset)
      {
        if (0x000A == rtuFrame.data[0] ||
            0x000B == rtuFrame.data[0] ||
            0x000C == rtuFrame.data[0])
        {
          analogOutput[AO_REG_OFFSET_ISO_A] = rtuFrame.data[0];
          mbrtu_generate_confirmation(&rtuFrame);
          update = update | UPDATE_STATE_A;
        }
      }
      else if (FUNC_WRITE_ANALOG_OUTPUT == rtuFrame.function && AO_REG_OFFSET_ISO_B == rtuFrame.regOffset)
      {
        if (0x000A == rtuFrame.data[0] ||
            0x000B == rtuFrame.data[0] ||
            0x000C == rtuFrame.data[0])
        {
          analogOutput[AO_REG_OFFSET_ISO_B] = rtuFrame.data[0];
          mbrtu_generate_confirmation(&rtuFrame);
          update = update | UPDATE_STATE_B;
        }
      }
      else if (FUNC_WRITE_ANALOG_OUTPUT == rtuFrame.function && AO_REG_OFFSET_MOD_A == rtuFrame.regOffset)
      {
        if (REGVAL_MODE_MAX == rtuFrame.data[0] ||
            REGVAL_MODE_13A == rtuFrame.data[0] ||
            REGVAL_MODE_ERR == rtuFrame.data[0])
        {
          analogOutput[AO_REG_OFFSET_MOD_A] = rtuFrame.data[0];
          mbrtu_generate_confirmation(&rtuFrame);
          update = update | UPDATE_STATE_A;
        }
      }
      else if (FUNC_WRITE_ANALOG_OUTPUT == rtuFrame.function && AO_REG_OFFSET_MOD_B == rtuFrame.regOffset)
      {
        if (REGVAL_MODE_MAX == rtuFrame.data[0] ||
            REGVAL_MODE_13A == rtuFrame.data[0] ||
            REGVAL_MODE_ERR == rtuFrame.data[0])
        {
          analogOutput[AO_REG_OFFSET_MOD_B] = rtuFrame.data[0];
          mbrtu_generate_confirmation(&rtuFrame);
          update = update | UPDATE_STATE_B;
        }
      }
      else if (FUNC_WRITE_ANALOG_OUTPUT == rtuFrame.function && AO_REG_OFFSET_CFG_ROTATION == rtuFrame.regOffset)
      {
        analogOutput[rtuFrame.regOffset] = rtuFrame.data[0];
        mbrtu_generate_confirmation(&rtuFrame);
        update_rotation();
      }
      else if (FUNC_WRITE_ANALOG_OUTPUT == rtuFrame.function && AO_REG_OFFSET_AUX == rtuFrame.regOffset)
      {
        analogOutput[rtuFrame.regOffset] = rtuFrame.data[0];
        mbrtu_generate_confirmation(&rtuFrame);
        if(rtuFrame.data[0] == 0)
          AUX_PORT = AUX_PORT & ~_BV(AUX_PORT_PIN);
        else
          AUX_PORT = AUX_PORT | _BV(AUX_PORT_PIN);
      }
      else if (FUNC_WRITE_ANALOG_OUTPUT == rtuFrame.function)
      {
        analogOutput[rtuFrame.regOffset] = rtuFrame.data[0];
        mbrtu_generate_confirmation(&rtuFrame);
      }

      if (FUNC_READ_ANALOG_OUTPUT == rtuFrame.function && rtuFrame.len < NR_OF_AO_REGISTERS)
      {
        mbrtu_generate_frame(FUNC_READ_ANALOG_OUTPUT, &analogOutput[rtuFrame.regOffset], rtuFrame.len);
      }
      if (FUNC_READ_ANALOG_INPUT == rtuFrame.function && rtuFrame.len < NR_OF_AO_REGISTERS)
      {
        mbrtu_generate_frame(FUNC_READ_ANALOG_INPUT, &analogInput[rtuFrame.regOffset], rtuFrame.len);
      }
    }
    
    // Handle PWM duty measurements and related LEDs
    //----------------------------------------------------- 
    dc_A = dutyCycle_1();
    if (dc_A_prev != dc_A)
    {
      dc_A_prev = dc_A;
      update = update | UPDATE_PWM_A;
      analogInput[AI_REG_OFFSET_PWM_A] = dc_A;
      set_led_array_pwm(&pixel_array[2], dc_A);
      send_rgb_stream(pixel_array[0].data, 15, 1);
    }
    dc_B = dutyCycle_2();
    if (dc_B_prev != dc_B)
    {
      dc_B_prev = dc_B;
      update = update | UPDATE_PWM_B;
      analogInput[AI_REG_OFFSET_PWM_B] = dc_B;
      set_led_array_pwm(&pixel_array[7], dc_A);
      send_rgb_stream(pixel_array[0].data, 15, 1);
    }


    // Handle update OLED and state LEDs
    //----------------------------------------------------- 
    if (update)
    {
      update_display(update);
      set_led_array(SELECT_A);
      set_relais_A();
      send_rgb_stream(pixel_array[0].data, 15, 1);
      set_relais_B();
      update = 0;
    }
  }
}
