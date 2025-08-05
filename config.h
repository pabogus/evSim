#ifndef _CONFIG_H_
#define _CONFIG_H_

/*
 * config.h
 *
 * Created: 30.12.2023
 * Author:  Robert Weikert
 */

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>


/*
//--------------------------------------------------------------------
// configure USART Settings
//--------------------------------------------------------------------
*/
#define USE_UART
// Must be power of 2
#define UART_RX_BUFFER_LEN 64
//#define UART_MAX_LINE_LENGTH 16
#define USART_USE_RX_INTERRUPT
//#define USART_USE_RX_INTERRUPT_ECHO
//#define uart_rx_idx GPIOR1
#define RESET_TIMEOUT TCA0_SINGLE_CNT=0

/*
//--------------------------------------------------------------------
// configure modbus RTU functions
//--------------------------------------------------------------------
*/
// #define NR_OF_AO_REGISTERS 16
// #define NR_OF_AI_REGISTERS 16
#define RTU_TIMOUT_COUNTER TCA0_SINGLE_CNT


/*
//--------------------------------------------------------------------
// configure I2C
//--------------------------------------------------------------------
*/
#define USE_I2C_MASTER


/*
//--------------------------------------------------------------------
// configure oled functions
//--------------------------------------------------------------------
*/
#define SH1106	// or SSD1306, check datasheet of your display
//#define SSD1306
//#define SSD1306_64x48
//#define USE_OLED_FONT_6x8
#define USE_OLED_FONT_8x16
//#define USE_OLED_GRAPHIC_SUPPORT
#define USE_OLED_VERTICAL
#define OLED_FLIP_HORIZONTAL

#define CFG_DEFAULT_ROTATION ROTATION_VERTICAL_FLIP
//#define CFG_DEFAULT_ROTATION ROTATION_HORIZONTAL_FLIP


/*
//--------------------------------------------------------------------
// configure support functions
//--------------------------------------------------------------------
*/

//#define USE_BIT_TO_STR_FUNCTION
#define USE_U_TO_STR_FUNCTION
//#define USE_I_TO_STR_FUNCTION
//#define USE_DISPLAY_BUFFER
//#define SIZE_OF_DISPLAY_BUFFER 40


/*
//--------------------------------------------------------------------
// configure led_ws2812
//--------------------------------------------------------------------
*/
#define LED_VPORT_OUT VPORTC_OUT
#define LED_STIPE_DATA_PIN_NR 1
#define USE_FAST_HSV2RGB_FUNCTION
//#define LED_DIM_NR_SHIFTS 4


/*
//--------------------------------------------------------------------
// configure LED Strip functions
//--------------------------------------------------------------------
*/
#define DEFAULT_BRIGTNES 128
#define NR_OF_LEDS_ON_STRIP 20


//--------------------------------------------------------------------
// Configuration for key input functions
// Place needed defines into config.h
//--------------------------------------------------------------------
#define _ENABLE_SIMPLE_KEY_INPUT
#define TASTER1_PORT_IN PORTA_IN
#define TASTER1_PIN 5
//#define TASTER2_PORT_IN PORTA_IN
//#define TASTER2_PIN 2
//#define TASTER3_PORT_IN PORTA_IN
//#define TASTER3_PIN 3
//#define TASTER4_PORT_IN PORTA_IN
//#define TASTER4_PIN 4
//#define TASTER5_PORT_IN PORTA_IN
//#define TASTER5_PIN 5

//#define ENABLE_ADC_KEY_INPUT
//#define TASTER_ADCIN ADC_MUXPOS_AIN2_gc

#endif // _CONFIG_H_
