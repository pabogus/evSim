#ifndef __EV_SIM_H__
#define __EV_SIM_H__

/*
 * evSim.h
 *
 * Created: 30.12.2023
 * Author:  Robert Weikert
 *
 * Provides functions to display satus information
 * about the ev-simulator on an OLED display.
 */

#include "config.h"



/**************************************************************
 * Defines for default values
 * These might be adapted as wished e.g. in config.h
 * ***********************************************************/
#ifndef CFG_DEFAULT_BRIGHTNESS
  #define CFG_DEFAULT_BRIGHTNESS 48
#endif

#ifndef CFG_DEFAULT_ROTATION
  #define CFG_DEFAULT_ROTATION ROTATION_HORIZONTAL
#endif



/**************************************************************
 * Enums and typse
 * ***********************************************************/

enum
{
  ISO_A = 0,
  ISO_B,
  ISO_C,
  ISO_B_,
  ISO_MAX
};

enum
{
  MODE_MAX_A = 0,
  MODE_13_A,
  MODE_ERROR,
  MODE_MAX
};

enum
{
  SELECT_A,
  SELECT_B
};



/**************************************************************
 * Function declaration
 * ***********************************************************/
/**
 * Update display according to status flags.
 * 
 * @param update Status flags from `UPDATE_STATE_...` and `UPDATE_PWM_...`
 */
void update_display(uint8_t update);

/**
 * Update display rotation from the `AO_REG_OFFSET_CFG_ROTATION` register.
 */
void update_rotation(void);



/**************************************************************
 * Defines needed for the usage of the evSim functions
 * ***********************************************************/
#define UPDATE_STATE_A (1<<0)
#define UPDATE_STATE_B (1<<1)
#define UPDATE_PWM_A   (1<<2)
#define UPDATE_PWM_B   (1<<3)



/**************************************************************
 * Defines needed for OLED display functions
 * ***********************************************************/
#define ROTATION_HORIZONTAL       0
#define ROTATION_HORIZONTAL_FLIP  1
#define ROTATION_VERTICAL         2
#define ROTATION_VERTICAL_FLIP    3



/**************************************************************
 * Defines needed for Modbus RTU functions
 * ***********************************************************/

/* analogOutput register definitions
 * can be written.
 */
//---Control Registers---
#define AO_REG_OFFSET_ISO_A 0
#define AO_REG_OFFSET_MOD_A 1
#define AO_REG_OFFSET_ISO_B 2
#define AO_REG_OFFSET_MOD_B 3
#define AO_REG_OFFSET_AUX   4


//---Configuration Registers---
#define AO_REG_OFFSET_CFG_BRIGHTNESS 10
#define AO_REG_OFFSET_CFG_ROTATION   11


/* analogInput register definitions
 * only readable
 */
//---variable content  Registers---
#define AI_REG_OFFSET_PWM_A       0
#define AI_REG_OFFSET_PWM_B       1
//---fixed content  Registers---
#define AI_REG_OFFSET_SW_VERSION  10
#define AI_REG_OFFSET_SW_YEAR     11
#define AI_REG_OFFSET_SW_MONTHDAY 12


/* valid register content and their meaning
 */
#define REGVALUE_ISOSTATE_A  0x000A
#define REGVALUE_ISOSTATE_B  0x000B
#define REGVALUE_ISOSTATE_C  0x000C
#define REGVALUE_ISOSTATE_1B 0x001B

#define REGVAL_MODE_13A 0x0013
#define REGVAL_MODE_MAX 0x0000
#define REGVAL_MODE_ERR 0x000E


#endif // __EV_SIM_H__
