/*
  ******************************************************************************
  * @file     coolant_control.c
  * @author   leftradio
  * @version  1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "grbl.h"
#include "limits.h"
#include "hal_abstract.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Private Functions ---------------------------------------------------------*/
/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Initialize and start the coolant subsystem
  * @param  None
  * @retval None
  */
void coolant_init(void) {
    grbl_hal_gpio_init(COOLANT_FLOOD_PORT, PUSH_PULL);
    grbl_hal_gpio_init(COOLANT_MIST_PORT, PUSH_PULL);
    coolant_stop();
}

/**
  * @brief  Returns current coolant output state. Overrides may alter it from programmed state.
  * @param  None
  * @retval 8 bit state value
  */
uint8_t coolant_get_state(void) {
    uint8_t cl_state = COOLANT_STATE_DISABLE;
    uint8_t flood_port_val = grbl_hal_gpio_get_port(COOLANT_FLOOD_PORT, (1 << COOLANT_FLOOD_BIT));
    uint8_t mist_port_val = grbl_hal_gpio_get_port(COOLANT_MIST_PORT, (1 << COOLANT_MIST_BIT));

    /* */
    #ifdef INVERT_COOLANT_FLOOD_PIN
      /* */
      if ( flood_port_val ) {
    #else
      if ( flood_port_val ) {
    #endif
      cl_state |= COOLANT_STATE_FLOOD;
    }
    #ifdef ENABLE_M7
      /* */
      #ifdef INVERT_COOLANT_MIST_PIN
        if ( mist_port_val) {
      #else
        if ( mist_port_val ) {
      #endif
        cl_state |= COOLANT_STATE_MIST;
      }
    #endif
    /* */
    return cl_state;
}

/**
  * @brief  Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
            an interrupt-level. No report flag set, but only called by routines that don't need it.
  * @param  None
  * @retval None
  */
void coolant_stop(void) {
    /* */
    #ifdef INVERT_COOLANT_FLOOD_PIN
      grbl_hal_gpio_set_port(COOLANT_FLOOD_PORT, 0xFF, (1 << COOLANT_FLOOD_BIT));
    #else
      grbl_hal_gpio_set_port(COOLANT_FLOOD_PORT, (1 << COOLANT_FLOOD_BIT), ~(1 << COOLANT_FLOOD_BIT));
    #endif
    #ifdef ENABLE_M7
      #ifdef INVERT_COOLANT_MIST_PIN
        grbl_hal_gpio_set_port(COOLANT_MIST_PORT, 0xFF, (1 << COOLANT_MIST_BIT));
      #else
        grbl_hal_gpio_set_port(COOLANT_MIST_PORT, (1 << COOLANT_MIST_BIT), ~(1 << COOLANT_MIST_BIT));
      #endif
    #endif
}

/**
  * @brief  Main program only. Immediately sets flood coolant running state and also mist coolant,
            if enabled. Also sets a flag to report an update to a coolant state.
            Called by coolant toggle override, parking restore, parking retract, sleep mode, g-code
            parser program end, and g-code parser coolant_sync()
  * @param  8 bit mode value
  * @retval None
  */
void coolant_set_state(uint8_t mode) {
    /* block during abort */
    if (sys.abort) { return; }

    /* */
    if (mode & COOLANT_FLOOD_ENABLE) {
        #ifdef INVERT_COOLANT_FLOOD_PIN
          grbl_hal_gpio_set_port(COOLANT_FLOOD_PORT, (1 << COOLANT_FLOOD_BIT), ~(1 << COOLANT_FLOOD_BIT));
        #else
          grbl_hal_gpio_set_port(COOLANT_FLOOD_PORT, 0xFF, (1 << COOLANT_FLOOD_BIT));
        #endif
    }
    else {
        #ifdef INVERT_COOLANT_FLOOD_PIN
          grbl_hal_gpio_set_port(COOLANT_FLOOD_PORT, 0xFF, (1 << COOLANT_FLOOD_BIT));
        #else
          grbl_hal_gpio_set_port(COOLANT_FLOOD_PORT, (1 << COOLANT_FLOOD_BIT), ~(1 << COOLANT_FLOOD_BIT));
        #endif
    }

    #ifdef ENABLE_M7
      if (mode & COOLANT_MIST_ENABLE) {
          #ifdef INVERT_COOLANT_MIST_PIN
            grbl_hal_gpio_set_port(COOLANT_MIST_PORT, (1 << COOLANT_MIST_BIT), ~(1 << COOLANT_MIST_BIT));
          #else
            grbl_hal_gpio_set_port(COOLANT_MIST_PORT, 0xFF, (1 << COOLANT_MIST_BIT));
          #endif
      }
      else {
          #ifdef INVERT_COOLANT_MIST_PIN
            grbl_hal_gpio_set_port(COOLANT_MIST_PORT, 0xFF, (1 << COOLANT_MIST_BIT));
          #else
            grbl_hal_gpio_set_port(COOLANT_MIST_PORT, (1 << COOLANT_MIST_BIT), ~(1 << COOLANT_MIST_BIT));
          #endif
      }
    #endif

    /* set to report change immediately */
    sys.report_ovr_counter = 0;
}

/**
  * @brief  G-code parser entry-point for setting coolant state, forces a planner
            buffer sync and bails if an abort or check-mode is active.
  * @param  8 bit mode value
  * @retval None
  */
void coolant_sync(uint8_t mode) {
    /* */
    if (sys.state == STATE_CHECK_MODE) {
        return;
    }
    /* ensure coolant turns on when specified in program */
    protocol_buffer_synchronize();
    coolant_set_state(mode);
}


/******************************************************************************
      END FILE
******************************************************************************/