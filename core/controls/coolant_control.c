/*
  ******************************************************************************
  * @file     coolant_control.c
  * @author
  * @version  1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "system.h"
#include "planner.h"
#include "protocol.h"
#include "gcode.h"
#include "coolant_control.h"
#include "hal_abstract.h"
#include "nuts_bolts.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define COOLANT_NO_SYNC             false
#define COOLANT_FORCE_SYNC          true

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
    ngrbl_hal_coolant_init();
    coolant_stop();
}

/**
  * @brief  Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
            an interrupt-level. No report flag set, but only called by routines that don't need it.
  * @param  None
  * @retval None
  */
void coolant_stop(void) {
    /* */
    ngrbl_hal_coolant_stop(NGRBL_HAL_COOLANT_FLOOD);
    /**/
    #ifdef ENABLE_M7
    ngrbl_hal_coolant_stop(NGRBL_HAL_COOLANT_MIST);
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
    if (mode & COOLANT_FLOOD_ENABLE) ngrbl_hal_coolant_start(NGRBL_HAL_COOLANT_FLOOD);
    else ngrbl_hal_coolant_stop(NGRBL_HAL_COOLANT_FLOOD);
    /* */
    #ifdef ENABLE_M7
      if (mode & COOLANT_MIST_ENABLE) ngrbl_hal_coolant_start(NGRBL_HAL_COOLANT_MIST);
      else ngrbl_hal_coolant_stop(NGRBL_HAL_COOLANT_MIST);
    #endif
    /* set to report change immediately */
    sys.report_ovr_counter = 0;
}

/**
  * @brief  Returns current coolant output state. Overrides may alter it from programmed state.
  * @param  None
  * @retval 8 bit state value
  */
uint8_t coolant_get_state(void) {
    /* */
    uint8_t cl_state = COOLANT_STATE_DISABLE;
    /* */
    if (ngrbl_hal_coolant_get_state(NGRBL_HAL_COOLANT_FLOOD) != NGRBL_HAL_DISABLE) {
        cl_state |= COOLANT_STATE_FLOOD;
    }
    /* */
    #ifdef ENABLE_M7
      if (ngrbl_hal_coolant_get_state(NGRBL_HAL_COOLANT_MIST) != NGRBL_HAL_DISABLE) {
          cl_state |= COOLANT_STATE_MIST;
      }
    #endif
    /* */
    return cl_state;
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