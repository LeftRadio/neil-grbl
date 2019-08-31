/*
  ******************************************************************************
  * @file     probe.c
  * @author
  * @version  1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "probe.h"
#include "system.h"
#include "hal_abstract.h"
#include "nuts_bolts.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Homing axis search distance multiplier. Computed by this value times the cycle travel */
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.5 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Inverts the probe pin state depending on user settings and probing cycle mode.
uint8_t probe_invert_mask;

/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Private Functions ---------------------------------------------------------*/
/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Probe pin initialization routine
  * @param  None
  * @retval None
  */
void probe_init(void) {
    /* */
    ngrbl_hal_probe_init();
}

/**
  * @brief  Returns the probe pin state, triggered = true,
            called by gcode parser and probe state monitor.
  * @param  is probe away
  * @retval None
  */
uint8_t probe_get_state(void) {
    /* */
    return ngrbl_hal_probe_get_state();
}

/**
  * @brief  Monitors probe pin state and records the system position when detected. Called by the
            stepper ISR per ISR tick.
            NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
  * @param  None
  * @retval None
  */
void probe_state_monitor(void) {
    /* */
    if ( !ngrbl_hal_probe_get_state() ) {
        return;
    }
    /* */
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
}


/******************************************************************************
      END FILE
******************************************************************************/