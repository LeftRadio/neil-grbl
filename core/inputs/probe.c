/*
  ******************************************************************************
  * @file     probe.c
  * @author   leftradio
  * @version  1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
// #include "grbl.h"
#include "probe.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "hal_abstract.h"

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
    #ifdef DISABLE_PROBE_PIN_PULL_UP
      grbl_hal_gpio_init(PROBE_PORT, PULL_DOWN);
    #else
      grbl_hal_gpio_init(PROBE_PORT, PULL_UP);
    #endif
    /* */
    probe_configure_invert_mask(false); // Initialize invert mask.
}

/**
  * @brief  Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
            appropriately set the pin logic according to setting for normal-high/normal-low operation
            and the probing cycle modes for toward-workpiece/away-from-workpiece.
  * @param  is probe away
  * @retval None
  */
void probe_configure_invert_mask(uint8_t is_probe_away) {
  /* initialize as zero */
  probe_invert_mask = 0;
  /* */
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
  if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}

/**
  * @brief  Returns the probe pin state, triggered = true,
            called by gcode parser and probe state monitor.
  * @param  is probe away
  * @retval None
  */
uint8_t probe_get_state(void) {
    /* */
    return grbl_hal_gpio_get_port(PROBE_PORT, PROBE_MASK) ^ probe_invert_mask;
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
    if ( !probe_get_state() ) return;
    /* */
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    // bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
}


/******************************************************************************
      END FILE
******************************************************************************/