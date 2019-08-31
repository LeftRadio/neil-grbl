/**
  ******************************************************************************
  * @file    jog.c
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "jog.h"
#include "settings.h"
#include "system.h"
#include "report.h"
#include "motion_control.h"
#include "stepper.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/**
  * @brief  Sets up valid jog motion received from g-code parser,
            checks for soft-limits, and executes the jog.
  * @param  plan_line_data_t *pl_data, parser_block_t *gc_block
  * @retval uint8_t STATUS_OK or STATUS_TRAVEL_EXCEEDED
  */
uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block) {
  /* Initialize planner data struct for jogging motions.
     NOTE: Spindle and coolant are allowed to fully function with overrides during a jog.
  */
  pl_data->feed_rate = gc_block->values.f;
  pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_block->values.n;
  #endif
  /* */
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) {
      if (system_check_travel_limits(gc_block->values.xyz)) {
          return STATUS_TRAVEL_EXCEEDED;
      }
  }
  /* valid jog command. Plan, set state, and execute */
  mc_line(gc_block->values.xyz,pl_data);
  if (sys.state == STATE_IDLE) {
      /* Check if there is a block to execute */
      if ( plan_get_current_block() != (void*)0 ) {
          sys.state = STATE_JOG;
          stepper_prep_buffer();
          /* NOTE: Manual start. No state machine required */
          stepper_wake_up();
      }
  }
  /* all is OK */
  return STATUS_OK;
}


/******************************************************************************
      END FILE
******************************************************************************/