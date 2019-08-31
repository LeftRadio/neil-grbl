/**
  ******************************************************************************
  * @file    spindle_control.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_SPINDLE_CONTROL_H
#define __GRBL_SPINDLE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "nuts_bolts.h"

/* Exported define -----------------------------------------------------------*/
#define SPINDLE_NO_SYNC                       false
#define SPINDLE_FORCE_SYNC                     true

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
typedef enum {
    SPINDLE_STATE_DISABLE = ((uint8_t)0), // Must be zero.
    SPINDLE_STATE_CW = bit(0),
    SPINDLE_STATE_CCW = bit(1)
} grbl_spindle_state_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void spindle_init(void);
extern uint8_t spindle_get_state(void);
extern void spindle_sync(uint8_t state, float rpm);
extern void spindle_set_state(uint8_t state, float rpm);
extern void spindle_set_speed(uint8_t pwm_value);
extern uint8_t spindle_compute_pwm_value(float rpm);
extern void spindle_stop(void);


#endif /* __GRBL_SPINDLE_CONTROL_H */
/*******************************************************************************
      END FILE
*******************************************************************************/