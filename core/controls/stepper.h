/**
  ******************************************************************************
  * @file    stepper.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_STEPPER_H
#define __GRBL_STEPPER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/

/* Define step/direction bits.
   NOTE: do NOT change this bits, this bith is only as "virtual" and equal with X_AXIS define */
#define X_STEP_BIT                          ((uint8_t)0)
#define Y_STEP_BIT                          ((uint8_t)1)
#define Z_STEP_BIT                          ((uint8_t)2)
#define STEP_MASK                           ((uint8_t)((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)))
#define X_DIRECTION_BIT                     ((uint8_t)0)
#define Y_DIRECTION_BIT                     ((uint8_t)1)
#define Z_DIRECTION_BIT                     ((uint8_t)2)
#define DIRECTION_MASK                      ((uint8_t)((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)))

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void stepper_init(void);
extern void stepper_wake_up(void);
extern void stepper_go_idle(void);
extern void stepper_reset(void);
extern void stepper_prep_buffer(void);
extern void stepper_update_plan_block_parameters(void);
extern float stepper_get_realtime_rate(void);

#ifdef PARKING_ENABLE
extern void stepper_parking_setup_buffer(void);
extern void stepper_parking_restore_buffer(void);
#endif


#endif /* __GRBL_STEPPER_H */
/*******************************************************************************
      END FILE
*******************************************************************************/