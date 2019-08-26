/**
  ******************************************************************************
  * @file    stepper.h
  * @author  leftradio
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
#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

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

#ifdef PARKING_ENABLE
extern void stepper_parking_setup_buffer(void);
extern void stepper_parking_restore_buffer(void);
#endif

extern float stepper_get_realtime_rate(void);


#endif /* __GRBL_STEPPER_H */
/*******************************************************************************
      END FILE
*******************************************************************************/