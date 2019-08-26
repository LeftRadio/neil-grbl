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

extern void stepper_init();
extern void st_wake_up(void);
extern void st_go_idle(void);
extern void st_generate_step_dir_invert_masks(void);
extern void st_reset(void);
extern void st_parking_setup_buffer(void);
extern void st_parking_restore_buffer(void);
extern void st_prep_buffer(void);
extern void st_update_plan_block_parameters(void);
extern float st_get_realtime_rate(void);
/* */
extern uint8_t get_step_pin_mask(uint8_t axis_idx);
extern uint8_t get_direction_pin_mask(uint8_t axis_idx);
extern uint8_t get_limit_pin_mask(uint8_t axis_idx);


#endif /* __GRBL_HAL__H */
/*******************************************************************************
      END FILE
*******************************************************************************/