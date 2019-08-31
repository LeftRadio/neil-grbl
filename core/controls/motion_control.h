/**
  ******************************************************************************
  * @file    motion_control.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_MOTION_CONTROL_H
#define __GRBL_MOTION_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "planner.h"

/* Exported define -----------------------------------------------------------*/
/* System motion commands must have a line number of zero */
#define HOMING_CYCLE_LINE_NUMBER 0
#define PARKING_MOTION_LINE_NUMBER 0
/* */
#define HOMING_CYCLE_ALL  0  // Must be zero.
#define HOMING_CYCLE_X    bit(X_AXIS)
#define HOMING_CYCLE_Y    bit(Y_AXIS)
#define HOMING_CYCLE_Z    bit(Z_AXIS)

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void mc_line(float *target, plan_line_data_t *pl_data);
extern void mc_arc(
    float *target, plan_line_data_t *pl_data,
    float *position, float *offset, float radius,
    uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc
);
extern void mc_dwell(float seconds);
extern void mc_homing_cycle(uint8_t cycle_mask);
extern uint8_t mc_probe_cycle(float *target, plan_line_data_t *pl_data, uint8_t parser_flags);
extern void mc_override_ctrl_update(uint8_t override_state);
extern void mc_parking_motion(float *parking_target, plan_line_data_t *pl_data);
extern void mc_reset(void);


#endif /* __GRBL_MOTION_CONTROL_H */
/*******************************************************************************
      END FILE
*******************************************************************************/