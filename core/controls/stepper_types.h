/**
  ******************************************************************************
  * @file    stepper_types.h
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_STEPPER_TYPES_H
#define __GRBL_STEPPER_TYPES_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "planner.h"
#include "settings.h"
#include "config.h"
#include "nuts_bolts.h"

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/

/* Stores the one block of Bresenham algorithm execution data */
typedef struct _st_block_t {
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    uint8_t direction_bits;
    /* tracks motions that require constant laser power/rate */
    #ifdef VARIABLE_SPINDLE
      uint8_t is_pwm_rate_adjusted;
    #endif
} st_block_t;

/* Stores the planner blocks for the segments in the segment buffer.
   Normally, this buffer is partially in-use, but, for the worst case scenario, it will
   never exceed the number of accessible stepper buffer segments (STEPPER_SEGMENT_BUFFER_SIZE-1).
   NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
   discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
   data for its own use. */
typedef struct _st_block_buffer_t {
    st_block_t buffer[STEPPER_SEGMENT_BUFFER_SIZE-1];
    /* Pointers for the step segment being prepped from the planner buffer.
       Accessed only by the main program. Pointers may be planning segments
       or planner blocks ahead of what being executed */
    plan_block_t *pl_block;     // planner block being prepped
    st_block_t *st_prep_block;  // stepper block data being prepped
} st_block_buffer_t;

/* Step one segment type */
typedef struct _segment_t {
    uint16_t n_step;           // Number of step events to be executed for this segment
    uint16_t cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
    uint8_t  st_block_index;   // Stepper block data index. Uses this information to execute this segment.
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
    #else
      uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
    #endif
    #ifdef VARIABLE_SPINDLE
      uint8_t spindle_pwm;
    #endif
} segment_t;

/* Primary stepper segments ring buffer. Contains small, short line segments for the stepper
   algorithm to execute, which are "checked-out" incrementally from the first block in the
   planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
   the planner, where the remaining planner block steps still can. */
typedef struct _segments_buf_t {
    segment_t buffer[STEPPER_SEGMENT_BUFFER_SIZE];
    uint8_t head;
    uint8_t next_head;
    volatile uint8_t tail;
} segments_t;

/* Stepper data struct. Contains the running data for the main stepper ISR */
typedef struct _stepper_t {
    /* Used by the bresenham line algorithm */
    uint32_t counter_x, counter_y, counter_z;
    #ifdef STEP_PULSE_DELAY
      /* Stores out_bits output to complete the step pulse delay */
      uint16_t step_delay_bits;
    #endif
    uint8_t execute_step;         // Flags step execution for each interrupt.
    uint16_t step_pulse_time;     // Step pulse reset time after step rise
    uint8_t step_outbits;         // The next stepping-bits to be output
    uint8_t dir_outbits;
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      uint32_t steps[N_AXIS];
    #endif
    uint16_t step_count;          // Steps remaining in line segment motion
    uint8_t exec_block_index;     // Tracks the current st_block index. Change indicates new block.
    st_block_t *exec_block;       // Pointer to the block data for the segment being executed
    segment_t *exec_segment;      // Pointer to the segment being executed
    volatile bool_g busy;           // Used to avoid ISR nesting of the "Stepper Driver Interrupt"
} stepper_t;

/* segment preparation data struct. Contains all the necessary information
   to compute new segments based on the current executing planner block */
typedef struct _st_prep_t {
    uint8_t st_block_index;  // Index of stepper common data block being prepped
    uint8_t recalculate_flag;

    float dt_remainder;
    float steps_remaining;
    float step_per_mm;
    float req_mm_increment;

    #ifdef PARKING_ENABLE
      uint8_t last_st_block_index;
      float last_steps_remaining;
      float last_step_per_mm;
      float last_dt_remainder;
    #endif

    uint8_t ramp_type;      // Current segment ramp state
    float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                            // NOTE: This value must coincide with a step(no mantissa) when converted.
    float current_speed;    // Current speed at the end of the segment buffer (mm/min)
    float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
    float exit_speed;       // Exit speed of executing block (mm/min)
    float accelerate_until; // Acceleration ramp end measured from end of block (mm)
    float decelerate_after; // Deceleration ramp start measured from end of block (mm)

    #ifdef VARIABLE_SPINDLE
      float inv_rate;    // Used by PWM laser mode to speed up segment calculations.
      uint8_t current_spindle_pwm;
    #endif
} st_prep_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/


#endif /* __GRBL_STEPPER_TYPES_H */
/*******************************************************************************
      END FILE
*******************************************************************************/