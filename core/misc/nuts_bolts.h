/**
  ******************************************************************************
  * @file    nuts_bolts.h
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_MISC_H
#define __GRBL_MISC_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
// #include "config.h"

/* Exported define -----------------------------------------------------------*/
#define false 0
#define true 1

#define SOME_LARGE_VALUE 1.0E+38

// Axis array index values. Must start with 0 and be continuous.
#define N_AXIS 				((uint8_t)3) // Number of axes
#define X_AXIS 				((uint8_t)0) // Axis indexing value.
#define Y_AXIS 				((uint8_t)1)
#define Z_AXIS 				((uint8_t)2)
// #define A_AXIS 3

// CoreXY motor assignments. DO NOT ALTER.
// NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
#ifdef COREXY
 #define A_MOTOR X_AXIS // Must be X_AXIS
 #define B_MOTOR Y_AXIS // Must be Y_AXIS
#endif

// Conversions
#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)
#define TICKS_PER_MICROSECOND (F_CPU/1000000)

#define DELAY_MODE_DWELL       0
#define DELAY_MODE_SYS_SUSPEND 1

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define isequal_position_vector(a,b) !(memcmp(a, b, sizeof(float)*N_AXIS))

// Bit field and masking macros
#define bit(n) (1 << n)
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);
extern float hypot_f(float x, float y);
extern float convert_delta_vector_to_unit_vector(float *vector);
extern float limit_value_by_axis_maximum(float *max_value, float *unit_vec);


#endif /* __GRBL_MISC_H */
/******************************************************************************
      END FILE
******************************************************************************/