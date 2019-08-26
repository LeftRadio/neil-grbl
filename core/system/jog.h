/**
  ******************************************************************************
  * @file    jog.h
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_JOG_H
#define __GRBL_JOG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "planner.h"
#include "gcode.h"

/* Exported define -----------------------------------------------------------*/
/* system motion line numbers must be zero */
#define JOG_LINE_NUMBER 0

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block);


#endif /* __GRBL_JOG_H */
/******************************************************************************
      END FILE
******************************************************************************/