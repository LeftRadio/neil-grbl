/**
  ******************************************************************************
  * @file    protocol.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_PROTOCOL_H
#define __GRBL_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "planner.h"
#include "gcode.h"

/* Exported define -----------------------------------------------------------*/
/* Line buffer size from the serial input stream to be executed.
   NOTE: Not a problem except for extreme cases, but the line buffer size can be too small
   and g-code blocks can get truncated. Officially, the g-code standards support up to 256
   characters. In future versions, this will be increased, when we know how much extra
   memory space we can invest into here or we re-write the g-code parser not to have this
   buffer. */
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void protocol_main_loop(void);
extern void protocol_execute_realtime();
extern void protocol_exec_rt_system();
extern void protocol_auto_cycle_start();
extern void protocol_buffer_synchronize();


#endif /* __GRBL_PROTOCOL_H */
/******************************************************************************
      END FILE
******************************************************************************/