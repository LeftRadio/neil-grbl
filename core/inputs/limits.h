/**
  ******************************************************************************
  * @file    limits.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_LIMITS_H
#define __GRBL_LIMITS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void limits_init(void);
extern void limits_state(uint8_t state);
extern uint8_t limits_get_state(void);
extern void limits_go_home(uint8_t cycle_mask);
extern void limits_soft_check(float *target);


#endif /* __GRBL_LIMITS_H */
/******************************************************************************
      END FILE
******************************************************************************/