/**
  ******************************************************************************
  * @file    coolant_control.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_COOLANT_CONTROL_H
#define __GRBL_COOLANT_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
#define COOLANT_STATE_DISABLE       0  // Must be zero
#define COOLANT_STATE_FLOOD         PL_COND_FLAG_COOLANT_FLOOD
#define COOLANT_STATE_MIST          PL_COND_FLAG_COOLANT_MIST

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void coolant_init(void);
extern uint8_t coolant_get_state(void);
extern void coolant_stop(void);
extern void coolant_set_state(uint8_t mode);
extern void coolant_sync(uint8_t mode);


#endif /* __GRBL_COOLANT_CONTROL_H */
/*******************************************************************************
      END FILE
*******************************************************************************/