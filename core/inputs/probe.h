/**
  ******************************************************************************
  * @file    probe.h
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_PROBE_H
#define __GRBL_PROBE_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
// Values that define the probing state machine.
#define PROBE_OFF     0 // Probing disabled or not in use. (Must be zero.)
#define PROBE_ACTIVE  1 // Actively watching the input pin.

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void probe_init(void);
extern void probe_configure_invert_mask(uint8_t is_probe_away);
extern uint8_t probe_get_state(void);
extern void probe_state_monitor(void);


#endif /* __GRBL_PROBE_H */
/******************************************************************************
      END FILE
******************************************************************************/