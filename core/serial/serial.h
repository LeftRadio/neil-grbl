/**
  ******************************************************************************
  * @file    serial.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_SERIAL__H
#define __GRBL_SERIAL__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
#define RX_BUFFER_SIZE                ((uint8_t)128)
#define SERIAL_NO_DATA 0xff

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void serial_init(void);
extern void serial_write(uint8_t data);
extern uint8_t serial_read(void);
extern void serial_reset_read_buffer(void);
extern uint8_t serial_get_rx_buffer_available(void);


#endif /* __GRBL_SERIAL__H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/