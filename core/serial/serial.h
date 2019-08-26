/**
  ******************************************************************************
  * @file    serial.h
  * @author  leftradio
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
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #ifdef USE_LINE_NUMBERS
    #define TX_BUFFER_SIZE 112
  #else
    #define TX_BUFFER_SIZE 104
  #endif
#endif

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
extern uint8_t serial_get_rx_buffer_count(void);
extern uint8_t serial_get_tx_buffer_count(void);

#endif /* __GRBL_SERIAL__H */

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/