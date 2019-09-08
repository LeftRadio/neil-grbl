/**
  ******************************************************************************
  * @file    eeprom.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_EEPROM__H
#define __GRBL_EEPROM__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void eeprom_init(void);
extern uint8_t eeprom_get_char(uint16_t addr);
extern void eeprom_put_char(uint16_t addr, uint8_t new_value);
extern void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
extern int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);


#endif /* __EEPROM__H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/