/**
  ******************************************************************************
  * @file    eeprom.h
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM__H
#define __EEPROM__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
//extern unsigned char eeprom_get_char(unsigned int addr);
//extern void eeprom_put_char(unsigned int addr, unsigned char new_value);
extern void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
extern int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);


#endif /* __EEPROM__H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/