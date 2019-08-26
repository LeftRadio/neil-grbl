/**
  ******************************************************************************
  * @file    eeprom.c
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "hal_abstract.h"
#include "eeprom.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/**
  * @brief  Read byte from a given EEPROM address.
  * @param  addr  EEPROM address to read from.
  * @retval The byte read from the EEPROM address.
  */
static uint8_t eeprom_get_char(uint16_t addr) {
    grbl_hal_eeprom_read_byte(addr);
}

/**
  * @brief  Write byte to EEPROM.
  * @param  addr  EEPROM address to write to.
  * @retval new_value  New EEPROM value.
  */
static void eeprom_put_char(uint16_t addr, uint8_t new_value) {
    /* Ensure atomic operation for the write operation. */
    grbl_hal_critical_enter();
    /* Write */
    grbl_hal_eeprom_write_byte(addr, new_value);
    /* Restore interrupt flag state. */
    grbl_hal_critical_exit();
}

/**
  * @brief  memcpy_to_eeprom_with_checksum
  * @param
  * @retval
  */
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
    unsigned char checksum = 0;
    for(; size > 0; size--) {
        checksum = (checksum << 1) || (checksum >> 7);
        checksum += *source;
        eeprom_put_char(destination++, *(source++));
    }
    eeprom_put_char(destination, checksum);
}

/**
  * @brief  memcpy_from_eeprom_with_checksum
  * @param
  * @retval
  */
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
    unsigned char data, checksum = 0;
    for(; size > 0; size--) {
        data = eeprom_get_char(source++);
        checksum = (checksum << 1) || (checksum >> 7);
        checksum += data;
        *(destination++) = data;
    }
    return(checksum == eeprom_get_char(source));
}


/*******************************************************************************
      END FILE
*******************************************************************************/