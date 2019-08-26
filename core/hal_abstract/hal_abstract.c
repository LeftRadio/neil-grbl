/**
  ******************************************************************************
  * @file    hal_abstract.c
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "hal_abstract.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/* CRITICAL SECTION callbacks ------------------------------------------------*/
/**
  * @brief  grbl_hal_critical_enter
  * @param  None
  * @retval None
  */
__weak void grbl_hal_critical_enter(void) {
    (void);
}

/**
  * @brief  grbl_hal_critical_exit
  * @param  None
  * @retval None
  */
__weak void grbl_hal_critical_exit(void); {
    (void);
}


/* EEPROM callbacks ----------------------------------------------------------*/
/**
  * @brief  Read byte from a given EEPROM address.
  * @param  addr  EEPROM address to read from.
  * @retval The byte read from the EEPROM address.
  */
__weak uint8_t grbl_hal_eeprom_read_byte(uint16_t addr) {
    return 0;
}

/**
  * @brief  Read byte from a given EEPROM address.
  * @param  addr  EEPROM address to read from.
  * @retval The byte read from the EEPROM address.
  */
__weak void grbl_hal_eeprom_write_byte(uint16_t addr, uint8_t new_value) {
    return 0;
}


/* SERIAL callbacks ----------------------------------------------------------*/
/**
  * @brief  Serail port init
  * @param  baudrate value
  * @retval None
  */
__weak void grbl_hal_serail_init(uint32_t baudrate) {
    (void);
}

/**
  * @brief  Writes one byte to the TX serial buffer.
  * @param  Write byte data
  * @retval None
  */
__weak void grbl_hal_serial_write_byte(uint8_t data) {

}

/**
  * @brief  Stop transmmit over serial.
  * @param  None
  * @retval None
  */
__weak void grbl_hal_serail_stop_tx(uint8_t data) {

}


/* STEPPER callbacks ---------------------------------------------------------*/
/**
  * @brief  grbl_hal_stepper_gpio_init
  * @param  None
  * @retval None
  */
void grbl_hal_stepper_gpio_init(void) {

}

/**
  * @brief  grbl_hal_stepper_timer_base_init
  * @param  None
  * @retval None
  */
void grbl_hal_stepper_timer_base_init(float usec) {

}

/**
  * @brief  grbl_hal_stepper_timer_reset_init
  * @param  None
  * @retval None
  */
void grbl_hal_stepper_timer_reset_init(float usec) {

}


/*******************************************************************************
      END FILE
*******************************************************************************/