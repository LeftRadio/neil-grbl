/**
  ******************************************************************************
  * @file    print.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_PRINT_CONSOLE_H
#define __GRBL_PRINT_CONSOLE_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void printString(const char *s);
extern void printPgmString(const char *s);
extern void printInteger(int64_t n);
extern void print_uint32_base10(uint32_t n);
extern void print_uint8_base10(uint8_t n);
extern void print_uint8_base2_ndigit(uint8_t n, uint8_t digits);
extern void printFloat(float n, uint8_t decimal_places);
extern void printFloat_CoordValue(float n);
extern void printFloat_RateValue(float n);
extern void printFreeMemory();


#endif /* __GRBL_PRINT_CONSOLE_H */
/******************************************************************************
      END FILE
******************************************************************************/