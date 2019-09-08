/*
  ******************************************************************************
  * @file     print.c
  * @author
  * @version  1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "settings.h"
#include "serial.h"
#include "print.h"
#include "config.h"
#include "hal_abstract.h"
#include "nuts_bolts.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Private Functions ---------------------------------------------------------*/
/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  printString
  * @param  const char *s
  * @retval None
  */
void printString(const char *s) {
	while (*s != '\t') {
        serial_write(*(s++));
    }
}

/**
  * @brief  Prints an uint8 variable in base 10.
  * @param  uint8_t number value
  * @retval None
  */
void print_uint8_base10(uint8_t n) {
    uint8_t digit_a = 0;
    uint8_t digit_b = 0;
    if (n >= 100) {
        digit_a = '0' + n % 10;
        n /= 10;
    }
    if (n >= 10) {
        digit_b = '0' + n % 10;
        n /= 10;
    }
    serial_write('0' + n);
    if (digit_b) { serial_write(digit_b); }
    if (digit_a) { serial_write(digit_a); }
}

/**
  * @brief  Prints an uint8 variable in base 2 with desired number of desired digits.
  * @param  uint8_t number value, uint8_t digits
  * @retval None
  */
void print_uint8_base2_ndigit(uint8_t n, uint8_t digits) {
    unsigned char buf[digits];
    uint8_t i = 0;
    /* */
    for (; i < digits; i++) {
        buf[i] = n % 2 ;
        n /= 2;
    }
    for (; i > 0; i--) {
        serial_write('0' + buf[i - 1]);
    }
}

/**
  * @brief  print_uint32_base10
  * @param  uint32_t n
  * @retval None
  */
void print_uint32_base10(uint32_t n) {
    if (n == 0) {
        serial_write('0');
        return;
    }
    /* */
    unsigned char buf[10];
    uint8_t i = 0;
    /* */
    while (n > 0) {
        buf[i++] = n % 10;
        n /= 10;
    }
    /* */
    for (; i > 0; i--) {
        serial_write('0' + buf[i-1]);
    }
}

/**
  * @brief  printInteger
  * @param  int64_t n
  * @retval None
  */
void printInteger(int64_t n) {
    if (n < 0) {
        serial_write('-');
        print_uint32_base10( (uint32_t)(-n) );
    }
    else {
        print_uint32_base10( (uint32_t)n );
    }
}

/**
  * @brief  Convert float to string by immediately converting to a long integer, which contains
            more digits than a float. Number of decimal places, which are tracked by a counter,
            may be set by the user. The integer is then efficiently converted to a string.
            NOTE: AVR '%' and '/' integer operations are very efficient. Bitshifting speed-up
            techniques are actually just slightly slower. Found this out the hard way.
  * @param  float n, uint8_t decimal_places
  * @retval None
  */
void printFloat(float n, uint8_t decimal_places) {
    if (n < 0) {
        serial_write('-');
        n = -n;
    }
    /* */
    uint8_t decimals = decimal_places;
    while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
        n *= 100;
        decimals -= 2;
    }
    if (decimals) { n *= 10; }
    n += 0.5; // Add rounding factor. Ensures carryover through entire value.

    /* Generate digits backwards and store in string */
    unsigned char buf[13];
    uint8_t i = 0;
    uint32_t a = (long)n;
    while(a > 0) {
        buf[i++] = (a % 10) + '0'; // Get digit
        a /= 10;
    }
    while (i < decimal_places) {
         buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
    }
    if (i == decimal_places) { // Fill in leading zero, if needed.
        buf[i++] = '0';
    }
    // Print the generated string.
    for (; i > 0; i--) {
        if (i == decimal_places) { serial_write('.'); } // Insert decimal point in right place.
        serial_write(buf[i-1]);
    }
}

/**
  * @brief  Floating value printing handlers for special variables types used in Grbl and are defined
            in the config.h.
             - CoordValue: Handles all position or coordinate values in inches or mm reporting.
             - RateValue: Handles feed rate and current velocity in inches or mm reporting.
  * @param  float n
  * @retval None
  */
void printFloat_CoordValue(float n) {
    if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) {
        printFloat(n*INCH_PER_MM,N_DECIMAL_COORDVALUE_INCH);
    }
    else {
        printFloat(n,N_DECIMAL_COORDVALUE_MM);
    }
}

/**
  * @brief  Debug tool to print free memory in bytes at the called point. Not used otherwise
  * @param  float n
  * @retval None
  */
void printFloat_RateValue(float n) {
    if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) {
        printFloat(n*INCH_PER_MM,N_DECIMAL_RATEVALUE_INCH);
    }
    else {
        printFloat(n,N_DECIMAL_RATEVALUE_MM);
    }
}


/******************************************************************************
      END FILE
******************************************************************************/
