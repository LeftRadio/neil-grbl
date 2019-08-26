/*
  ******************************************************************************
  * @file     nuts_bolts.c
  * @author   leftradio
  * @version  1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "grbl.h"
#include "nuts_bolts.h"
#include "hal_abstract.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_INT_DIGITS        8 // Maximum number of digits in int32 (and float)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Private Functions ---------------------------------------------------------*/
/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Extracts a floating point value from a string. The following code is based loosely on
            the avr-libc strtod() function by Michael Stumpf and Dmitry Xmelkov and many freely
            available conversion method examples, but has been highly optimized for Grbl. For known
            CNC applications, the typical decimal value is expected to be in the range of E0 to E-4.
            Scientific notation is officially not supported by g-code, and the 'E' character may
            be a g-code word on some CNC systems. So, 'E' notation will not be recognized.
            NOTE: Thanks to Radu-Eosif Mihailescu for identifying the issues with using strtod().
  * @param  char *line, uint8_t *char_counter, float *float_ptr
  * @retval uint8_t value
  */
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr) {
    char *ptr = line + *char_counter;
    unsigned char c;

    /* grab first character and increment pointer, no spaces assumed in line */
    c = *ptr++;
    /* capture initial positive/minus character */
    bool isnegative = false;
    if (c == '-') {
      isnegative = true;
      c = *ptr++;
    } else if (c == '+') {
      c = *ptr++;
    }
    /* extract number into fast integer, track decimal in terms of exponent value */
    uint32_t intval = 0;
    int8_t exp = 0;
    uint8_t ndigit = 0;
    bool isdecimal = false;
    while(1) {
        c -= '0';
        if (c <= 9) {
            ndigit++;
            if (ndigit <= MAX_INT_DIGITS) {
                if (isdecimal) exp--;
                intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
            }
            else {
                if (!(isdecimal)) exp++;  // drop overflow digits
            }
        }
        else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) {
            isdecimal = true;
        }
        else {
            break;
        }
        c = *ptr++;
    }

    /* return if no digits have been read */
    if (!ndigit) { return(false); };
    /* convert integer into floating point */
    float fval;
    fval = (float)intval;
    /* apply decimal, should perform no more than two floating point multiplications
       for the expected range of E0 to E-4 */
    if (fval != 0) {
        while (exp <= -2) {
            fval *= 0.01;
            exp += 2;
        }
        if (exp < 0) {
            fval *= 0.1;
        }
        else if (exp > 0) {
            do {
                fval *= 10.0;
            } while (--exp > 0);
        }
    }
    /* assign floating point value with correct sign */
    if (isnegative) {
        *float_ptr = -fval;
    }
    else {
        *float_ptr = fval;
    }
    /* set char_counter to next statement */
    *char_counter = ptr - line - 1;
    /* */
    return true;
}

/**
  * @brief  simple hypotenuse computation function.
  * @param  float x, float y
  * @retval sqrt(x*x + y*y)
  */
float hypot_f(float x, float y) {
    return(sqrt(x*x + y*y));
}

/**
  * @brief  simple convert delta vector to unit vector
  * @param  float *vector
  * @retval float magnitude
  */
float convert_delta_vector_to_unit_vector(float *vector) {
    uint8_t i;
    float magnitude = 0.0;
    for (i = 0; i < N_AXIS; i++) {
        if (vector[i] != 0.0) {
            magnitude += vector[i]*vector[i];
        }
    }
    magnitude = sqrt(magnitude);
    float inv_magnitude = 1.0/magnitude;
    for (i = 0; i < N_AXIS; i++) {
        vector[i] *= inv_magnitude;
    }
    return magnitude;
}

/**
  * @brief  simple limit value by axis maximum
  * @param  float *max_value, float *unit_vec
  * @retval float limit_value
  */
float limit_value_by_axis_maximum(float *max_value, float *unit_vec) {
    float limit_value = SOME_LARGE_VALUE;
    for (uint8_t i = 0; i < N_AXIS; i++) {
        if (unit_vec[i] != 0) {
            limit_value = min( limit_value, fabs(max_value[i] / unit_vec[i]) );
        }
    }
    return limit_value;
}


/******************************************************************************
      END FILE
******************************************************************************/