/**
  ******************************************************************************
  * @file    settings.h
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRBL_SETTINGS_H
#define __GRBL_SETTINGS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "nuts_bolts.h"

/* Exported define -----------------------------------------------------------*/
#ifndef STEPPER_SEGMENT_BUFFER_SIZE
  #define STEPPER_SEGMENT_BUFFER_SIZE 6
#endif

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 10  // NOTE: Check settings_reset() when moving to next version.

// Define bit flag masks for the boolean settings in settings.flag.
#define BIT_REPORT_INCHES      0
#define BIT_LASER_MODE         1
#define BIT_INVERT_ST_ENABLE   2
#define BIT_HARD_LIMIT_ENABLE  3
#define BIT_HOMING_ENABLE      4
#define BIT_SOFT_LIMIT_ENABLE  5
#define BIT_INVERT_LIMIT_PINS  6
#define BIT_INVERT_PROBE_PIN   7

#define BITFLAG_REPORT_INCHES      bit(BIT_REPORT_INCHES)
#define BITFLAG_LASER_MODE         bit(BIT_LASER_MODE)
#define BITFLAG_INVERT_ST_ENABLE   bit(BIT_INVERT_ST_ENABLE)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(BIT_HARD_LIMIT_ENABLE)
#define BITFLAG_HOMING_ENABLE      bit(BIT_HOMING_ENABLE)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(BIT_SOFT_LIMIT_ENABLE)
#define BITFLAG_INVERT_LIMIT_PINS  bit(BIT_INVERT_LIMIT_PINS)
#define BITFLAG_INVERT_PROBE_PIN   bit(BIT_INVERT_PROBE_PIN)

// Define status reporting boolean enable bit flags in settings.status_report_mask
#define BITFLAG_RT_STATUS_POSITION_TYPE     bit(0)
#define BITFLAG_RT_STATUS_BUFFER_STATE      bit(1)

// Define settings restore bitflags.
#define SETTINGS_RESTORE_DEFAULTS bit(0)
#define SETTINGS_RESTORE_PARAMETERS bit(1)
#define SETTINGS_RESTORE_STARTUP_LINES bit(2)
#define SETTINGS_RESTORE_BUILD_INFO bit(3)
#ifndef SETTINGS_RESTORE_ALL
  #define SETTINGS_RESTORE_ALL 0xFF // All bitflags
#endif

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
#define EEPROM_ADDR_GLOBAL         1U
#define EEPROM_ADDR_PARAMETERS     512U
#define EEPROM_ADDR_STARTUP_BLOCK  768U
#define EEPROM_ADDR_BUILD_INFO     942U

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM     6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD    (N_COORDINATE_SYSTEM + 1) // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28       N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30       (N_COORDINATE_SYSTEM + 1)  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

// Define Grbl axis settings numbering scheme. Starts at START_VAL, every INCREMENT, over N_SETTINGS.
#define AXIS_N_SETTINGS          4
#define AXIS_SETTINGS_START_VAL  100 // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings


/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/

/* Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards) */
typedef struct _settings_t {
    // Axis settings
    float steps_per_mm[N_AXIS];
    float max_rate[N_AXIS];
    float acceleration[N_AXIS];
    float max_travel[N_AXIS];

    // Remaining Grbl settings
    uint8_t pulse_microseconds;
    uint8_t step_invert_mask;
    uint8_t dir_invert_mask;
    uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
    uint8_t status_report_mask; // Mask to indicate desired report data.
    float junction_deviation;
    float arc_tolerance;

    float rpm_max;
    float rpm_min;

    uint8_t flags;  // Contains default boolean settings

    uint8_t homing_dir_mask;
    float homing_feed_rate;
    float homing_seek_rate;
    uint16_t homing_debounce_delay;
    float homing_pulloff;
} settings_t;
extern settings_t settings;


/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
extern void settings_init(void);
extern void settings_restore(uint8_t restore_flag);
extern uint8_t settings_store_global_setting(uint8_t parameter, float value);
extern void settings_store_startup_line(uint8_t n, char *line);
extern uint8_t settings_read_startup_line(uint8_t n, char *line);
extern void settings_store_build_info(char *line);
extern uint8_t settings_read_build_info(char *line);
extern void settings_write_coord_data(uint8_t coord_select, float *coord_data);
extern uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);


#endif /* __GRBL_SETTINGS_H */
/******************************************************************************
      END FILE
******************************************************************************/