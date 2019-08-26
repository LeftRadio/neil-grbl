/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl officially supports
   only the Arduino Mega328p. */


#ifndef cpu_map_h
#define cpu_map_h


/* Define step/direction bits.
   NOTE: do NOT change this bits, this bith is only as "virtual" and equal with X_AXIS define */
#define X_STEP_BIT                          ((uint8_t)0)
#define Y_STEP_BIT                          ((uint8_t)1)
#define Z_STEP_BIT                          ((uint8_t)2)
#define STEP_MASK                           ((uint8_t)((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)))
#define X_DIRECTION_BIT                     ((uint8_t)0)
#define Y_DIRECTION_BIT                     ((uint8_t)1)
#define Z_DIRECTION_BIT                     ((uint8_t)2)
#define DIRECTION_MASK                      ((uint8_t)((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)))

/* Define stepper driver enable/disable output bits */
#define STEPPERS_DISABLE_BIT                ((uint8_t)0)
#define STEPPERS_DISABLE_MASK               ((uint8_t)(1<<STEPPERS_DISABLE_BIT))

/* Define homing/hard limit switch input pins and limit interrupt vectors */
#define X_LIMIT_BIT                         ((uint8_t)0)
#define Y_LIMIT_BIT                         ((uint8_t)1)
#define Z_LIMIT_BIT                         ((uint8_t)2)
#define LIMIT_MASK                          ((uint8_t)((1 << X_LIMIT_BIT) | (1 << Y_LIMIT_BIT) | (1 << Z_LIMIT_BIT)))

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_BIT                  ((uint8_t)0)
#define SPINDLE_DIRECTION_BIT               ((uint8_t)1)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_BIT                   ((uint8_t)0)
#define COOLANT_MIST_BIT                    ((uint8_t)1)

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_RESET_BIT                   ((uint8_t)0)
#define CONTROL_FEED_HOLD_BIT               ((uint8_t)1)
#define CONTROL_CYCLE_START_BIT             ((uint8_t)2)
#define CONTROL_SAFETY_DOOR_BIT             ((uint8_t)3)
#define CONTROL_MASK                        ((uint8_t)((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT)))
#define CONTROL_INVERT_MASK                 ((uint8_t)CONTROL_MASK)

// Define probe switch input pin.
#define PROBE_BIT                           ((uint8_t)5)
#define PROBE_MASK                          ((uint8_t)(1<<PROBE_BIT))

// Variable spindle configuration below. Do not change unless you know what you are doing.
// NOTE: Only used when variable spindle is enabled.
#define SPINDLE_PWM_MAX_VALUE               ((uint8_t)255)
#define SPINDLE_PWM_MIN_VALUE               1
#define SPINDLE_PWM_OFF_VALUE               ((uint8_t)0)
#define SPINDLE_PWM_RANGE                   ((uint8_t)(SPINDLE_PWM_MAX_VALUE - SPINDLE_PWM_MIN_VALUE))

#define SPINDLE_PWM_BIT	                    ((uint8_t)0)


/*
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.
#endif
*/

#endif
