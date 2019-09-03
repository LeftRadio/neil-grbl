/**
  ******************************************************************************
  * @file    platform_default.c
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief   This source code is a preparation for describing the behavior
  *          of the hardware level of the target platform for universal
  *          requests of the NLRBL library.
  *			 NOTE: THIS FILE IS NOT A PART OF NGRBL, IT MAY USE ONLY FOR TARGET
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "hal_abstract.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NGRBL_TIMER_BASE            (&htim2)
#define NGRBL_TIMER_PULSE           (&htim3)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/* CRITICAL SECTION ----------------------------------------------------------*/
void ngrbl_hal_critical_enter(void) { /* */ }
void ngrbl_hal_critical_exit(void) { /* */ }

/* INTERRUPT -----------------------------------------------------------------*/
void ngrbl_hal_enable_interrupts(void) { /* */ }
void ngrbl_hal_disable_interrupts(void) { /* */ }

/* STEPPER ------------------------------------------------------------------*/
void ngrbl_hal_stepper_init(void) { /* */ }
void ngrbl_hal_stepper_set_driver_state(ngrbl_hal_state_t state) { /* */ }
void ngrbl_hal_stepper_set_dir(uint8_t dir_mask, uint8_t dir_bits) { /* */ }
void ngrbl_hal_stepper_set_step(uint8_t step_mask, uint8_t step_bits) { /* */ }
void ngrbl_hal_stepper_timer_base_init(float usec) { /* */ }
void ngrbl_hal_stepper_timer_base_irq_start(void) { /* */ }
void ngrbl_hal_stepper_timer_base_stop(void) { /* */ }
void ngrbl_hal_stepper_timer_base_set_reload(uint32_t val) { /* */ }
void ngrbl_hal_stepper_timer_base_set_prescaler(uint32_t val) { /* */ }
void ngrbl_hal_stepper_timer_pulse_init(float usec) { /* */ }
void ngrbl_hal_stepper_timer_pulse_irq_start(void) { /* */ }
void ngrbl_hal_stepper_timer_pulse_stop(void) { /* */ }
void ngrbl_hal_stepper_timer_pulse_set_reload(uint32_t val) { /* */ }
void ngrbl_hal_stepper_timer_pulse_set_prescaler(uint32_t val) { /* */ }
void ngrbl_hal_stepper_timer_pulse_set_compare(uint32_t val) { /* */ }

/* SPINDLE -------------------------------------------------------------------*/
void ngrbl_hal_spindle_init(ngrbl_hal_spindle_mode_t pwm_mode) { /* */ }
uint8_t ngrbl_hal_spindle_get_state(void) { return 0; }
void ngrbl_hal_spindle_start(void) { /* */ }
void ngrbl_hal_spindle_stop(void) { /* */ }
void ngrbl_hal_spindle_set_pwm(uint8_t val) { /* */ }

/* COOLANT -------------------------------------------------------------------*/
void ngrbl_hal_coolant_init(void) { /* */ }
void ngrbl_hal_coolant_start(ngrbl_hal_coolant_flood_mist_t flood_mist) { /* */ }
void ngrbl_hal_coolant_stop(ngrbl_hal_coolant_flood_mist_t flood_mist) { /* */ }
ngrbl_hal_state_t ngrbl_hal_coolant_get_state(ngrbl_hal_coolant_flood_mist_t flood_mist) { return NGRBL_HAL_DISABLE; }

/* SYS CONTROL ---------------------------------------------------------------*/
void ngrbl_hal_sys_control_init(void) { /* */ }
uint8_t ngrbl_hal_sys_control_get_state(void) { return 0; }

/* LIMITS --------------------------------------------------------------------*/
void ngrbl_hal_limits_init(void) { /* */ }
void ngrbl_hal_limits_set_state(ngrbl_hal_state_t state) { /* */ }
uint8_t ngrbl_hal_limits_get_state(void) { return 0; }
void ngrbl_hal_limits_disable(void) { /* */ }

/* PROBES --------------------------------------------------------------------*/
void ngrbl_hal_probe_init(void) { /* */ }
uint8_t ngrbl_hal_probe_get_state(void) { return 0; }

/* EEPROM --------------------------------------------------------------------*/
uint8_t ngrbl_hal_eeprom_read_byte(uint16_t addr) { return 0; }
void ngrbl_hal_eeprom_write_byte(uint16_t addr, uint8_t new_value) { /* */ }

/* SERIAL --------------------------------------------------------------------*/
void ngrbl_hal_serail_init(uint32_t baudrate) { /* */ }
void ngrbl_hal_serial_write_byte(uint8_t data) { /* */ }
void ngrbl_hal_serail_stop_tx(void) { /* */ }

/* UTILS ---------------------------------------------------------------------*/
void ngrbl_hal_delay_ms(uint16_t val) { /* */ }


/******************************************************************************
      END FILE
******************************************************************************/