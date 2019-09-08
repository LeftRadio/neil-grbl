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
#define __weak          __attribute__((weak))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/* CRITICAL SECTION ----------------------------------------------------------*/
__weak void ngrbl_hal_critical_enter(void) { /* */ }
__weak void ngrbl_hal_critical_exit(void) { /* */ }

/* INTERRUPT -----------------------------------------------------------------*/
__weak void ngrbl_hal_enable_interrupts(void) { /* */ }
__weak void ngrbl_hal_disable_interrupts(void) { /* */ }

/* STEPPER ------------------------------------------------------------------*/
__weak void ngrbl_hal_stepper_init(void) { /* */ }
__weak void ngrbl_hal_stepper_set_driver_state(ngrbl_hal_state_t state) { /* */ }
__weak void ngrbl_hal_stepper_set_dir(uint8_t dir_mask, uint8_t dir_bits) { /* */ }
__weak void ngrbl_hal_stepper_set_step(uint8_t dir_mask, uint8_t step_bits) { /* */ }
__weak void ngrbl_hal_stepper_timer_base_init(float usec) { /* */ }
__weak void ngrbl_hal_stepper_timer_base_stop(void) { /* */ }
__weak void ngrbl_hal_stepper_timer_base_set_reload(uint32_t val) { /* */ }
__weak void ngrbl_hal_stepper_timer_base_set_prescaler(uint32_t val) { /* */ }
__weak void ngrbl_hal_stepper_timer_base_irq_start(void) { /* */ }
__weak void ngrbl_hal_stepper_timer_pulse_init(float usec) { /* */ }
__weak void ngrbl_hal_stepper_timer_pulse_stop(void) { /* */ }
__weak void ngrbl_hal_stepper_timer_pulse_set_reload(uint32_t val) { /* */ }
__weak void ngrbl_hal_stepper_timer_pulse_set_prescaler(uint32_t val) { /* */ }
__weak void ngrbl_hal_stepper_timer_pulse_set_compare(uint32_t val) { /* */ }
__weak void ngrbl_hal_stepper_timer_pulse_irq_start(void) { /* */ }

/* SPINDLE -------------------------------------------------------------------*/
__weak void ngrbl_hal_spindle_init(ngrbl_hal_spindle_mode_t pwm_mode) { /* */ }
__weak uint8_t ngrbl_hal_spindle_get_state(void) { return 0; }
__weak void ngrbl_hal_spindle_start(void) { /* */ }
__weak void ngrbl_hal_spindle_stop(void) { /* */ }
__weak void ngrbl_hal_spindle_set_pwm(uint8_t val) { /* */ }

/* COOLANT -------------------------------------------------------------------*/
__weak void ngrbl_hal_coolant_init(void) { /* */ }
__weak void ngrbl_hal_coolant_start(ngrbl_hal_coolant_flood_mist_t flood_mist) { /* */ }
__weak void ngrbl_hal_coolant_stop(ngrbl_hal_coolant_flood_mist_t flood_mist) { /* */ }
__weak ngrbl_hal_state_t ngrbl_hal_coolant_get_state(ngrbl_hal_coolant_flood_mist_t flood_mist) { return NGRBL_HAL_DISABLE; }

/* SYS CONTROL ---------------------------------------------------------------*/
__weak void ngrbl_hal_sys_control_init(void) { /* */ }
__weak uint8_t ngrbl_hal_sys_control_get_state(void) { return 0; }

/* LIMITS --------------------------------------------------------------------*/
__weak void ngrbl_hal_limits_init(void) { /* */ }
__weak void ngrbl_hal_limits_set_state(ngrbl_hal_state_t state) { /* */ }
__weak uint8_t ngrbl_hal_limits_get_state(void) { return 0; }
__weak void ngrbl_hal_limits_disable(void) { /* */ }

/* PROBES --------------------------------------------------------------------*/
__weak void ngrbl_hal_probe_init(void) { /* */ }
__weak uint8_t ngrbl_hal_probe_get_state(void) { return 0; }

/* EEPROM --------------------------------------------------------------------*/
__weak void ngrbl_hal_eeprom_init(void) { /* */ }
__weak uint8_t ngrbl_hal_eeprom_read_byte(uint16_t addr) { return 0; }
__weak void ngrbl_hal_eeprom_write_byte(uint16_t addr, uint8_t new_value) { /* */ }

/* SERIAL --------------------------------------------------------------------*/
__weak void ngrbl_hal_serail_init(uint32_t baudrate) { /* */ }
__weak void ngrbl_hal_serial_write_byte(uint8_t data) { /* */ }
__weak void ngrbl_hal_serail_stop_tx(void) { /* */ }

/* UTILS ---------------------------------------------------------------------*/
__weak void ngrbl_hal_delay_ms(uint16_t val) { /* */ }


/******************************************************************************
      END FILE
******************************************************************************/