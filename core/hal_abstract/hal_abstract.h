/**
  ******************************************************************************
  * @file    hal_abstract.h
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NGRBL_HAL_ABSTRACT_H
#define __NGRBL_HAL_ABSTRACT_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
typedef enum {
  NGRBL_HAL_DISABLE = (uint8_t)0,
  NGRBL_HAL_ENABLE = !NGRBL_HAL_DISABLE
} ngrbl_hal_state_t;

typedef enum {
  NGRBL_HAL_COOLANT_FLOOD,
  NGRBL_HAL_COOLANT_MIST
} ngrbl_hal_coolant_flood_mist_t;

typedef enum {
  NGRBL_HAL_SPINDLE_DISCRETE_MODE,
  NGRBL_HAL_SPINDLE_PWM_MODE
} ngrbl_hal_spindle_mode_t;

typedef enum {
  STEP_PORT,
  DIRECTION_PORT,
  LIMIT_PORT,
  SPINDLE_ENABLE_PORT,
  SPINDLE_DIRECTION_PORT,
  COOLANT_FLOOD_PORT,
  COOLANT_MIST_PORT,
  CONTROL_PORT,
  PROBE_PORT,
  SPINDLE_PWM_PORT
} ngrbl_hal_port_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/

/* CRITICAL SECTION ----------------------------------------------------------*/
void ngrbl_hal_critical_enter(void);
void ngrbl_hal_critical_exit(void);


/* INTERRUPT SECTION ---------------------------------------------------------*/
void ngrbl_hal_enable_interrupts(void);
void ngrbl_hal_disable_interrupts(void);


/* STEPPER -------------------------------------------------------------------*/
void ngrbl_hal_stepper_init(void);
void ngrbl_hal_stepper_set_driver_state(ngrbl_hal_state_t state);
void ngrbl_hal_stepper_set_dir(uint8_t dir_mask, uint8_t dir_bits);
void ngrbl_hal_stepper_set_step(uint8_t dir_mask, uint8_t step_bits);
void ngrbl_hal_stepper_timer_base_init(float usec);
void ngrbl_hal_stepper_timer_base_stop(void);
void ngrbl_hal_stepper_timer_base_set_reload(uint32_t val);
void ngrbl_hal_stepper_timer_base_set_prescaler(uint32_t val);
void ngrbl_hal_stepper_timer_base_irq_start(void);
void ngrbl_hal_stepper_timer_pulse_init(float usec);
void ngrbl_hal_stepper_timer_pulse_stop(void);
void ngrbl_hal_stepper_timer_pulse_set_reload(uint32_t val);
void ngrbl_hal_stepper_timer_pulse_set_prescaler(uint32_t val);
void ngrbl_hal_stepper_timer_pulse_set_compare(uint32_t val);
void ngrbl_hal_stepper_timer_pulse_irq_start(void);
/* HAL callbacks */
extern void ngrbl_stepper_timer_base_irq_callback(void);
extern void ngrbl_stepper_timer_reset_irq_callback(void);


/* SPINDLE -------------------------------------------------------------------*/
void ngrbl_hal_spindle_init(ngrbl_hal_spindle_mode_t pwm_mode);
uint8_t ngrbl_hal_spindle_get_state(void);
void ngrbl_hal_spindle_start(void);
void ngrbl_hal_spindle_stop(void);
void ngrbl_hal_spindle_set_pwm(uint8_t val);


/* COOLANT -------------------------------------------------------------------*/
void ngrbl_hal_coolant_init(void);
void ngrbl_hal_coolant_start(ngrbl_hal_coolant_flood_mist_t flood_mist);
void ngrbl_hal_coolant_stop(ngrbl_hal_coolant_flood_mist_t flood_mist);
ngrbl_hal_state_t ngrbl_hal_coolant_get_state(ngrbl_hal_coolant_flood_mist_t flood_mist);


/* SYS CONTROL ---------------------------------------------------------------*/
void ngrbl_hal_sys_control_init(void);
uint8_t ngrbl_hal_sys_control_get_state(void);
/* HAL callbacks */
extern void ngrbl_sys_control_state_change_callback(uint8_t state);

/* LIMITS --------------------------------------------------------------------*/
void ngrbl_hal_limits_init(void);
void ngrbl_hal_limits_set_state(ngrbl_hal_state_t state);
uint8_t ngrbl_hal_limits_get_state(void);
void ngrbl_hal_limits_disable(void);
/* HAL callbacks */
extern void ngrbl_limits_state_change_callback(uint8_t state);


/* PROBES --------------------------------------------------------------------*/
void ngrbl_hal_probe_init(void);
uint8_t ngrbl_hal_probe_get_state(void);


/* EEPROM --------------------------------------------------------------------*/
uint8_t ngrbl_hal_eeprom_read_byte(uint16_t addr);
void ngrbl_hal_eeprom_write_byte(uint16_t addr, uint8_t new_value);


/* SERIAL --------------------------------------------------------------------*/
void ngrbl_hal_serail_init(uint32_t baudrate);
void ngrbl_hal_serial_write_byte(uint8_t data);
void ngrbl_hal_serail_stop_tx(void);

extern void ngrbl_hal_serial_tx_callback(void);
extern void ngrbl_hal_serial_rx_callback(uint8_t* data, uint8_t length);


/* UTILS ---------------------------------------------------------------------*/
void ngrbl_hal_delay_ms(uint16_t val);


#endif /* __GRBL_HAL__H */
/******************************************************************************
      END FILE
******************************************************************************/