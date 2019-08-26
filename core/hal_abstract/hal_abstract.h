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
#ifndef __GRBL_HAL_ABSTRACT_H
#define __GRBL_HAL_ABSTRACT_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
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
} grbl_hal_port_t;

typedef enum { IN_PULL_UP, IN_PULL_DOWN } grbl_pin_pull_t;
typedef enum { DEFAULT, PUSH_PULL, OPEN_DRAIN, IN_IRQ_ENABLE, IN_IRQ_DISABLE } grbl_pin_irq_state_t;

/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/

/* CRITICAL SECTION ----------------------------------------------------------*/
void grbl_hal_critical_enter(void);
void grbl_hal_critical_exit(void);

/* INTERRUPT SECTION ---------------------------------------------------------*/
void grbl_hal_enable_interrupts(void);
void grbl_hal_disable_interrupts(void);

/* GPIO ----------------------------------------------------------------------*/
void grbl_hal_gpio_init(grbl_hal_port_t port, grbl_pin_pull_t pull_updown);
void grbl_hal_gpio_irq_set_state(grbl_hal_port_t port, grbl_pin_irq_state_t state);
void grbl_hal_gpio_set_pin(grbl_hal_port_t port, uint32_t pin, uint8_t value);
void grbl_hal_gpio_set_port(grbl_hal_port_t port, uint32_t mask, uint32_t value);
uint8_t grbl_hal_gpio_get_port(grbl_hal_port_t port, uint8_t mask);

/* EEPROM --------------------------------------------------------------------*/
uint8_t grbl_hal_eeprom_read_byte(uint16_t addr);
void grbl_hal_eeprom_write_byte(uint16_t addr, uint8_t new_value);

/* SERIAL --------------------------------------------------------------------*/
void grbl_hal_serail_init(uint32_t baudrate);
void grbl_hal_serial_write_byte(uint8_t data);
void grbl_hal_serail_stop_tx(void);

extern void grbl_hal_serial_tx_callback(void);
extern void grbl_hal_serial_rx_callback(uint8_t* data, uint8_t length);

/* STEPPER ------------------------------------------------------------------*/
void grbl_hal_stepper_set_state(uint8_t state);

void grbl_hal_stepper_timer_base_init(float usec);
void grbl_hal_stepper_timer_base_set_reload(uint32_t val);
void grbl_hal_stepper_timer_base_set_prescaler(uint32_t val);
void grbl_hal_stepper_timer_base_irq_start(void);

void grbl_hal_stepper_timer_pulse_init(float usec);
void grbl_hal_stepper_timer_pulse_set_reload(uint32_t val);
void grbl_hal_stepper_timer_pulse_set_prescaler(uint32_t val);
void grbl_hal_stepper_timer_pulse_set_compare(uint32_t val);
void grbl_hal_stepper_timer_pulse_irq_start(void);

extern void grbl_stepper_timer_base_irq_callback(void);
extern void grbl_stepper_timer_reset_irq_callback(void);

/* LIMITS -------------------------------------------------------------------*/
extern void grbl_limits_gpio_irq_callback(void);

/* COOLANT -------------------------------------------------------------------*/
extern void grbl_coolant_gpio_irq_callback(void);


/* UTILS --------------------------------------------------------------------*/
extern void grbl_hal_delay_ms(uint16_t val);


#endif /* __GRBL_HAL__H */
/******************************************************************************
      END FILE
******************************************************************************/