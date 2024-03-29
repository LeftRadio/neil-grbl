/**
  ******************************************************************************
  * @file    serial.c
  * @author
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "system.h"
#include "config.h"
#include "serial.h"
#include "motion_control.h"
#include "hal_abstract.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef RX_BUFFER_SIZE

#endif

#define RX_RING_BUFFER (RX_BUFFER_SIZE + 1)
// #define TX_RING_BUFFER (TX_BUFFER_SIZE + 1)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/**
  * @brief  Returns the number of bytes available in the RX serial buffer.
  * @param  None
  * @retval The number of bytes available in the RX serial buffer.
  */
uint8_t serial_get_rx_buffer_available(void) {
    /* Copy to limit multiple calls to volatile */
    uint8_t rtail = serial_rx_buffer_tail;
    if (serial_rx_buffer_head >= rtail) {
        return (RX_BUFFER_SIZE - (serial_rx_buffer_head - rtail));
    }
    return (rtail - serial_rx_buffer_head - 1);
}

/**
  * @brief  serial_init
  * @param  None
  * @retval None
  */
void serial_init(void) {
    ngrbl_hal_serail_init(BAUD_RATE);
}

/**
  * @brief  Writes one byte to the TX serial buffer. Called by main program.
  * @param  data byte
  * @retval None
  */
void serial_write(uint8_t data) {
    ngrbl_hal_serial_write_byte(data);
}

/**
  * @brief  Fetches the first byte in the serial read buffer. Called by main program.
  * @param  None
  * @retval data byte
  */
uint8_t serial_read(void) {
    /* Temporary serial_rx_buffer_tail (to optimize for volatile) */
    uint8_t tail = serial_rx_buffer_tail;
    uint8_t data;
    /* */
    if (serial_rx_buffer_head == tail) {
        return SERIAL_NO_DATA;
    }
    else {
        data = serial_rx_buffer[tail];
        tail++;
        if (tail == RX_RING_BUFFER) {
            tail = 0;
        }
        serial_rx_buffer_tail = tail;
        return data;
    }
}

/**
  * @brief  ngrbl_hal_serial_rx_callback
  * @param  rx byte data
  * @retval None
  */
void serial_reset_read_buffer(void) {
    serial_rx_buffer_tail = serial_rx_buffer_head;
}


/* RX/TX callback function ---------------------------------------------------*/

/**
  * @brief  _store_handle_data_in
  * @param  rx byte data
  * @retval None
  */
static void _store_handle_data_in(uint8_t data) {
    /* */
    uint8_t next_head;

    /* Pick off realtime command characters directly from the serial stream. These characters are
       not passed into the main buffer, but these set system state flag bits for realtime execution.
    */
    switch (data) {

        /* Call motion control reset routine.*/
        case CMD_RESET:
            mc_reset();
        break;
        /* Set as true */
        case CMD_STATUS_REPORT:
            system_set_exec_state_flag(EXEC_STATUS_REPORT);
        break;
        /* Set as true */
        case CMD_CYCLE_START:
            system_set_exec_state_flag(EXEC_CYCLE_START);
        break;
        /* Set as true */
        case CMD_FEED_HOLD:
            system_set_exec_state_flag(EXEC_FEED_HOLD);
        break;
        /* */
        default:
            if (data > 0x7F) { // Real-time control characters are extended ACSII only.
                switch(data) {
                    /* Set as true */
                    case CMD_SAFETY_DOOR: system_set_exec_state_flag(EXEC_SAFETY_DOOR); break;
                    /* Block all other states from invoking motion cancel. */
                    case CMD_JOG_CANCEL:
                        if (sys.state & STATE_JOG) {
                            system_set_exec_state_flag(EXEC_MOTION_CANCEL);
                        }
                    break;
                    /* debug */
                    #ifdef DEBUG
                    case CMD_DEBUG_REPORT: { uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg; } break;
                    #endif
                    /* */
                    case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
                    case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
                    case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
                    case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
                    case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
                    case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
                    case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
                    case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
                    case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
                    case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
                    case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
                    case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
                    case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
                    case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
                    case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
                    #ifdef ENABLE_M7
                      case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
                    #endif
                }
                // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
            }
            else {
                /* Write character to buffer */
                next_head = serial_rx_buffer_head + 1;
                if (next_head == RX_RING_BUFFER) {
                    next_head = 0;
                }
                /* Write data to buffer unless it is full. */
                if (next_head != serial_rx_buffer_tail) {
                    serial_rx_buffer[serial_rx_buffer_head] = data;
                    serial_rx_buffer_head = next_head;
                }
            }
        break; // default
    }
}

/**
  * @brief  ngrbl_hal_serial_rx_callback
  * @param  rx byte data
  * @retval None
  */
void ngrbl_hal_serial_rx_callback(uint8_t* data, uint16_t length) {
    uint16_t i = 0;
    while (i != length) {
        _store_handle_data_in(data[i]);
        i++;
    }
}


/*******************************************************************************
      END FILE
*******************************************************************************/