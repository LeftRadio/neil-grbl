/**
  ******************************************************************************
  * @file    grbl.c
  * @author  leftradio
  * @version 1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "grbl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/* CRITICAL SECTION callbacks ------------------------------------------------*/

/**
  * @brief  grbl_init
  * @param  None
  * @retval None
  */
void grbl_init(void) {  
    /*
      Initialize system upon power-up.
    */
    
    grbl_hal_disable_interrupts();

    /* Setup serial baud rate and interrupts */
    serial_init();
    /* Load Grbl settings from EEPROM */
    settings_init();
    /* Configure stepper pins and interrupt timers */
    stepper_init();
    /* Configure pinout pins and pin-change interrupt */
    system_init();
    /* Clear machine position. */
    memset(sys_position,0,sizeof(sys_position));

    grbl_hal_enable_interrupts();

    /* Initialize system state. */
    #ifdef FORCE_INITIALIZATION_ALARM
      /* Force Grbl into an ALARM state upon a power-cycle or hard reset. */
      sys.state = STATE_ALARM;
    #else
      sys.state = STATE_IDLE;
    #endif

    /*
      Check for power-up and set system alarm if homing is enabled to force homing cycle
      by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
      startup scripts, but allows access to settings and internal commands. Only a homing
      cycle '$H' or kill alarm locks '$X' will disable the alarm.
      NOTE: The startup script will run after successful completion of the homing cycle, but
      not after disabling the alarm locks. Prevents motion startup blocks from crashing into
      things uncontrollably. Very bad.
    */
    #ifdef HOMING_INIT_LOCK
      if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) {
          sys.state = STATE_ALARM;
      }
    #endif
}


/**
  * @brief  grbl_init
  * @param  None
  * @retval None
  */
void grbl_main_loop(void) {
    /* Grbl initialization loop upon power-up or a system abort. For the latter, all processes
       will return to this loop to be cleanly re-initialized. */
    
    /* Reset system variables. */
    uint8_t prior_state = sys.state;
    /* Clear system struct variable. */
    memset(&sys, 0, sizeof(system_t));
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
    memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    /* Reset Grbl primary systems. */
    serial_reset_read_buffer();
    gc_init(); // Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    /* Sync cleared gcode and planner positions to current system position. */
    plan_sync_position();
    gc_sync_position();

    /* Print welcome message. Indicates an initialization has occured at power-up or with a reset. */
    report_init_message();

    /* Start Grbl main loop. Processes program inputs and executes them. */
    protocol_main_loop();
}


/*******************************************************************************
      END FILE
*******************************************************************************/