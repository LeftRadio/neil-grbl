/*
  ******************************************************************************
  * @file     limits.c
  * @author
  * @version  1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include "system.h"
#include "config.h"
#include "planner.h"
#include "limits.h"
#include "settings.h"
#include "stepper.h"
#include "motion_control.h"
#include "protocol.h"
#include "hal_abstract.h"
#include "nuts_bolts.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Homing axis search distance multiplier. Computed by this value times the cycle travel */
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.5 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Private Functions ---------------------------------------------------------*/
/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Initialize and start the limits subsystem
  * @param  None
  * @retval None
  */
void limits_init(void) {
    /* init limits gpio as input pins */
    ngrbl_hal_limits_init();
    /* */
    limits_state( bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE) );

    // #ifdef ENABLE_SOFTWARE_DEBOUNCE
    //   MCUSR &= ~(1<<WDRF);
    //   WDTCSR |= (1<<WDCE) | (1<<WDE);
    //   WDTCSR = (1<<WDP0); // Set time-out at ~32msec.
    // #endif
}

/**
  * @brief  Enable/Disable hard limits.
  * @param  new state, 0 - disable
  * @retval None
  */
void limits_state(uint8_t state) {
    if (state) {
        ngrbl_hal_limits_set_state(NGRBL_HAL_ENABLE);
    }
    else {
        ngrbl_hal_limits_set_state(NGRBL_HAL_DISABLE);
    }
}

/**
  * @brief  Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where
            triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
            number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
  * @param  new state, 0 - disable
  * @retval None
  */
uint8_t limits_get_state(void) {
    return ngrbl_hal_limits_get_state();
}

/**
  * @brief  Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
            completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
            the trigger point of the limit switches. The rapid stops are handled by a system level axis lock
            mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically
            circumvent the processes for executing motions in normal operation.
            NOTE: Only the abort realtime command can interrupt this process.
            TODO: Move limit pin-specific calls to a general function for portability.
  * @param  8 bit cycle mask
  * @retval None
  */
void limits_go_home(uint8_t cycle_mask) {
    /* block if system reset has been issued */
    if (sys.abort) { return; }

    /* initialize plan data struct for homing motion,
       spindle and coolant are disabled */
    plan_line_data_t pl_data;
    memset( &pl_data, 0, sizeof(plan_line_data_t) );
    pl_data.condition = PL_COND_FLAG_SYSTEM_MOTION | PL_COND_FLAG_NO_FEED_OVERRIDE;
    #ifdef USE_LINE_NUMBERS
      pl_data.line_number = HOMING_CYCLE_LINE_NUMBER;
    #endif

    /* initialize variables used for homing computations */
    uint8_t n_cycle = (2 * N_HOMING_LOCATE_CYCLE + 1);
    uint8_t step_pin[N_AXIS];
    float target[N_AXIS];
    float max_travel = 0.0;
    /* */
    for (uint8_t idx = 0; idx < N_AXIS; idx++) {
        /* initialize step pin masks */
        step_pin[idx] = (uint8_t)(1 << idx);
        #ifdef COREXY
          if ( (idx == A_MOTOR) || (idx == B_MOTOR) ) {
              step_pin[idx] = ( (uint8_t)(1 << X_AXIS) | (uint8_t)(1 << Y_AXIS) );
          }
        #endif

        /* Set target based on max_travel setting. Ensure homing switches engaged with search scalar.
           NOTE: settings.max_travel[] is stored as a negative value */
        if (bit_istrue(cycle_mask,bit(idx))) {
            max_travel = max( max_travel, (-HOMING_AXIS_SEARCH_SCALAR) * settings.max_travel[idx] );
        }
    }

    /* Set search mode with approach at seek rate to quickly engage the specified cycle_mask limit switches */
    bool_g approach = true;
    float homing_rate = settings.homing_seek_rate;

    uint8_t limit_state, axislock, n_active_axis;
    do {
        system_convert_array_steps_to_mpos(target,sys_position);
        /* initialize and declare variables needed for homing routine */
        axislock = 0;
        n_active_axis = 0;
        for (uint8_t idx = 0; idx < N_AXIS; idx++) {
            /* set target location for active axes and setup computation for homing rate */
            if (bit_istrue(cycle_mask,bit(idx))) {
                n_active_axis++;
                #ifdef COREXY
                  if (idx == X_AXIS) {
                    int32_t axis_position = system_convert_corexy_to_y_axis_steps(sys_position);
                    sys_position[A_MOTOR] = axis_position;
                    sys_position[B_MOTOR] = -axis_position;
                  } else if (idx == Y_AXIS) {
                    int32_t axis_position = system_convert_corexy_to_x_axis_steps(sys_position);
                    sys_position[A_MOTOR] = sys_position[B_MOTOR] = axis_position;
                  } else {
                    sys_position[Z_AXIS] = 0;
                  }
                #else
                  sys_position[idx] = 0;
                #endif
                /* set target direction based on cycle mask and homing cycle approach state.
                   NOTE: This happens to compile smaller than any other implementation tried */
                if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
                    if (approach) target[idx] = -max_travel;
                    else target[idx] = max_travel;
                }
                else {
                    if (approach) target[idx] = max_travel;
                    else target[idx] = -max_travel;
                }
                /* apply axislock to the step port pins active in this cycle */
                axislock |= step_pin[idx];
            }
        }
        /* [sqrt(N_AXIS)] adjust so individual axes all move at homing rate */
        homing_rate *= sqrt(n_active_axis);
        sys.homing_axis_lock = axislock;
        /* perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle */
        pl_data.feed_rate = homing_rate; // Set current homing rate.
        plan_buffer_line(target, &pl_data); // Bypass mc_line(). Directly plan homing motion.
        /* set to execute homing motion and clear existing flags */
        sys.step_control = STEP_CONTROL_EXECUTE_SYS_MOTION;
        /* Prep and fill segment buffer from newly planned block */
        stepper_prep_buffer();
        /* Initiate motion */
        stepper_wake_up();
        /* */
        do {
            if (approach) {
                /* check limit state. Lock out cycle axes when they change */
                limit_state = limits_get_state();
                for (uint8_t idx = 0; idx < N_AXIS; idx++) {
                    if (axislock & step_pin[idx]) {
                        if (limit_state & (1 << idx)) {
                          #ifdef COREXY
                            if (idx==Z_AXIS) {
                                axislock &= ~(step_pin[Z_AXIS]);
                            }
                            else {
                                axislock &= ~(step_pin[A_MOTOR]|step_pin[B_MOTOR]);
                            }
                          #else
                            axislock &= ~(step_pin[idx]);
                          #endif
                        }
                    }
                }
                sys.homing_axis_lock = axislock;
            }

            /* check and prep segment buffer
               NOTE: Should take no longer than 200us */
            stepper_prep_buffer();

            /* exit routines: No time to run protocol_execute_realtime() in this loop */
            if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) {
                uint8_t rt_exec = sys_rt_exec_state;
                /* homing failure condition: Reset issued during cycle */
                if (rt_exec & EXEC_RESET) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_RESET); }
                /* homing failure condition: Safety door was opened */
                if (rt_exec & EXEC_SAFETY_DOOR) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_DOOR); }
                /* homing failure condition: Limit switch still engaged after pull-off motion */
                if (!approach && (limits_get_state() & cycle_mask)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_PULLOFF); }
                /* homing failure condition: Limit switch not found during approach */
                if (approach && (rt_exec & EXEC_CYCLE_STOP)) { system_set_exec_alarm(EXEC_ALARM_HOMING_FAIL_APPROACH); }
                /* ALARM state, stop motors, if they are running, return */
                if (sys_rt_exec_alarm) {
                    mc_reset();
                    protocol_execute_realtime();
                    return;
                }
                else {
                    /* pull-off motion complete, disable CYCLE_STOP from executing */
                    system_clear_exec_state_flag(EXEC_CYCLE_STOP);
                    break;
                }
            }
        } while (STEP_MASK & axislock);

        /* immediately force kill steppers and reset step segment buffer */
        stepper_reset();
        /* delay to allow transient dynamics to dissipate */
        ngrbl_hal_delay_ms(settings.homing_debounce_delay);
        /* reverse direction and reset homing rate for locate cycle(s) */
        approach = !approach;
        /* after first cycle, homing enters locating phase. Shorten search to pull-off distance */
        if (approach) {
            max_travel = settings.homing_pulloff*HOMING_AXIS_LOCATE_SCALAR;
            homing_rate = settings.homing_feed_rate;
        }
        else {
            max_travel = settings.homing_pulloff;
            homing_rate = settings.homing_seek_rate;
        }

    } while (n_cycle-- > 0);

    /*  The active cycle axes should now be homed and machine limits have been located. By
        default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
        can be on either side of an axes, check and set axes machine zero appropriately. Also,
        set up pull-off maneuver from axes limit switches that have been homed. This provides
        some initial clearance off the switches and should also help prevent them from falsely
        triggering when hard limits are enabled or when more than one axes shares a limit pin.
    */
    int32_t set_axis_position;
    /* set machine positions for homed limit switches, don't update non-homed axes */
    for (uint8_t idx = 0; idx < N_AXIS; idx++) {
      /* NOTE: settings.max_travel[] is stored as a negative value */
      if (cycle_mask & bit(idx)) {
        #ifdef HOMING_FORCE_SET_ORIGIN
          set_axis_position = 0;
        #else
          if ( bit_istrue(settings.homing_dir_mask,bit(idx)) ) {
              set_axis_position = lround((settings.max_travel[idx]+settings.homing_pulloff)*settings.steps_per_mm[idx]);
          }
          else {
              set_axis_position = lround(-settings.homing_pulloff*settings.steps_per_mm[idx]);
          }
        #endif

        #ifdef COREXY
          if (idx==X_AXIS) {
              int32_t off_axis_position = system_convert_corexy_to_y_axis_steps(sys_position);
              sys_position[A_MOTOR] = set_axis_position + off_axis_position;
              sys_position[B_MOTOR] = set_axis_position - off_axis_position;
          }
          else if (idx==Y_AXIS) {
              int32_t off_axis_position = system_convert_corexy_to_x_axis_steps(sys_position);
              sys_position[A_MOTOR] = off_axis_position + set_axis_position;
              sys_position[B_MOTOR] = off_axis_position - set_axis_position;
          }
          else {
              sys_position[idx] = set_axis_position;
          }
        #else
          sys_position[idx] = set_axis_position;
        #endif

      }
    }
    sys.step_control = STEP_CONTROL_NORMAL_OP; // Return step control to normal operation.
}

/**
  * @brief  Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
            the workspace volume is in all negative space, and the system is in normal operation.
            NOTE: Used by jogging to limit travel within soft-limit volume.
  * @param  float target value
  * @retval None
  */
void limits_soft_check(float *target) {
    /* */
    if (system_check_travel_limits(target)) {
        sys.soft_limit = true;
        /* Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
           workspace volume so just come to a controlled stop so position is not lost. When complete
           enter alarm mode.
        */
        if (sys.state == STATE_CYCLE) {
            system_set_exec_state_flag(EXEC_FEED_HOLD);
            do {
                protocol_execute_realtime();
                if (sys.abort) return;
            } while ( sys.state != STATE_IDLE );
        }
        /* issue system reset and ensure spindle and coolant are shutdown */
        mc_reset();
        /* indicate soft limit critical event */
        system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT);
        /* Execute to enter critical event loop and system abort */
        protocol_execute_realtime();
        return;
    }
}


/* Callbacks -----------------------------------------------------------------*/

/**
  * @brief  ngrbl_limits_state_change_callback
  * @param  None
  * @retval None
  */
void ngrbl_limits_state_change_callback(uint8_t state) {
    /* Ignore limit switches if already in an alarm state or in-process of executing an alarm.
       When in the alarm state, Grbl should have been reset or will force a reset, so any pending
       moves in the planner and serial buffers are all cleared and newly sent blocks will be
       locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
       limit setting if their limits are constantly triggering after a reset and move their axes */
    if ( (sys.state != STATE_ALARM) && (!(sys_rt_exec_alarm)) ) {

      #ifdef HARD_LIMIT_FORCE_STATE_CHECK
        /* check limit pin state */
        if (state) {
            /* initiate system kill and indicate hard limit critical event */
            mc_reset();
            system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
        }
      #else
        /* initiate system kill and indicate hard limit critical event */
        mc_reset();
        system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT);
      #endif
    }
}


/******************************************************************************
      END FILE
******************************************************************************/