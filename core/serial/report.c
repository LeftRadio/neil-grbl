/**
  ******************************************************************************
  * @file     report.c
  * @author
  * @version  1.0.0
  * @date
  * @brief    This file functions as the primary feedback interface for Grbl.
              Any outgoing data, such as the protocol status messages,
              feedback messages, and status reports, are stored here.
              For the most part, these functions primarily are called from
              protocol.c methods. If a different style feedback is desired
              (i.e. JSON), then a user can change these following methods to
              accomodate their needs.
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ngrbl.h"
#include "report.h"
#include "serial.h"
#include "print.h"
#include "settings.h"
#include "system.h"
#include "config.h"
#include "hal_abstract.h"
#include "report.h"
#include "gcode.h"
#include "limits.h"
#include "stepper.h"
#include "probe.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "nuts_bolts.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define report_util_setting_prefix(n)     { serial_write('$'); print_uint8_base10(n); serial_write('='); }
#define report_util_line_feed()           { printString("\r\n\t"); }
#define report_util_feedback_line_feed()  { serial_write(']'); report_util_line_feed(); }
#define report_util_gcode_modes_G()       { printString(" G\t"); }
#define report_util_gcode_modes_M()       { printString(" M\t"); }
// #define report_util_comment_line_feed() { serial_write(')'); report_util_line_feed(); }

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Private Functions ---------------------------------------------------------*/

/**
  * @brief  report_util_axis_values
  * @param  float *axis_value
  * @retval None
  */
static void report_util_axis_values(float *axis_value) {
    uint8_t idx;
    for (idx = 0; idx < N_AXIS; idx++) {
        printFloat_CoordValue(axis_value[idx]);
        if (idx < (N_AXIS - 1)) { serial_write(','); }
    }
}

/**
  * @brief  report_util_uint8_setting
  * @param  uint8_t n, int val
  * @retval None
  */
static void report_util_uint8_setting(uint8_t n, int val) {
    report_util_setting_prefix(n);
    print_uint8_base10(val);
    report_util_line_feed();
}

/**
  * @brief  report_util_float_setting
  * @param  uint8_t n, float val, uint8_t n_decimal
  * @retval None
  */
static void report_util_float_setting(uint8_t n, float val, uint8_t n_decimal) {
    report_util_setting_prefix(n);
    printFloat(val, n_decimal);
    report_util_line_feed();
}


/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
            For every incoming line, this method responds with an 'ok' for a successful command or an
            'error:'  to indicate some error event with the line or some critical system error during
            operation. Errors events can originate from the g-code parser, settings module, or asynchronously
            from a critical error, such as a triggered hard limit. Interface should always monitor for these
            responses.
  * @param  uint8_t status_code
  * @retval None
  */
void report_status_message(uint8_t status_code) {
    /* */
    switch(status_code) {

      case STATUS_OK:
          printString("ok\r\n\t");
      break;

      default:
        printString("error:\t");
        print_uint8_base10(status_code);
        report_util_line_feed();
      break;
    }
}

/**
  * @brief  Prints alarm messages.
  * @param  uint8_t status_code
  * @retval None
  */
void report_alarm_message(uint8_t alarm_code) {
    printString("ALARM:\t");
    print_uint8_base10(alarm_code);
    report_util_line_feed();
    /* force delay to ensure message clears serial write buffer. */
    ngrbl_hal_delay_ms(500);
}

/**
  * @brief  Prints feedback messages. This serves as a centralized method to provide additional
            user feedback for things that are not of the status/alarm message protocol. These are
            messages such as setup warnings, switch toggling, and how to exit alarms.
            NOTE: For interfaces, messages are always placed within brackets. And if silent mode
            is installed, the message number codes are less than zero.
  * @param  uint8_t message_code
  * @retval None
  */
void report_feedback_message(uint8_t message_code) {
  printString("[MSG:\t");
  switch(message_code) {
    case MESSAGE_CRITICAL_EVENT:
      printString("Reset to continue\t"); break;
    case MESSAGE_ALARM_LOCK:
      printString("'$H'|'$X' to unlock\t"); break;
    case MESSAGE_ALARM_UNLOCK:
      printString("Caution: Unlocked\t"); break;
    case MESSAGE_ENABLED:
      printString("Enabled\t"); break;
    case MESSAGE_DISABLED:
      printString("Disabled\t"); break;
    case MESSAGE_SAFETY_DOOR_AJAR:
      printString("Check Door\t"); break;
    case MESSAGE_CHECK_LIMITS:
      printString("Check Limits\t"); break;
    case MESSAGE_PROGRAM_END:
      printString("Pgm End\t"); break;
    case MESSAGE_RESTORE_DEFAULTS:
      printString("Restoring defaults\t"); break;
    case MESSAGE_SPINDLE_RESTORE:
      printString("Restoring spindle\t"); break;
    case MESSAGE_SLEEP_MODE:
      printString("Sleeping\t"); break;
  }
  report_util_feedback_line_feed();
}

/**
  * @brief  Welcome message
  * @param  None
  * @retval None
  */
void report_init_message(void) {
    /* */
    printString("\r\nGrbl " FW_VERSION " ['$' for help]\r\n\t");
}

/**
  * @brief  Grbl help message
  * @param  None
  * @retval None
  */
void report_grbl_help(void) {
    /* */
    printString("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H ~ ! ? ctrl-x]\r\n\t");
}

/**
  * @brief  Grbl global settings print out.
            NOTE: The numbering scheme here must correlate to storing in settings.c
  * @param  None
  * @retval None
  */
void report_grbl_settings(void) {
    /* print Grbl settings */
    report_util_uint8_setting(0,settings.pulse_microseconds);
    report_util_uint8_setting(1,settings.stepper_idle_lock_time);
    report_util_uint8_setting(2,settings.step_invert_mask);
    report_util_uint8_setting(3,settings.dir_invert_mask);
    report_util_uint8_setting(4,bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE));
    report_util_uint8_setting(5,bit_istrue(settings.flags,BITFLAG_INVERT_LIMIT_PINS));
    report_util_uint8_setting(6,bit_istrue(settings.flags,BITFLAG_INVERT_PROBE_PIN));
    report_util_uint8_setting(10,settings.status_report_mask);
    report_util_float_setting(11,settings.junction_deviation,N_DECIMAL_SETTINGVALUE);
    report_util_float_setting(12,settings.arc_tolerance,N_DECIMAL_SETTINGVALUE);
    report_util_uint8_setting(13,bit_istrue(settings.flags,BITFLAG_REPORT_INCHES));
    report_util_uint8_setting(20,bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE));
    report_util_uint8_setting(21,bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE));
    report_util_uint8_setting(22,bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE));
    report_util_uint8_setting(23,settings.homing_dir_mask);
    report_util_float_setting(24,settings.homing_feed_rate,N_DECIMAL_SETTINGVALUE);
    report_util_float_setting(25,settings.homing_seek_rate,N_DECIMAL_SETTINGVALUE);
    report_util_uint8_setting(26,settings.homing_debounce_delay);
    report_util_float_setting(27,settings.homing_pulloff,N_DECIMAL_SETTINGVALUE);
    report_util_float_setting(30,settings.rpm_max,N_DECIMAL_RPMVALUE);
    report_util_float_setting(31,settings.rpm_min,N_DECIMAL_RPMVALUE);
    #ifdef VARIABLE_SPINDLE
      report_util_uint8_setting(32,bit_istrue(settings.flags,BITFLAG_LASER_MODE));
    #else
      report_util_uint8_setting(32,0);
    #endif
    /* print axis settings */
    uint8_t idx, set_idx;
    uint8_t val = AXIS_SETTINGS_START_VAL;
    for (set_idx = 0; set_idx < AXIS_N_SETTINGS; set_idx++) {
        for (idx = 0; idx < N_AXIS; idx++) {
            switch (set_idx) {
                case 0: report_util_float_setting( val + idx, settings.steps_per_mm[idx], N_DECIMAL_SETTINGVALUE); break;
                case 1: report_util_float_setting( val + idx, settings.max_rate[idx], N_DECIMAL_SETTINGVALUE); break;
                case 2: report_util_float_setting( val + idx, settings.acceleration[idx] / (60 * 60), N_DECIMAL_SETTINGVALUE); break;
                case 3: report_util_float_setting( val + idx, -settings.max_travel[idx], N_DECIMAL_SETTINGVALUE); break;
            }
        }
        val += AXIS_SETTINGS_INCREMENT;
    }
}

/**
  * @brief  Prints current probe parameters. Upon a probe command, these parameters are updated upon a
            successful probe or upon a failed probe with the G38.3 without errors command (if supported).
            These values are retained until Grbl is power-cycled, whereby they will be re-zeroed.
  * @param  None
  * @retval None
  */
void report_probe_parameters(void) {
    /* report in terms of machine position */
    printString("[PRB:\t");
    float print_position[N_AXIS];
    system_convert_array_steps_to_mpos(print_position,sys_probe_position);
    report_util_axis_values(print_position);
    serial_write(':');
    print_uint8_base10(sys.probe_succeeded);
    report_util_feedback_line_feed();
}

/**
  * @brief  Prints Grbl NGC parameters (coordinate offsets, probing)
  * @param  None
  * @retval None
  */
void report_ngc_parameters(void) {
    float coord_data[N_AXIS];
    uint8_t coord_select;
    for (coord_select = 0; coord_select <= SETTING_INDEX_NCOORD; coord_select++) {
        if (!(settings_read_coord_data(coord_select,coord_data))) {
            report_status_message(STATUS_SETTING_READ_FAIL);
            return;
        }
        printString("[G\t");
        switch (coord_select) {
            case 6: printString("28\t"); break;
            case 7: printString("30\t"); break;
            default: print_uint8_base10(coord_select+54); break; // G54-G59
        }
        serial_write(':');
        report_util_axis_values(coord_data);
        report_util_feedback_line_feed();
    }
    printString("[G92:\t"); // Print G92,G92.1 which are not persistent in memory
    report_util_axis_values(gc_state.coord_offset);
    report_util_feedback_line_feed();
    printString("[TLO:\t"); // Print tool length offset value
    printFloat_CoordValue(gc_state.tool_length_offset);
    report_util_feedback_line_feed();
    report_probe_parameters(); // Print probe parameters. Not persistent in memory.
}

/**
  * @brief  Print current gcode parser mode state
  * @param  None
  * @retval None
  */
void report_gcode_modes(void) {
    printString("[GC:G\t");
    if (gc_state.modal.motion >= MOTION_MODE_PROBE_TOWARD) {
        printString("38.\t");
        print_uint8_base10(gc_state.modal.motion - (MOTION_MODE_PROBE_TOWARD-2));
    }
    else {
      print_uint8_base10(gc_state.modal.motion);
    }

    report_util_gcode_modes_G();
    print_uint8_base10(gc_state.modal.coord_select+54);

    report_util_gcode_modes_G();
    print_uint8_base10(gc_state.modal.plane_select+17);

    report_util_gcode_modes_G();
    print_uint8_base10(21-gc_state.modal.units);

    report_util_gcode_modes_G();
    print_uint8_base10(gc_state.modal.distance+90);

    report_util_gcode_modes_G();
    print_uint8_base10(94-gc_state.modal.feed_rate);

    if (gc_state.modal.program_flow) {
        report_util_gcode_modes_M();
        switch (gc_state.modal.program_flow) {
            case PROGRAM_FLOW_PAUSED :
                serial_write('0');
            break;
            // case PROGRAM_FLOW_OPTIONAL_STOP : serial_write('1'); break; // M1 is ignored and not supported.
            case PROGRAM_FLOW_COMPLETED_M2 :
            case PROGRAM_FLOW_COMPLETED_M30 :
                print_uint8_base10(gc_state.modal.program_flow);
            break;
        }
    }

    report_util_gcode_modes_M();
    switch (gc_state.modal.spindle) {
        case SPINDLE_ENABLE_CW : serial_write('3'); break;
        case SPINDLE_ENABLE_CCW : serial_write('4'); break;
        case SPINDLE_DISABLE : serial_write('5'); break;
    }

    #ifdef ENABLE_M7
      if (gc_state.modal.coolant) { // Note: Multiple coolant states may be active at the same time.
          if (gc_state.modal.coolant & PL_COND_FLAG_COOLANT_MIST) { report_util_gcode_modes_M(); serial_write('7'); }
          if (gc_state.modal.coolant & PL_COND_FLAG_COOLANT_FLOOD) { report_util_gcode_modes_M(); serial_write('8'); }
      }
      else {
          report_util_gcode_modes_M(); serial_write('9');
      }
    #else
      report_util_gcode_modes_M();
      if (gc_state.modal.coolant) { serial_write('8'); }
      else { serial_write('9'); }
    #endif

    #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
      if (sys.override_ctrl == OVERRIDE_PARKING_MOTION) {
          report_util_gcode_modes_M();
          print_uint8_base10(56);
      }
    #endif

    printString(" T\t");
    print_uint8_base10(gc_state.tool);

    printString(" F\t");
    printFloat_RateValue(gc_state.feed_rate);

    #ifdef VARIABLE_SPINDLE
      printString(" S\t");
      printFloat(gc_state.spindle_speed,N_DECIMAL_RPMVALUE);
    #endif

    report_util_feedback_line_feed();
}

/**
  * @brief  Prints specified startup line
  * @param  None
  * @retval None
  */
void report_startup_line(uint8_t n, char *line) {
    printString("$N\t");
    print_uint8_base10(n);
    serial_write('=');
    printString(line);
    report_util_line_feed();
}

/**
  * @brief  report_execute_startup_message
  * @param  char *line, uint8_t status_code
  * @retval None
  */
void report_execute_startup_message(char *line, uint8_t status_code) {
    serial_write('>');
    printString(line);
    serial_write(':');
    report_status_message(status_code);
}

/**
  * @brief  Prints build info line
  * @param  char *line
  * @retval None
  */
void report_build_info(char *line) {
    printString("[VER:" FW_VERSION "." FW_VERSION_BUILD ":\t");
    printString(line);
    report_util_feedback_line_feed();
    printString("[OPT:\t"); // Generate compile-time build option list
    #ifdef VARIABLE_SPINDLE
      serial_write('V');
    #endif
    #ifdef USE_LINE_NUMBERS
      serial_write('N');
    #endif
    #ifdef ENABLE_M7
      serial_write('M');
    #endif
    #ifdef COREXY
      serial_write('C');
    #endif
    #ifdef PARKING_ENABLE
      serial_write('P');
    #endif
    #ifdef HOMING_FORCE_SET_ORIGIN
      serial_write('Z');
    #endif
    #ifdef HOMING_SINGLE_AXIS_COMMANDS
      serial_write('H');
    #endif
    #ifdef LIMITS_TWO_SWITCHES_ON_AXES
      serial_write('T');
    #endif
    #ifdef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
      serial_write('A');
    #endif
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      serial_write('D');
    #endif
    #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      serial_write('0');
    #endif
    #ifdef ENABLE_SOFTWARE_DEBOUNCE
      serial_write('S');
    #endif
    #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
      serial_write('R');
    #endif
    #ifndef HOMING_INIT_LOCK
      serial_write('L');
    #endif
    #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
      serial_write('+');
    #endif
    #ifndef ENABLE_RESTORE_EEPROM_WIPE_ALL // NOTE: Shown when disabled.
      serial_write('*');
    #endif
    #ifndef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // NOTE: Shown when disabled.
      serial_write('$');
    #endif
    #ifndef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // NOTE: Shown when disabled.
      serial_write('#');
    #endif
    #ifndef ENABLE_BUILD_INFO_WRITE_COMMAND // NOTE: Shown when disabled.
      serial_write('I');
    #endif
    #ifndef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE // NOTE: Shown when disabled.
      serial_write('E');
    #endif
    #ifndef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE // NOTE: Shown when disabled.
      serial_write('W');
    #endif
    // NOTE: Compiled values, like override increments/max/min values, may be added at some point later.
    serial_write(',');
    print_uint8_base10(BLOCK_BUFFER_SIZE-1);
    serial_write(',');
    print_uint8_base10(RX_BUFFER_SIZE);

    report_util_feedback_line_feed();
}

/**
  * @brief  Prints the character string line Grbl has received from the user,
            which has been pre-parsed, and has been sent into
            protocol_execute_line() routine to be executed by Grbl.
  * @param  char *line
  * @retval None
  */
void report_echo_line_received(char *line) {
    printString("[echo: \t"); printString(line);
    report_util_feedback_line_feed();
}

/**
  * @brief  Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram
            and the actual location of the CNC machine. Users may change the following function to their
            specific needs, but the desired real-time data report must be as short as possible. This is
            requires as it minimizes the computational overhead and allows grbl to keep running smoothly,
            especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
  * @param  None
  * @retval None
  */
void report_realtime_status(void) {
    /* copy current state of the system position variable */
    int32_t current_position[N_AXIS];
    memcpy(current_position, sys_position, sizeof(sys_position));
    /* */
    float print_position[N_AXIS];
    system_convert_array_steps_to_mpos(print_position,current_position);

    /* report current machine state and sub-states */
    serial_write('<');
    switch (sys.state) {
      case STATE_IDLE:
    	  printString("Idle\t");
      break;
      case STATE_CYCLE: printString("Run\t"); break;
      case STATE_HOLD:
        if (!(sys.suspend & SUSPEND_JOG_CANCEL)) {
          printString("Hold:\t");
          if (sys.suspend & SUSPEND_HOLD_COMPLETE) { serial_write('0'); } // Ready to resume
          else { serial_write('1'); } // Actively holding
          break;
        } // Continues to print jog state during jog cancel.
      case STATE_JOG: printString("Jog\t"); break;
      case STATE_HOMING: printString("Home\t"); break;
      case STATE_ALARM: printString("Alarm\t"); break;
      case STATE_CHECK_MODE: printString("Check\t"); break;
      case STATE_SAFETY_DOOR:
        printString("Door:\t");
        if (sys.suspend & SUSPEND_INITIATE_RESTORE) {
          serial_write('3'); // Restoring
        } else {
          if (sys.suspend & SUSPEND_RETRACT_COMPLETE) {
            if (sys.suspend & SUSPEND_SAFETY_DOOR_AJAR) {
              serial_write('1'); // Door ajar
            } else {
              serial_write('0');
            } // Door closed and ready to resume
          } else {
            serial_write('2'); // Retracting
          }
        }
        break;
      case STATE_SLEEP: printString("Sleep\t"); break;
    }

    float wco[N_AXIS];
    if (bit_isfalse(settings.status_report_mask, BITFLAG_RT_STATUS_POSITION_TYPE) || (sys.report_wco_counter == 0) ) {
        /* */
        for (uint8_t idx = 0; idx < N_AXIS; idx++) {
            /* apply work coordinate offsets and tool length offset to current position */
            wco[idx] = gc_state.coord_system[idx] + gc_state.coord_offset[idx];
            /* */
            if (idx == TOOL_LENGTH_OFFSET_AXIS) {
                wco[idx] += gc_state.tool_length_offset;
            }
            if (bit_isfalse(settings.status_report_mask,BITFLAG_RT_STATUS_POSITION_TYPE)) {
                print_position[idx] -= wco[idx];
            }
        }
    }

    // Report machine position
    if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_POSITION_TYPE)) {
      printString("|MPos:\t");
    } else {
      printString("|WPos:\t");
    }
    report_util_axis_values(print_position);

    // Returns planner and serial read buffer states.
    #ifdef REPORT_FIELD_BUFFER_STATE
      if (bit_istrue(settings.status_report_mask,BITFLAG_RT_STATUS_BUFFER_STATE)) {
        printString("|Bf:\t");
        print_uint8_base10(plan_get_block_buffer_available());
        serial_write(',');
        print_uint8_base10(serial_get_rx_buffer_available());
      }
    #endif

    #ifdef USE_LINE_NUMBERS
      #ifdef REPORT_FIELD_LINE_NUMBERS
        // Report current line number
        plan_block_t * cur_block = plan_get_current_block();
        if (cur_block != NULL) {
          uint32_t ln = cur_block->line_number;
          if (ln > 0) {
            printString("|Ln:\t");
            printInteger(ln);
          }
        }
      #endif
    #endif

    // Report realtime feed speed
    #ifdef REPORT_FIELD_CURRENT_FEED_SPEED
      #ifdef VARIABLE_SPINDLE
        printString("|FS:\t");
        printFloat_RateValue( stepper_get_realtime_rate() );
        serial_write(',');
        printFloat(sys.spindle_speed,N_DECIMAL_RPMVALUE);
      #else
        printString("|F:\t");
        printFloat_RateValue( st_get_realtime_rate() );
      #endif
    #endif

    #ifdef REPORT_FIELD_PIN_STATE
      uint8_t lim_pin_state = limits_get_state();
      uint8_t ctrl_pin_state = system_control_get_state();
      uint8_t prb_pin_state = probe_get_state();
      if (lim_pin_state | ctrl_pin_state | prb_pin_state) {
        printString("|Pn:\t");
        if (prb_pin_state) { serial_write('P'); }
        if (lim_pin_state) {
          if (bit_istrue(lim_pin_state,bit(X_AXIS))) { serial_write('X'); }
          if (bit_istrue(lim_pin_state,bit(Y_AXIS))) { serial_write('Y'); }
          if (bit_istrue(lim_pin_state,bit(Z_AXIS))) { serial_write('Z'); }
        }
        if (ctrl_pin_state) {
          #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
            if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_SAFETY_DOOR)) { serial_write('D'); }
          #endif
          if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_RESET)) { serial_write('R'); }
          if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_FEED_HOLD)) { serial_write('H'); }
          if (bit_istrue(ctrl_pin_state,CONTROL_PIN_INDEX_CYCLE_START)) { serial_write('S'); }
        }
      }
    #endif

    #ifdef REPORT_FIELD_WORK_COORD_OFFSET
      if (sys.report_wco_counter > 0) { sys.report_wco_counter--; }
      else {
        if (sys.state & (STATE_HOMING | STATE_CYCLE | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)) {
          sys.report_wco_counter = (REPORT_WCO_REFRESH_BUSY_COUNT-1); // Reset counter for slow refresh
        } else { sys.report_wco_counter = (REPORT_WCO_REFRESH_IDLE_COUNT-1); }
        if (sys.report_ovr_counter == 0) { sys.report_ovr_counter = 1; } // Set override on next report.
        printString("|WCO:\t");
        report_util_axis_values(wco);
      }
    #endif

    #ifdef REPORT_FIELD_OVERRIDES
      if (sys.report_ovr_counter > 0) { sys.report_ovr_counter--; }
      else {
        if (sys.state & (STATE_HOMING | STATE_CYCLE | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)) {
          sys.report_ovr_counter = (REPORT_OVR_REFRESH_BUSY_COUNT-1); // Reset counter for slow refresh
        } else { sys.report_ovr_counter = (REPORT_OVR_REFRESH_IDLE_COUNT-1); }
        printString("|Ov:\t");
        print_uint8_base10(sys.f_override);
        serial_write(',');
        print_uint8_base10(sys.r_override);
        serial_write(',');
        print_uint8_base10(sys.spindle_speed_ovr);

        uint8_t sp_state = spindle_get_state();
        uint8_t cl_state = coolant_get_state();
        if (sp_state || cl_state) {
          printString("|A:\t");
          if (sp_state) { // != SPINDLE_STATE_DISABLE
            #ifdef VARIABLE_SPINDLE
              #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
                serial_write('S'); // CW
              #else
                if (sp_state == SPINDLE_STATE_CW) { serial_write('S'); } // CW
                else { serial_write('C'); } // CCW
              #endif
            #else
              if (sp_state & SPINDLE_STATE_CW) { serial_write('S'); } // CW
              else { serial_write('C'); } // CCW
            #endif
          }
          if (cl_state & COOLANT_STATE_FLOOD) { serial_write('F'); }
          #ifdef ENABLE_M7
            if (cl_state & COOLANT_STATE_MIST) { serial_write('M'); }
          #endif
        }
      }
    #endif
    /* */
    serial_write('>');
    report_util_line_feed();
}


/******************************************************************************
      END FILE
******************************************************************************/
