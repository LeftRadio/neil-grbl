/*
  ******************************************************************************
  * @file     motion_control.c
  * @author
  * @version  1.0.0
  * @date
  * @brief
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <math.h>
// #include <string.h>

#include "motion_control.h"
#include "settings.h"
#include "planner.h"
#include "system.h"
#include "limits.h"
#include "protocol.h"
#include "spindle_control.h"
#include "coolant_control.h"
#include "gcode.h"
#include "probe.h"
#include "stepper.h"
#include "config.h"
#include "report.h"
#include "nuts_bolts.h"
#include "hal_abstract.h"
#include "config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile float pwm_gradient;

/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Private Functions ---------------------------------------------------------*/
/* Exported Functions --------------------------------------------------------*/

/**
  * @brief  Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
            unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
            (1 minute)/feed_rate time.
            NOTE: This is the primary gateway to the grbl planner. All line motions, including arc line
            segments, must pass through this routine before being passed to the planner. The seperation of
            mc_line and plan_buffer_line is done primarily to place non-planner-type functions from being
            in the planner and to let backlash compensation or canned cycle integration simple and direct.
  * @param  float *target, plan_line_data_t *pl_data
  * @retval None
  */
void spindle_init(void) {
    #ifdef VARIABLE_SPINDLE
      /* configure variable spindle PWM and enable pin, if requried, on the Uno, PWM and enable are
         combined unless configured otherwise */
      ngrbl_hal_spindle_init(NGRBL_HAL_SPINDLE_PWM_MODE);
      /* */
      pwm_gradient = SPINDLE_PWM_RANGE / (settings.rpm_max - settings.rpm_min);
    #else
      /* configure no variable spindle and only enable pin */
      ngrbl_hal_spindle_init(NGRBL_HAL_SPINDLE_DISCRETE_MODE);
    #endif
    /* */
    ngrbl_hal_spindle_stop();
}

/**
  * @brief  spindle_get_state
  * @param  None
  * @retval uint8_t state
  */
grbl_spindle_state_t spindle_get_state(void) {
    /* */
    return ngrbl_hal_spindle_get_state();
}

/**
  * @brief  Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
            Called by various main program and ISR routines. Keep routine small, fast, and efficient.
            Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
  * @param  None
  * @retval None
  */
void spindle_stop(void) {
    ngrbl_hal_spindle_stop();
}

/**
  * @brief  Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
            and stepper ISR. Keep routine small and efficient.
  * @param  None
  * @retval None
  */
void spindle_set_speed(uint8_t pwm_value) {
    /* Set PWM output level. */
    ngrbl_hal_spindle_set_pwm(pwm_value);
}




/**
  * @brief  Called by spindle_set_state() and step segment generator.
            Keep routine small and efficient.
  * @param  None
  * @retval None
  */
uint8_t spindle_compute_pwm_value(float rpm) {
    /* */
    uint8_t pwm_value = 0;
    /* scale by spindle speed override value */
    rpm *= (0.010*sys.spindle_speed_ovr); //


    #ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE

      /* calculate PWM register value based on rpm max/min settings and programmed rpm */
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= RPM_MAX)) {
          rpm = RPM_MAX;
          pwm_value = SPINDLE_PWM_MAX_VALUE;
      }
      else if (rpm <= RPM_MIN) {
          /* S0 disables spindle */
          if (rpm == 0.0) {
              pwm_value = SPINDLE_PWM_OFF_VALUE;
          }
          else {
              rpm = RPM_MIN;
              pwm_value = SPINDLE_PWM_MIN_VALUE;
          }
      }
      else {
          /* compute intermediate PWM value with linear spindle speed model
             via piecewise linear fit model */
          #if (N_PIECES > 3)
            if (rpm > RPM_POINT34) { pwm_value = floor(RPM_LINE_A4*rpm - RPM_LINE_B4); }
          #endif
          #if (N_PIECES > 2)
            if (rpm > RPM_POINT23) { pwm_value = floor(RPM_LINE_A3*rpm - RPM_LINE_B3); }
          #endif
          #if (N_PIECES > 1)
            if (rpm > RPM_POINT12) { pwm_value = floor(RPM_LINE_A2*rpm - RPM_LINE_B2); }
          #endif
          /* */
          else { pwm_value = floor(RPM_LINE_A1*rpm - RPM_LINE_B1); }
      }
      sys.spindle_speed = rpm;

    #else

      /* calculate PWM register value based on rpm max/min settings and programmed rpm */
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
        /* no PWM range possible, set simple on/off spindle control pin state */
          sys.spindle_speed = settings.rpm_max;
          pwm_value = SPINDLE_PWM_MAX_VALUE;
      }
      else if (rpm <= settings.rpm_min) {
          /* S0 disables spindle */
          if (rpm == 0.0) {
              sys.spindle_speed = 0.0;
              pwm_value = SPINDLE_PWM_OFF_VALUE;
          }
          /* set minimum PWM output */
          else {
              sys.spindle_speed = settings.rpm_min;
              pwm_value = SPINDLE_PWM_MIN_VALUE;
          }
      }
      else {
          /* compute intermediate PWM value with linear spindle speed model,
             NOTE: A nonlinear model could be installed here, if required,
             but keep it light-weight as possible */
          sys.spindle_speed = rpm;
          pwm_value = floor((rpm - settings.rpm_min) * pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
      }
    #endif

    return(pwm_value);
}

/**
  * @brief  Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
            Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
            sleep, and spindle stop override.
  * @param  None
  * @retval None
  */
void spindle_set_state(uint8_t state, float rpm) {
    /* block during abort */
    if (sys.abort) { return; }
    /* Halt or set spindle direction and rpm */
    if (state == SPINDLE_DISABLE) {
        #ifdef VARIABLE_SPINDLE
          sys.spindle_speed = 0.0;
        #endif
        ngrbl_hal_spindle_stop();
    }
    else {
        /* */
        ngrbl_hal_spindle_start();
        /* */
        #ifdef VARIABLE_SPINDLE
          /* assumes all calls to this function is when grbl is not moving or must remain off */
          if (settings.flags & BITFLAG_LASER_MODE) {
              /* may need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE) */
              if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; };
          }
          spindle_set_speed( spindle_compute_pwm_value(rpm) );
        #endif
  }
  sys.report_ovr_counter = 0; // Set to report change immediately
}

/**
  * @brief  G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails
            if an abort or check-mode is active.
  * @param  None
  * @retval None
  */
void spindle_sync(uint8_t state, float rpm) {
    if (sys.state == STATE_CHECK_MODE) { return; }
    /* empty planner buffer to ensure spindle is set when programmed */
    protocol_buffer_synchronize();
    spindle_set_state(state,rpm);
}


/******************************************************************************
      END FILE
******************************************************************************/