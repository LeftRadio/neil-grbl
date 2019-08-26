/**
  ******************************************************************************
  * @file     stepper.c
  * @author   leftradio
  * @version  1.0.0
  * @date
  * @brief    Stepper motor driver: executes motion plans using stepper motors
  ******************************************************************************
**/

/*    BLOCK VELOCITY PROFILE DEFINITION
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity

  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  +
                    +--------+ <- maximum_speed (= nominal_speed)          /|\
                   /          \                                           / | \
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |
                  +-------------+                     current_speed -> +----+--+
                   time -->  ^  ^                                           ^  ^
                             |  |                                           |  |
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)

  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters
  are shown and defined in the above illustration.
*/

/* Includes ------------------------------------------------------------------*/
#include "grbl.h"
#include "hal_abstarct.h"


/* Private typedef -----------------------------------------------------------*/

/* stores the planner block Bresenham algorithm execution data for the segments in the segment
   buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
   never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
   NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
   discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
   data for its own use. */
typedef struct _st_block_t {
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    uint8_t direction_bits;
    #ifdef VARIABLE_SPINDLE
      uint8_t is_pwm_rate_adjusted; // Tracks motions that require constant laser power/rate
    #endif
} st_block_t;

/* primary stepper segment ring buffer. Contains small, short line segments for the stepper
   algorithm to execute, which are "checked-out" incrementally from the first block in the
   planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
   the planner, where the remaining planner block steps still can. */
typedef struct _segment_t {
    uint16_t n_step;           // Number of step events to be executed for this segment
    uint16_t cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
    uint8_t  st_block_index;   // Stepper block data index. Uses this information to execute this segment.
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
    #else
      uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
    #endif
    #ifdef VARIABLE_SPINDLE
      uint8_t spindle_pwm;
    #endif
} segment_t;

/* stepper data struct. Contains the running data for the main stepper ISR */
typedef struct _stepper_t {
    /* Used by the bresenham line algorithm */
    uint32_t counter_x, counter_y, counter_z;
    #ifdef STEP_PULSE_DELAY
      /* Stores out_bits output to complete the step pulse delay */
      uint16_t step_delay_bits;
    #endif
    uint8_t execute_step;         // Flags step execution for each interrupt.
    uint16_t step_pulse_time;     // Step pulse reset time after step rise
    uint8_t step_outbits;         // The next stepping-bits to be output
    uint8_t dir_outbits;
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      uint32_t steps[N_AXIS];
    #endif
    uint16_t step_count;          // Steps remaining in line segment motion
    uint8_t exec_block_index;     // Tracks the current st_block index. Change indicates new block.
    st_block_t *exec_block;       // Pointer to the block data for the segment being executed
    segment_t *exec_segment;      // Pointer to the segment being executed
    volatile bool busy;           // Used to avoid ISR nesting of the "Stepper Driver Interrupt"
} stepper_t;

/* segment preparation data struct. Contains all the necessary information
   to compute new segments based on the current executing planner block */
typedef struct _st_prep_t {
    uint8_t st_block_index;  // Index of stepper common data block being prepped
    uint8_t recalculate_flag;

    float dt_remainder;
    float steps_remaining;
    float step_per_mm;
    float req_mm_increment;

    #ifdef PARKING_ENABLE
      uint8_t last_st_block_index;
      float last_steps_remaining;
      float last_step_per_mm;
      float last_dt_remainder;
    #endif

    uint8_t ramp_type;      // Current segment ramp state
    float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                            // NOTE: This value must coincide with a step(no mantissa) when converted.
    float current_speed;    // Current speed at the end of the segment buffer (mm/min)
    float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
    float exit_speed;       // Exit speed of executing block (mm/min)
    float accelerate_until; // Acceleration ramp end measured from end of block (mm)
    float decelerate_after; // Deceleration ramp start measured from end of block (mm)

    #ifdef VARIABLE_SPINDLE
      float inv_rate;    // Used by PWM laser mode to speed up segment calculations.
      uint8_t current_spindle_pwm;
    #endif
} st_prep_t;


/* Private define ------------------------------------------------------------*/

/* define step pulse bits */
#define X_STEP_BIT      0
#define Y_STEP_BIT      1
#define Z_STEP_BIT      2
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT))

/* define step direction output pins */
#define X_DIRECTION_BIT   0
#define Y_DIRECTION_BIT   1
#define Z_DIRECTION_BIT   2
#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT))

/* define stepper driver enable/disable output pin */
#define STEPPERS_DISABLE_BIT    0
#define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)



/* some useful constants */
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment
#define REQ_MM_INCREMENT_SCALAR 1.25
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2
#define RAMP_DECEL_OVERRIDE 3

#define PREP_FLAG_RECALCULATE bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK bit(1)
#define PREP_FLAG_PARKING bit(2)
#define PREP_FLAG_DECEL_OVERRIDE bit(3)

/* Private macro -------------------------------------------------------------*/
// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  #define MAX_AMASS_LEVEL 3
  // AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
  #define AMASS_LEVEL1 (F_CPU/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
  #define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
  #define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)
#endif


/* Private variables ---------------------------------------------------------*/
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];
static stepper_t stepper;
static st_prep_t prep;

/* step segment ring buffer indices */
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

/* Step and direction port invert masks */
static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;



/* Pointers for the step segment being prepped from the planner buffer.
   Accessed only by the main program. Pointers may be planning segments
   or planner blocks ahead of what being executed */
static plan_block_t *pl_block;     // planner block being prepped
static st_block_t *st_prep_block;  // stepper block data being prepped

/* Private function prototypes -----------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/**
  * @brief  Stepper state initialization. Cycle should only start if the stepper.cycle_start flag is
  *         enabled. Startup init and limits call this function but shouldn't start the cycle.
  * @param  None
  * @retval None
  */
void st_wake_up(void) {
    /* Enable stepper drivers */
    grbl_hal_stepper_set_state(true);

    #ifdef STP_DRIVERS_ENABLE_DELAY
      grbl_hal_delay_ms(STP_DRIVERS_ENABLE_DELAY);
    #endif

    /* Initialize stepper output bits to ensure first ISR call does not step */
    stepper.step_outbits = step_port_invert_mask;

    /* Initialize step pulse timing from settings. Here to ensure updating after re-writing */
    #ifdef STEP_PULSE_DELAY
      /* set compare for pulse timer */
      grbl_hal_stepper_timer_pulse_set_compare((STEP_PULSE_DELAY - 1) * TICKS_PER_MICROSECOND + 1);
      /* Set total step pulse time after direction pin set. Ad hoc computation from oscilloscope */
      stepper.step_pulse_time = (settings.pulse_microseconds + (STEP_PULSE_DELAY - 1)) * TICKS_PER_MICROSECOND + 1;
    #else
      /* Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement */
      stepper.step_pulse_time = (settings.pulse_microseconds)*TICKS_PER_MICROSECOND;
    #endif

    /* set the autoreload value for base timer */
    grbl_hal_stepper_timer_base_set_reload(stepper.exec_segment->cycles_per_tick - 1);
    /* Set step pulse time */
    grbl_hal_stepper_timer_pulse_set_reload(stepper.step_pulse_time);

    #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      /* set prescaler value for base timer */
      grbl_hal_stepper_timer_base_set_prescaler(stepper.exec_segment->prescaler);
    #endif

    /* Enable stepper drivers timers interrupts */
    grbl_hal_stepper_timer_base_irq_start();
    grbl_hal_stepper_timer_pulse_irq_start();
}

/**
  * @brief  Steppers shutdown
  * @param  None
  * @retval None
  */
void st_go_idle(void) {
    /* disable stepper timer base interrupts.
       Allow port reset interrupt to finish, if active. */
    grbl_hal_stepper_timer_base_stop();
    /* reset busy flag */
    stepper.busy = false;
    /* set idle state, disabled or enabled, depending on settings and circumstances. */
    bool state = true;
    if (((settings.stepper_idle_lock_time != 0xff) || \
          sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING) {
        /* force dwell to lock axes for a defined amount of time to ensure the axes come to a complete
           stop and not drift from residual inertial forces at the end of the last movement. */
        grbl_hal_delay_ms(settings.stepper_idle_lock_time);
        state = false;
    }
    /* update state for stepper drivers */
    grbl_hal_stepper_set_state(state);
}

/**
  * @brief  Generates the step and direction port invert masks used in the Stepper Interrupt Driver.
  * @param  None
  * @retval None
  */
void st_generate_step_dir_invert_masks(void) {
    /* Reset step/dir masks */
    step_port_invert_mask = 0;
    dir_port_invert_mask = 0;
    /* */
    for (uint8_t i = 0; i < N_AXIS; i++) {
        if (bit_istrue(settings.step_invert_mask, bit(idx))) {
            step_port_invert_mask |= get_step_pin_mask(idx);
        }
        if (bit_istrue(settings.dir_invert_mask, bit(idx))) {
            dir_port_invert_mask |= get_direction_pin_mask(idx);
        }
    }
}

/**
  * @brief  Reset and clear stepper subsystem variables
  * @param  None
  * @retval None
  */
void st_reset(void) {
    /* Initialize stepper driver idle state */
    st_go_idle();
    /* Initialize stepper algorithm variables */
    memset(&prep, 0, sizeof(st_prep_t));
    memset(&st, 0, sizeof(stepper_t));
    stepper.exec_segment = NULL;
    /* Planner block pointer used by segment buffer */
    pl_block = NULL;
    segment_buffer_tail = 0;
    segment_buffer_head = 0;
    segment_next_head = 1;
    stepper.busy = false;
    /* Generate step and direction masks */
    st_generate_step_dir_invert_masks();
    /* Initialize direction bits to default. */
    stepper.dir_outbits = dir_port_invert_mask;
    /* Initialize step and direction port pins */
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | step_port_invert_mask;
    DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | dir_port_invert_mask;
}

/**
  * @brief  Initialize and start the stepper motor subsystem
  * @param  None
  * @retval None
  */
void stepper_init(void) {
    /* Init GPIO for ENABLE, DIRECTION, STEP signals for all stepper drivers */
    grbl_hal_stepper_gpio_init();
    /* Init two timers:
       1 - base timer with 33.3usec period
       2 - pulse width timer with (settings.pulse_microseconds) period (default 10usec)
    */
    grbl_hal_stepper_timer_base_init(33.3);
    grbl_hal_stepper_timer_pulse_init(settings.pulse_microseconds);
}

/**
  * @brief  Called by planner_recalculate() when the executing block is updated by the new plan.
  * @param  None
  * @retval None
  */
void st_update_plan_block_parameters(void) {
    /* Ignore if at start of a new block. */
    if (pl_block != NULL) {
        prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
        /* Update entry speed. */
        pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed;
        /* Flag st_prep_segment() to load and check active velocity profile. */
        pl_block = NULL;
    }
}

/**
  * @brief  Increments the step segment buffer block data ring buffer.
  * @param  None
  * @retval None
  */
static uint8_t st_next_block_index(uint8_t block_index) {
    block_index++;
    if ( block_index >= (SEGMENT_BUFFER_SIZE-1) ) {
        return 0;
    }
    return block_index;
}


#ifdef PARKING_ENABLE

/**
  * @brief  Changes the run state of the step segment buffer to execute the special parking motion.
  * @param  None
  * @retval None
  */
void st_parking_setup_buffer(void) {
      /* Store step execution data of partially completed block, if necessary */
      if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
          prep.last_st_block_index = prep.st_block_index;
          prep.last_steps_remaining = prep.steps_remaining;
          prep.last_dt_remainder = prep.dt_remainder;
          prep.last_step_per_mm = prep.step_per_mm;
      }
      /* Set flags to execute a parking motion */
      prep.recalculate_flag |= PREP_FLAG_PARKING;
      prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
      /* Always reset parking motion to reload new block */
      pl_block = NULL;
  }

/**
  * @brief  Restores the step segment buffer to the normal run state after a parking motion.
  * @param  None
  * @retval None
  */
void st_parking_restore_buffer(void) {
    /* Restore step execution data and flags of partially completed block, if necessary */
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
        st_prep_block = &st_block_buffer[prep.last_st_block_index];
        prep.st_block_index = prep.last_st_block_index;
        prep.steps_remaining = prep.last_steps_remaining;
        prep.dt_remainder = prep.last_dt_remainder;
        prep.step_per_mm = prep.last_step_per_mm;
        prep.recalculate_flag = (PREP_FLAG_HOLD_PARTIAL_BLOCK | PREP_FLAG_RECALCULATE);
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm; // Recompute this value.
    }
    else {
        prep.recalculate_flag = false;
    }
    /* Set to reload next block */
    pl_block = NULL;
}

#endif  /* PARKING_ENABLE */


/**
  * @brief  Prepares step segment buffer. Continuously called from main program.
            The segment buffer is an intermediary buffer interface between the execution of steps
            by the stepper algorithm and the velocity profiles generated by the planner. The stepper
            algorithm only executes steps within the segment buffer and is filled by the main program
            when steps are "checked-out" from the first block in the planner buffer. This keeps the
            step execution and planning optimization processes atomic and protected from each other.
            The number of steps "checked-out" from the planner buffer and the number of segments in
            the segment buffer is sized and computed such that no operation in the main program takes
            longer than the time it takes the stepper algorithm to empty it before refilling it.
            Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
            NOTE: Computation units are in steps, millimeters, and minutes.
  * @param  None
  * @retval None
  */
void st_prep_buffer(void) {
  /* Block step prep buffer, while in a suspend state and there is no suspend motion to execute */
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { return; }
  /* Fill the buffer if nedded */
  while (segment_buffer_tail != segment_next_head) {
    /* Determine if we need to load a new planner block or if the block needs to be recomputed */
    if (pl_block == NULL) {
      /* Query planner for a queued block */
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          pl_block = plan_get_system_motion_block();
      }
      else {
          pl_block = plan_get_current_block();
      }
      /* No planner blocks. Exit */
      if (pl_block == NULL) {
          return;
      }
      /* Check if we need to only recompute the velocity profile or load a new block */
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {
          #ifdef PARKING_ENABLE
              if (prep.recalculate_flag & PREP_FLAG_PARKING) { prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE); }
              else { prep.recalculate_flag = false; }
          #else
              prep.recalculate_flag = false;
          #endif
      }
      else {
          /* Load the Bresenham stepping data for the block */
          prep.st_block_index = st_next_block_index(prep.st_block_index);
          /* Prepare and copy Bresenham algorithm segment data from the new planner block, so that
             when the segment buffer completes the planner block, it may be discarded when the
             segment buffer finishes the prepped block, but the stepper ISR is still executing it */
          st_prep_block = &st_block_buffer[prep.st_block_index];
          st_prep_block->direction_bits = pl_block->direction_bits;

          #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (uint8_t i = 0; idx < N_AXIS; idx++) {
              st_prep_block->steps[idx] = (pl_block->steps[idx] << 1);
          }
          st_prep_block->step_event_count = (pl_block->step_event_count << 1);
          #else
          /* With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS
             level, such that we never divide beyond the original data anywhere in the algorithm.
             If the original data is divided, we can lose a step from integer roundoff */
          for (uint8_t idx = 0; idx < N_AXIS; idx++) {
              st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL;
          }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
          #endif
          /* Initialize segment buffer data for generating the segments */
          prep.steps_remaining = (float)pl_block->step_event_count;
          prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
          prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
          prep.dt_remainder = 0.0; // Reset for new segment block
          /* */
          if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
              /* New block loaded mid-hold. Override planner block entry speed to enforce deceleration */
              prep.current_speed = prep.exit_speed;
              pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
              prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
          }
          else {
              prep.current_speed = sqrt(pl_block->entry_speed_sqr);
          }

          #ifdef VARIABLE_SPINDLE
          /* Setup laser mode variables. PWM rate adjusted motions will always complete a motion with the spindle off */
          st_prep_block->is_pwm_rate_adjusted = false;
          if (settings.flags & BITFLAG_LASER_MODE) {
              if (pl_block->condition & PL_COND_FLAG_SPINDLE_CCW) {
                  /* Pre-compute inverse programmed rate to speed up PWM updating per step segment */
                  prep.inv_rate = 1.0/pl_block->programmed_rate;
                  st_prep_block->is_pwm_rate_adjusted = true;
              }
          }
          #endif
      }

			/* ---------------------------------------------------------------------------------
			 Compute the velocity profile of a new planner block based on its entry and exit
			 speeds, or recompute the profile of a partially-completed planner block if the
			 planner has updated it. For a commanded forced-deceleration, such as from a feed
			 hold, override the planner velocities and decelerate to the target exit speed.
			*/
			prep.mm_complete = 0.0; // Default velocity profile complete at 0.0mm from end of block.
			float inv_2_accel = 0.5/pl_block->acceleration;
			if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { // [Forced Deceleration to Zero Velocity]
				// Compute velocity profile parameters for a feed hold in-progress. This profile overrides
				// the planner block profile, enforcing a deceleration to zero speed.
				prep.ramp_type = RAMP_DECEL;
				// Compute decelerate distance relative to end of block.
				float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
				if (decel_dist < 0.0) {
					// Deceleration through entire planner block. End of feed hold is not in this block.
					prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
				} else {
					prep.mm_complete = decel_dist; // End of feed hold.
					prep.exit_speed = 0.0;
				}
			} else { // [Normal Operation]
				// Compute or recompute velocity profile parameters of the prepped planner block.
				prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
				prep.accelerate_until = pl_block->millimeters;

				float exit_speed_sqr;
				float nominal_speed;
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          prep.exit_speed = exit_speed_sqr = 0.0; // Enforce stop at end of system motion.
        } else {
          exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
          prep.exit_speed = sqrt(exit_speed_sqr);
        }

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);
				float nominal_speed_sqr = nominal_speed*nominal_speed;
				float intersect_distance =
								0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));

        if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // Only occurs during override reductions.
          prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
          if (prep.accelerate_until <= 0.0) { // Deceleration-only.
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;

            // Compute override block exit speed since it doesn't match the planner exit speed.
            prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // Flag to load next block as deceleration override.

            // TODO: Determine correct handling of parameters in deceleration-only.
            // Can be tricky since entry speed will be current speed, as in feed holds.
            // Also, look into near-zero speed handling issues with this.

          } else {
            // Decelerate to cruise or cruise-decelerate types. Guaranteed to intersect updated plan.
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr); // Should always be >= 0.0 due to planner reinit.
            prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE;
          }
				} else if (intersect_distance > 0.0) {
					if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
						// NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
						prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
						if (prep.decelerate_after < intersect_distance) { // Trapezoid type
							prep.maximum_speed = nominal_speed;
							if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
								// Cruise-deceleration or cruise-only type.
								prep.ramp_type = RAMP_CRUISE;
							} else {
								// Full-trapezoid or acceleration-cruise types
								prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr);
							}
						} else { // Triangle type
							prep.accelerate_until = intersect_distance;
							prep.decelerate_after = intersect_distance;
							prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
						}
					} else { // Deceleration-only type
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;
					}
				} else { // Acceleration-only type
					prep.accelerate_until = 0.0;
					// prep.decelerate_after = 0.0;
					prep.maximum_speed = prep.exit_speed;
				}
			}

      #ifdef VARIABLE_SPINDLE
        bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); // Force update whenever updating block.
      #endif
    }

    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create
      a full segment based on the current ramp conditions. If the segment time is incomplete
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if
      an incomplete segment terminates at the end of the velocity profile, the segment is
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at
      the end of planner block (typical) or mid-block at the end of a forced deceleration,
      such as from a feed hold.
    */
    float dt_max = DT_SEGMENT; // Maximum segment time
    float dt = 0.0; // Initialize segment time
    float time_var = dt_max; // Time worker variable
    float mm_var; // mm-Distance worker variable
    float speed_var; // Speed worker variable
    float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
    float minimum_mm = mm_remaining-prep.req_mm_increment; // Guarantee at least one step.
    if (minimum_mm < 0.0) { minimum_mm = 0.0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
          speed_var = pl_block->acceleration*time_var;
          if (prep.current_speed-prep.maximum_speed <= speed_var) {
            // Cruise or cruise-deceleration types only for deceleration override.
            mm_remaining = prep.accelerate_until;
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            prep.ramp_type = RAMP_CRUISE;
            prep.current_speed = prep.maximum_speed;
          } else { // Mid-deceleration override ramp.
            mm_remaining -= time_var*(prep.current_speed - 0.5*speed_var);
            prep.current_speed -= speed_var;
          }
          break;
        case RAMP_ACCEL:
          // NOTE: Acceleration ramp only computes during first do-while loop.
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5*speed_var);
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only.
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE:
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
          // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
          //   prevent this, simply enforce a minimum speed threshold in the planner.
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // End of cruise.
            // Cruise-deceleration junction or end of block.
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.
            mm_remaining = mm_var;
          }
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (mm/min)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed.
            // Compute distance from end of segment to end of block.
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // Typical case. In deceleration ramp.
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop.
            }
          }
          // Otherwise, at end of block or end of forced-deceleration.
          time_var = 2.0*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
          prep.current_speed = prep.exit_speed;
      }
      dt += time_var; // Add computed ramp time to total segment time.
      if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction.
      else {
        if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
          // Increase segment time to ensure at least one step in segment. Override and loop
          // through distance calculations until minimum_mm or mm_complete.
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; // **Complete** Exit loop. Segment execution time maxed.
        }
      }
    } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.

    #ifdef VARIABLE_SPINDLE
      /* -----------------------------------------------------------------------------------
        Compute spindle speed PWM output for step segment
      */

      if (st_prep_block->is_pwm_rate_adjusted || (sys.step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
        if (pl_block->condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)) {
          float rpm = pl_block->spindle_speed;
          // NOTE: Feed and rapid overrides are independent of PWM value and do not alter laser power/rate.
          if (st_prep_block->is_pwm_rate_adjusted) { rpm *= (prep.current_speed * prep.inv_rate); }
          // If current_speed is zero, then may need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE)
          // but this would be instantaneous only and during a motion. May not matter at all.
          prep.current_spindle_pwm = spindle_compute_pwm_value(rpm);
        } else {
          sys.spindle_speed = 0.0;
          prep.current_spindle_pwm = SPINDLE_PWM_OFF_VALUE;
        }
        bit_false(sys.step_control,STEP_CONTROL_UPDATE_SPINDLE_PWM);
      }
      prep_segment->spindle_pwm = prep.current_spindle_pwm; // Reload segment PWM value

    #endif

    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and apply necessary rate corrections.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions.
       However, since floats have only 7.2 significant digits, long moves with extremely
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
    */
    float step_dist_remaining = prep.step_per_mm*mm_remaining; // Convert mm_remaining to steps
    float n_steps_remaining = ceil(step_dist_remaining); // Round-up current steps remaining
    float last_n_steps_remaining = ceil(prep.steps_remaining); // Round-up last steps remaining
    prep_segment->n_step = last_n_steps_remaining-n_steps_remaining; // Compute number of steps to execute.

    // Bail if we are at the end of a feed hold and don't have a step to execute.
    if (prep_segment->n_step == 0) {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        // Less than one step to decelerate to zero speed, but already very close. AMASS
        // requires full steps to execute. So, just bail.
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Segment not generated, but current step data still retained.
      }
    }

    // Compute segment step rate. Since steps are integers and mm distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not
    // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the whole segment rate to keep step output exact. These rate adjustments are
    // typically very small and do not adversely effect performance, but ensures that Grbl
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    dt += prep.dt_remainder; // Apply previous segment partial step execute time
    float inv_rate = dt/(last_n_steps_remaining - step_dist_remaining); // Compute adjusted step rate inverse

    // Compute CPU cycles per step for the prepped segment.
    uint32_t cycles = ceil( (TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); // (cycles/step)

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // Compute step timing and multi-axis smoothing level.
      // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // Just set the slowest speed possible.
    #else
      // Compute step timing and timer prescalar for normal step generation.
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // prescaler: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // prescaler: 8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else {
        prep_segment->prescaler = 3; // prescaler: 64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { // Just set the slowest speed possible. (Around 4 step/sec.)
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    // Segment complete! Increment segment buffer indices, so stepper ISR can immediately execute it.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    // Update the appropriate planner and segment data.
    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining)*inv_rate;

    // Check for exit conditions and flag to load next planner block.
    if (mm_remaining == prep.mm_complete) {
      // End of planner block or forced-termination. No more distance to be executed.
      if (mm_remaining > 0.0) { // At end of forced-termination.
        // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
        // the segment queue, where realtime protocol will set new state upon receiving the
        // cycle stop flag from the ISR. Prep_segment is blocked until then.
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Bail!
      } else { // End of planner block
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
          return;
        }
        pl_block = NULL; // Set pointer to indicate check and load next planner block.
        plan_discard_current_block();
      }
    }

  }
}

/**
  * @brief  Called by realtime status reporting to fetch the current speed being executed. This value
            however is not exactly the current speed, but the speed computed in the last step segment
            in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
            divided by the ACCELERATION TICKS PER SECOND in seconds.
  * @param  None
  * @retval rate as float value
  */
float st_get_realtime_rate(void) {
    if ( sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR) ){
        return prep.current_speed;
    }
    return 0.0f;
}


// Returns step pin mask according to Grbl internal axis indexing.
uint8_t get_step_pin_mask(uint8_t axis_idx) {
    if ( axis_idx == X_AXIS ) { return ((1 << X_STEP_BIT)); }
    if ( axis_idx == Y_AXIS ) { return ((1 << Y_STEP_BIT)); }
    return ((1 << Z_STEP_BIT));
}

// Returns direction pin mask according to Grbl internal axis indexing.
uint8_t get_direction_pin_mask(uint8_t axis_idx) {
    if ( axis_idx == X_AXIS ) { return ((1 << X_DIRECTION_BIT)); }
    if ( axis_idx == Y_AXIS ) { return ((1 << Y_DIRECTION_BIT)); }
    return ((1 << Z_DIRECTION_BIT));
}

// Returns limit pin mask according to Grbl internal axis indexing.
uint8_t get_limit_pin_mask(uint8_t axis_idx) {
    if ( axis_idx == X_AXIS ) { return ((1 << X_LIMIT_BIT)); }
    if ( axis_idx == Y_AXIS ) { return ((1 << Y_LIMIT_BIT)); }
    return ((1 << Z_LIMIT_BIT));
}


/**
  * @brief  "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. Grbl employs
            the venerable Bresenham line algorithm to manage and exactly synchronize multi-axis moves.
            Unlike the popular DDA algorithm, the Bresenham algorithm is not susceptible to numerical
            round-off errors and only requires fast integer counters, meaning low computational overhead
            and maximizing the Arduino's capabilities. However, the downside of the Bresenham algorithm
            is, for certain multi-axis motions, the non-dominant axes may suffer from un-smooth step
            pulse trains, or aliasing, which can lead to strange audible noises or shaking. This is
            particularly noticeable or may cause motion issues at low step frequencies (0-5kHz), but
            is usually not a physical problem at higher frequencies, although audible.
             To improve Bresenham multi-axis performance, Grbl uses what we call an Adaptive Multi-Axis
            Step Smoothing (AMASS) algorithm, which does what the name implies. At lower step frequencies,
            AMASS artificially increases the Bresenham resolution without effecting the algorithm's
            innate exactness. AMASS adapts its resolution levels automatically depending on the step
            frequency to be executed, meaning that for even lower step frequencies the step smoothing
            level increases. Algorithmically, AMASS is acheived by a simple bit-shifting of the Bresenham
            step count for each AMASS level. For example, for a Level 1 step smoothing, we bit shift
            the Bresenham step event count, effectively multiplying it by 2, while the axis step counts
            remain the same, and then double the stepper ISR frequency. In effect, we are allowing the
            non-dominant Bresenham axes step in the intermediate ISR tick, while the dominant axis is
            stepping every two ISR ticks, rather than every ISR tick in the traditional sense. At AMASS
            Level 2, we simply bit-shift again, so the non-dominant Bresenham axes can step within any
            of the four ISR ticks, the dominant axis steps every four ISR ticks, and quadruple the
            stepper ISR frequency. And so on. This, in effect, virtually eliminates multi-axis aliasing
            issues with the Bresenham algorithm and does not significantly alter Grbl's performance, but
            in fact, more efficiently utilizes unused CPU cycles overall throughout all configurations.
             AMASS retains the Bresenham algorithm exactness by requiring that it always executes a full
            Bresenham step, regardless of AMASS Level. Meaning that for an AMASS Level 2, all four
            intermediate steps must be completed such that baseline Bresenham (Level 0) count is always
            retained. Similarly, AMASS Level 3 means all eight intermediate steps must be executed.
            Although the AMASS Levels are in reality arbitrary, where the baseline Bresenham counts can
            be multiplied by any integer value, multiplication by powers of two are simply used to ease
            CPU overhead with bitshift integer operations.
             This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
            determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
            defined as constant velocity over n number of steps, from the step segment buffer and then
            executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This
            ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
            after each pulse. The bresenham line tracer algorithm controls all stepper outputs
            simultaneously with these two interrupts.

            NOTE: This interrupt must be as efficient as possible and complete before the next ISR tick,
            which for Grbl must be less than 33.3usec (@30kHz ISR rate). Oscilloscope measured time in
            ISR is 5usec typical and 25usec maximum, well below requirement.
            NOTE: This ISR expects at least one step to be executed per segment.
  * @param  None
  * @retval None
  */
void grbl_stepper_timer_base_irq_callback(void) {
    /* the busy-flag is used to avoid reentering this callback */
    if (stepper.busy) { return; }

    /* stop base timer */
    // grbl_hal_stepper_timer_base_stop();

    /* Set the direction pins a couple of nanoseconds before we step the steppers */
    DIRECTION_PORT->ODR = ((DIRECTION_PORT->ODR & ~DIRECTION_MASK) | (stepper.dir_outbits & DIRECTION_MASK));

    /* pulse the stepping pins */
    #ifdef STEP_PULSE_DELAY
      /* store out_bits to prevent overwriting */
      stepper.step_delay_bits = stepper.step_outbits;
    #else
      grbl_hal_stepper_gpio_set_port(STEP_PORT, STEP_MASK, stepper.step_outbits);
    #endif

    /* Enable step pulse reset timer so that stepper port reset interrupt can reset the signal after
       exactly settings.pulse_microseconds microseconds, independent of the timer_pulse period and prescaler */
    grbl_hal_stepper_timer_pulse_irq_start();
    stepper.busy = true;

    /* if there is no step segment, attempt to pop one from the stepper buffer */
    if (stepper.exec_segment == NULL) {
        /* load and initialize next step segment if buffer not empty */
        if (segment_buffer_head != segment_buffer_tail) {

            /* initialize new step segment and load number of steps to execute */
            stepper.exec_segment = &segment_buffer[segment_buffer_tail];
            /* NOTE: Can sometimes be zero when moving slow */
            stepper.step_count = stepper.exec_segment->n_step;

            /* initialize step segment timing per step and load number of steps to execute */
            grbl_hal_stepper_timer_base_set_reload(stepper.exec_segment->cycles_per_tick - 1);

            #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
              /* if AMASS is disabled, set timer prescaler for segments
                 with slow step frequencies (< 250Hz) */
              grbl_hal_stepper_timer_base_set_prescaler(stepper.exec_segment->prescaler);
            #endif

            /* if the new segment starts a new planner block, initialize stepper variables and counters.
               NOTE: When the segment data index changes, this indicates a new planner block */
            if (stepper.exec_block_index != stepper.exec_segment->st_block_index) {
                stepper.exec_block_index = stepper.exec_segment->st_block_index;
                stepper.exec_block = &st_block_buffer[stepper.exec_block_index];
                /* initialize Bresenham line and distance counters */
                stepper.counter_x = stepper.counter_y = stepper.counter_z = (stepper.exec_block->step_event_count >> 1);
            }
            stepper.dir_outbits = stepper.exec_block->direction_bits ^ dir_port_invert_mask;

            /* With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level */
            #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
              stepper.steps[X_AXIS] = stepper.exec_block->steps[X_AXIS] >> stepper.exec_segment->amass_level;
              stepper.steps[Y_AXIS] = stepper.exec_block->steps[Y_AXIS] >> stepper.exec_segment->amass_level;
              stepper.steps[Z_AXIS] = stepper.exec_block->steps[Z_AXIS] >> stepper.exec_segment->amass_level;
            #endif

            /* Set real-time spindle output as segment is loaded, just prior to the first step */
            #ifdef VARIABLE_SPINDLE
              spindle_set_speed(stepper.exec_segment->spindle_pwm);
            #endif
        }
        else {
            /* segment buffer empty, shutdown */
            st_go_idle();
            #ifdef VARIABLE_SPINDLE
              /* ensure pwm is set properly upon completion of rate-controlled motion */
              if (stepper.exec_block->is_pwm_rate_adjusted) { spindle_set_speed(SPINDLE_PWM_OFF_VALUE); }
            #endif
            /* send to main program for cycle end */
            system_et_exec_state_flag(EXEC_CYCLE_STOP);
            /* nothing to do, exit */
            return;
        }
    }

    /* check probing state */
    if (sys_probe_state == PROBE_ACTIVE) { probe_state_monitor(); }
    /* reset step out bits */
    stepper.step_outbits = 0;

    /* execute step displacement profile by Bresenham line algorithm */
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      stepper.counter_x += stepper.steps[X_AXIS];
      stepper.counter_y += stepper.steps[Y_AXIS];
      stepper.counter_z += stepper.steps[Z_AXIS];
    #else
      stepper.counter_x += stepper.exec_block->steps[X_AXIS];
      stepper.counter_y += stepper.exec_block->steps[Y_AXIS];
      stepper.counter_z += stepper.exec_block->steps[Z_AXIS];
    #endif

    if (stepper.counter_x > stepper.exec_block->step_event_count) {
        stepper.step_outbits |= (1<<X_STEP_BIT);
        stepper.counter_x -= stepper.exec_block->step_event_count;
        if (stepper.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys_position[X_AXIS]--; }
        else { sys_position[X_AXIS]++; }
    }
    if (stepper.counter_y > stepper.exec_block->step_event_count) {
        stepper.step_outbits |= (1<<Y_STEP_BIT);
        stepper.counter_y -= stepper.exec_block->step_event_count;
        if (stepper.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys_position[Y_AXIS]--; }
        else { sys_position[Y_AXIS]++; }
    }
    if (stepper.counter_z > stepper.exec_block->step_event_count) {
        stepper.step_outbits |= (1<<Z_STEP_BIT);
        stepper.counter_z -= stepper.exec_block->step_event_count;
        if (stepper.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys_position[Z_AXIS]--; }
        else { sys_position[Z_AXIS]++; }
    }

    /* during a homing cycle, lock out and prevent desired axes from moving */
    if (sys.state == STATE_HOMING) { stepper.step_outbits &= sys.homing_axis_lock; }

    /* decrement step events count */
    stepper.step_count--;
    if (stepper.step_count == 0) {
      /* segment is complete. Discard current segment and advance segment indexing */
      stepper.exec_segment = NULL;
      if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
    }
    /* Apply step port invert mask */
    stepper.step_outbits ^= step_port_invert_mask;
    /* reset bysy flag */
    stepper.busy = false;
}

/**
  * @brief
  * @param  None
  * @retval None
  */
void grbl_stepper_timer_pulse_irq_callback(void) {
    /* reset step/dir pulse cycle */
    grbl_hal_stepper_gpio_set_port(STEP_PORT, STEP_MASK, step_port_invert_mask & STEP_MASK);
}

#ifdef STEP_PULSE_DELAY
/**
  * @brief
  * @param  None
  * @retval None
  */
void grbl_stepper_timer_pulse_step_delay_irq_callback(void) {
    /* Begin step pulse */
    grbl_hal_stepper_gpio_set_port(STEP_PORT, STEP_MASK, stepper.step_delay_bits);
}
#endif


/*******************************************************************************
      END FILE
*******************************************************************************/