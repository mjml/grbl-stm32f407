/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef stepper_h
#define stepper_h

#include <stm32f4xx_hal_tim.h>

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

// Redefine this macro since AVR is based on F_CPU:
// The timers on APB1 suffice for this timebase, which maxes at 42Mhz when the prescaler is set to 1
#define STEP_TICK_FREQ 42000000
#undef TICKS_PER_MICROSECOND
#define TICKS_PER_MICROSECOND (uint32_t)(STEP_TICK_FREQ / 1000000)

#define STEPPER_TIMBASE               TIM7
#define STEPPER_TIMBASE_CLK_ENABLE  __TIM7_CLK_ENABLE
#define STEPPER_RSTBASE               TIM5
#define STEPPER_RSTBASE_CLK_ENABLE  __TIM5_CLK_ENABLE

// This is for closed-loop steppers
#ifdef STEPPER_ENCODER_ALARM
#define __ENCODER_ALARM_HANDLER__ st_alarm_interrupt
#endif

// Initialize and setup the stepper motor subsystem
void stepper_init();

// Enable steppers, but cycle does not start unless called by motion control or realtime command.
void st_wake_up();

// Immediately disables steppers
void st_go_idle();

// Generate the step and direction port invert masks.
void st_generate_step_dir_invert_masks();

// Reset the stepper subsystem variables
void st_reset();

// Changes the run state of the step segment buffer to execute the special parking motion.
void st_parking_setup_buffer();

// Restores the step segment buffer to the normal run state after a parking motion.
void st_parking_restore_buffer();

// Reloads step segment buffer. Called continuously by realtime execution system.
void st_prep_buffer();

// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters();

// Called by realtime status reporting if realtime rate reporting is enabled in config.h.
float st_get_realtime_rate();

// This is the interrupt callback for the main stepper pulse edge.
void st_interrupt();

// This is a separate callback for the timed down-edge of the stepper pulse.
// GRBL uses two pulses instead of an OC mode so that the pulse is guaranteed to reset after an interval,
//   regardless of whether the main pulse timer is still running or not.
void st_rst_interrupt();

#ifdef STEPPER_ENCODER_ALARM
void st_alarm_interrupt();
#endif

extern TIM_HandleTypeDef st_timer;
extern TIM_HandleTypeDef st_rst_timer;

#endif
