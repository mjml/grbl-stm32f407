/*
  grbl.h - main Grbl include file
  Part of Grbl

  Copyright (c) 2015-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef grbl_h
#define grbl_h

#include <stdarg.h>

// Grbl versioning system
#define MICROMACHINE_VERSION "0.1"
#define MICROMACHINE_VERSION_BUILD MICROMACHINE_COMMIT_HASH

#ifdef STM32
// AVR-optimization PSTR not used by STM32, so no-op it by defining it as the identity.
#define PSTR(args...) (args)
#endif

// Define standard libraries used by Grbl.
#ifdef AVR
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#elif STM32
// cmsis
#include <stm32f407xx.h>
// hal
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_cortex.h>
#include <stm32f4xx_hal_dma.h>
#include <stm32f4xx_hal_flash.h>
#include <stm32f4xx_hal_tim.h>
#include <stm32f4xx_ll_tim.h>

#include <stm32f4xx_hal_gpio.h>
// usb core
#include <usbd_conf.h>
#include <usbd_core.h>
#include <usbd_def.h>
#include <usbd_desc.h>
#include <usbd_ioreq.h>
// usb cdc
#include <usbd_cdc.h>
#include <usbd_cdc_if.h>
#else 
#error "You must define AVR or STM32 in order to compile this project."
#endif

#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Define the Grbl system include files. NOTE: Do not alter organization.
#include "config.h"
#include "nuts_bolts.h"

#include "defaults.h"

#ifdef STM32
#include "log.h"
#endif

#ifdef AVR
#include "cpu_map.h"
#elif STM32
#include "gpio.stm32.h"
#include "gpio_map.h"
#endif

#ifdef AVR
#include "settings.h"
#include "system.h"
#elif STM32
#include "settings.stm32.h"
#include "system.stm32.h"
#endif

#include "planner.h"

#ifdef AVR
#include "coolant_control.h"
#elif STM32
#include "coolant_control.stm32.h"
#endif

#ifdef AVR
#include "eeprom.h"
#elif STM32
#include "flash.h"
#endif
#include "gcode.h"

#ifdef AVR
#include "limits.h"
#elif STM32
#include "limits.stm32.h"
#endif


#include "motion_control.h"
#include "planner.h"
#include "print.h"
#ifdef AVR
#include "probe.h"
#elif STM32
#include "probe.stm32.h"
#endif

#include "protocol.h"
#include "report.h"
#ifdef AVR
#include "serial.h"
#elif STM32
#include "serial.stm32.h"
#endif

#ifdef AVR
#include "spindle_control.h"
#elif STM32
#include "spindle_control.stm32.h"
#endif

#ifdef AVR
#include "stepper.h"
#elif STM32
#include "stepper.stm32.h"
#endif

#include "jog.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif

// ---------------------------------------------------------------------------------------
// COMPILE-TIME ERROR CHECKING OF DEFINE VALUES:

#ifndef HOMING_CYCLE_0
  #error "Required HOMING_CYCLE_0 not defined."
#endif

/* USE_SPINDLE_DIR_AS_ENABLE_PIN is an AVR artifact and is going away.
#if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(VARIABLE_SPINDLE)
  #error "USE_SPINDLE_DIR_AS_ENABLE_PIN may only be used with VARIABLE_SPINDLE enabled"
#endif

#if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(CPU_MAP_ATMEGA328P)
  #error "USE_SPINDLE_DIR_AS_ENABLE_PIN may only be used with a 328p processor"
#endif

#if !defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)
  #error "SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED may only be used with USE_SPINDLE_DIR_AS_ENABLE_PIN enabled"
#endif
*/

#if defined(PARKING_ENABLE)
  #if defined(HOMING_FORCE_SET_ORIGIN)
    #error "HOMING_FORCE_SET_ORIGIN is not supported with PARKING_ENABLE at this time."
  #endif
#endif

#if defined(ENABLE_PARKING_OVERRIDE_CONTROL)
  #if !defined(PARKING_ENABLE)
    #error "ENABLE_PARKING_OVERRIDE_CONTROL must be enabled with PARKING_ENABLE."
  #endif
#endif

#if defined(SPINDLE_PWM_MIN_VALUE)
  #if !(SPINDLE_PWM_MIN_VALUE > 0)
    #error "SPINDLE_PWM_MIN_VALUE must be greater than zero."
  #endif
#endif

#if (REPORT_WCO_REFRESH_BUSY_COUNT < REPORT_WCO_REFRESH_IDLE_COUNT)
  #error "WCO busy refresh is less than idle refresh."
#endif
#if (REPORT_OVR_REFRESH_BUSY_COUNT < REPORT_OVR_REFRESH_IDLE_COUNT)
  #error "Override busy refresh is less than idle refresh."
#endif
#if (REPORT_WCO_REFRESH_IDLE_COUNT < 2)
  #error "WCO refresh must be greater than one."
#endif
#if (REPORT_OVR_REFRESH_IDLE_COUNT < 1)
  #error "Override refresh must be greater than zero."
#endif

#if defined(ENABLE_DUAL_AXIS)
  #if !((DUAL_AXIS_SELECT == X_AXIS) || (DUAL_AXIS_SELECT == Y_AXIS))
    #error "Dual axis currently supports X or Y axes only."
  #endif
  #if defined(DUAL_AXIS_CONFIG_CNC_SHIELD_CLONE) && defined(VARIABLE_SPINDLE)
    #error "VARIABLE_SPINDLE not supported with DUAL_AXIS_CNC_SHIELD_CLONE."
  #endif
  #if defined(DUAL_AXIS_CONFIG_CNC_SHIELD_CLONE) && defined(DUAL_AXIS_CONFIG_PROTONEER_V3_51)
    #error "More than one dual axis configuration found. Select one."
  #endif
  #if !defined(DUAL_AXIS_CONFIG_CNC_SHIELD_CLONE) && !defined(DUAL_AXIS_CONFIG_PROTONEER_V3_51)
    #error "No supported dual axis configuration found. Select one."
  #endif
  #if defined(COREXY)
    #error "CORE XY not supported with dual axis feature."
  #endif
  #if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
    #error "USE_SPINDLE_DIR_AS_ENABLE_PIN not supported with dual axis feature."
  #endif
  #if defined(ENABLE_M7)
    #error "ENABLE_M7 not supported with dual axis feature."
  #endif
#endif

// ---------------------------------------------------------------------------------------

#endif
