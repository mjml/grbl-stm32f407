#ifndef cpu_map_stm32_h
#define cpu_map_stm32_h

#pragma once

#include <stm32f4xx_hal_gpio.h>
#include <stdbool.h>

/**
 * Hamster's GPIO layer. Runs on STM32 HAL.
 */ 
typedef struct gpio_s {
    GPIO_TypeDef* hal;
    uint32_t      pin;
    uint32_t      pinmask;
    bool          polarity;     // when true, flip the bit upon reading and writing. Useful for NO/NC switches with pull-ups or pull-downs.
    bool          enabled;
    bool          shadow_input; // for edge detection and interrupt multiplexing
    uint32_t      mode; 
    uint32_t      pull;
    uint32_t      speed;
    uint32_t      alternate;
} gpio_t;


typedef struct stepper_motor_s {
    gpio_t  pul;
    gpio_t  dir;
    gpio_t  ena;
#ifdef STEPPER_ENCODER_ALARM    
    gpio_t  alm;
#endif
} stepper_motor_t;


void      gpio_enable (gpio_t* gpio);
void      gpio_disable (gpio_t* gpio);
bool      gpio_read_pin(gpio_t* gpio);
void      gpio_write_pin (gpio_t* gpio, bool value);
void      gpio_set_pin (gpio_t* gpio);
void      gpio_reset_pin (gpio_t* gpio);

// Conditionally updates the input pin's shadow bit and returns true if the update was performed, 
// or returns false if it did not need to be performed. This function enables greater GPIO multiplexing.
bool      gpio_shadow (gpio_t* gpio);

///// CAUTION: the port i/o commands are "direct" and don't factor in the above-specified polarity of individual pins.

// reads all 16 bits
uint16_t  gpio_read_port (GPIO_TypeDef* hal);

// read only the masked bits
uint16_t  gpio_read_port_mask (GPIO_TypeDef* hal, uint16_t mask);

// writes all 16 bits
void      gpio_write_port (GPIO_TypeDef* hal, uint16_t value);

// sets or resets only the bits given by the mask
void      gpio_write_port_mask (GPIO_TypeDef* hal, uint16_t value, uint16_t mask);


#endif
