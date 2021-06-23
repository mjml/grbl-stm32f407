#ifndef cpu_map_stm32_h
#define cpu_map_stm32_h

#include <stm32f4xx_hal_gpio.h>


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

typedef struct stepper {
    gpio_t  pul;
    gpio_t  dir;
    gpio_t  ena;
#ifdef STEPPER_ENCODER_ALARM    
    gpio_t  alm;
#endif
} stepper_t;


/// OUTPUTS //////////////////////////////////////////////

// steppers with closed-loop encoders
#define NUM_MOTORS 5
stepper_t motors[NUM_MOTORS];

// coolant
// TODO: I don't know exactly how these will be wired yet
extern gpio_t coolantflood;
extern gpio_t coolantmist;

// atc
// TODO: I also don't know how these will be wired. 
// These pins control solenoids that gate the pressurized air.
// I'm reserving one for grab and one for some kind of release/flush
extern gpio_t atc1;
extern gpio_t atc2;

// spindle
// TODO: I am a bit fuzzy as what the VFD will require.
//       I think that since GRBL is already written for PWM and the VFD accepts PWM,
//         this section will be determined by grbl's needs.
extern gpio_t spen;
extern gpio_t spdir;
extern gpio_t sppwm;

// auxiliary (fan/lights/speaker)
extern gpio_t fan;
extern gpio_t lights;
extern gpio_t speaker;


/// INPUTS and EXTERNAL INTERRUPTS /////////////////////////////////////

// control block
#define CONTROL_PORT              GPIOE
#define CONTROL_MASK              ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))
#define CONTROL_INVERT_MASK       CONTROL_MASK // May be re-defined to only invert certain control pins.
#define CONTROL_RESET_BIT         1
#define CONTROL_FEED_HOLD_BIT     3
#define CONTROL_SAFETY_DOOR_BIT   2
#define CONTROL_CYCLE_START_BIT   0
#define CONTROL_INPUT_BIT         5
extern gpio_t reset;
extern gpio_t feedhld;
extern gpio_t cstart;
extern gpio_t safety;
extern gpio_t input;

// limits / homes
#define LIMIT_PORT                GPIOE
#define X_LIMIT_BIT               6
#define Y_LIMIT_BIT               7
#define Z_LIMIT_BIT               8
#define A_LIMIT_BIT               9
#define B_LIMIT_BIT               10
#define X_HOME_BIT                X_LIMIT_BIT
#define Y_HOME_BIT                Y_LIMIT_BIT
#define Z_HOME_BIT                Z_LIMIT_BIT
#define A_HOME_BIT                A_LIMIT_BIT
#define B_HOME_BIT                B_LIMIT_BIT
extern gpio_t limits[NUM_LIMITS];


// probing
#define PROBE_PORT                GPIOE
#define PROBE_BIT                 4
extern gpio_t probe;


#define SPINDLE_PWM_MAX_VALUE     1023
#define SPINDLE_PWM_MIN_VALUE     1

#define SPINDLE_PWM_OFF_VALUE     0
#define SPINDLE_PWM_RANGE (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)

/**
 * Hamster's own layer of GPIO pins 
*/
void      gpio_enable (gpio_t* gpio);
bool      gpio_disable (gpio_t* gpio);
void      gpio_write_pin (gpio_t* gpio, bool value);
void      gpio_set_pin (gpio_t* gpio);
void      gpio_reset_pin (gpio_t* gpio);

// Conditionally updates the input pin's shadow bit and returns true if the update was performed, 
// or returns false if it did not need to be performed. This function enables greater GPIO multiplexing.
bool      gpio_shadow (gpio_t* gpio);


///// CAUTION: the port i/o commands are "direct" and don't factor in the above-specified polarity of individual pins

// reads all 16 bits
uint16_t  gpio_read_port (GPIO_TypeDef* hal);

// read only the masked bits
uint16_t  gpio_read_port_mask (GPIO_TypeDef* hal, uint16_t mask);

// writes all 16 bits
void      gpio_write_port (GPIO_TypeDef* hal, uint16_t value);

// sets or resets only the bits given by the mask
void      gpio_write_port_mask (GPIO_TypeDef* hal, uint16_t value, uint16_t mask);

#endif
