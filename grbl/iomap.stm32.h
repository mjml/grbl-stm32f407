#ifndef cpu_map_stm32_h
#define cpu_map_stm32_h

#include <stm32f4xx_hal_gpio.h>

typedef struct gpio_s {
    GPIO_TypeDef* gpio;
    uint32_t      pin;
    bool polarity;
    uint32_t mode; 
    uint32_t pull;
    uint32_t speed;
    uint32_t alternate;

} gpio_t;


#define PINDEF(port,pin,pole,mode,pull) { port, pin, pole, mode, pull, GPIO_SPEED_FREQ_LOW, 0 }

// steppers with closed-loop encoders
gpio_t xpul = PINDEF(GPIOC, GPIO_PIN_7,  false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t xdir = PINDEF(GPIOD, GPIO_PIN_0,  false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t xena = PINDEF(GPIOF, GPIO_PIN_1,  false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t xalm = PINDEF(GPIOG, GPIO_PIN_7,  true,  GPIO_MODE_INPUT,     GPIO_PULLDOWN);

gpio_t ypul = PINDEF(GPIOC, GPIO_PIN_13, false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t ydir = PINDEF(GPIOC, GPIO_PIN_11, false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t yena = PINDEF(GPIOF, GPIO_PIN_3,  false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t yalm = PINDEF(GPIOG, GPIO_PIN_9,  true,  GPIO_MODE_OUTPUT_OD, GPIO_PULLDOWN);

gpio_t zpul = PINDEF(GPIOD, GPIO_PIN_6,  false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t zdir = PINDEF(GPIOC, GPIO_PIN_9,  false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t zena = PINDEF(GPIOF, GPIO_PIN_5,  false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t zalm = PINDEF(GPIOG, GPIO_PIN_11, true,  GPIO_MODE_OUTPUT_OD, GPIO_PULLDOWN);

gpio_t apul = PINDEF(GPIOE, GPIO_PIN_3, false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t adir = PINDEF(GPIOB, GPIO_PIN_1, false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t aena = PINDEF(GPIOF, GPIO_PIN_7, false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);

gpio_t bpul = PINDEF(GPIOE, GPIO_PIN_1, false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t bdir = PINDEF(GPIOC, GPIO_PIN_5, false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
gpio_t bena = PINDEF(GPIOF, GPIO_PIN_9, false, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);

// limits
gpio_t xlim = PINDEF(GPIOB, GPIO_PIN_9, true,  GPIO_MODE_INPUT,     GPIO_PULLDOWN);
gpio_t ylim = PINDEF(GPIOB, GPIO_PIN_7, true,  GPIO_MODE_INPUT,     GPIO_PULLDOWN);
gpio_t zlim = PINDEF(GPIOB, GPIO_PIN_5, true,  GPIO_MODE_INPUT,     GPIO_PULLDOWN);

// pendant control / door
gpio_t reset   = PINDEF(GPIOA, GPIO_PIN_0,  true,  GPIO_MODE_INPUT,   GPIO_PULLDOWN);
gpio_t feedhld = PINDEF(GPIOA, GPIO_PIN_2,  true,  GPIO_MODE_INPUT,   GPIO_PULLDOWN);
gpio_t cstart  = PINDEF(GPIOF, GPIO_PIN_11, true,  GPIO_MODE_INPUT,   GPIO_PULLDOWN);
gpio_t safety  = PINDEF(GPIOA, GPIO_PIN_2,  true,  GPIO_MODE_INPUT,   GPIO_PULLDOWN);

// probing
gpio_t probe   = PINDEF(GPIOA, GPIO_PIN_6,  true,  GPIO_MODE_INPUT,   GPIO_PULLDOWN);

// coolant
// TODO: I don't know exactly how these will be wired yet
gpio_t coolantflood = PINDEF(GPIOE, GPIO_PIN_15, true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
gpio_t coolantmist  = PINDEF(GPIOB, GPIO_PIN_11, true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

// atc
// TODO: I also don't know how these will be wired. I'm reserving one for grab and one for some kind of release/flush
gpio_t atc1  = PINDEF(GPIOB, GPIO_PIN_14, true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
gpio_t atc2  = PINDEF(GPIOB, GPIO_PIN_12, true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

// spindle
// TODO: I am a bit fuzzy as what the VFD will require.
//       I think that since GRBL is already written for PWM and the VFD accepts PWM,
//         this section will be determined by grbl's needs.
gpio_t spen    = PINDEF(GPIOD, GPIO_PIN_15, true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
gpio_t spdir   = PINDEF(GPIOD, GPIO_PIN_13, true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
gpio_t sppwm   = PINDEF(GPIOE, GPIO_PIN_5,  true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

// auxiliary (fan/lights/speaker)
gpio_t fan     = PINDEF(GPIOE, GPIO_PIN_11, true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
gpio_t lights  = PINDEF(GPIOE, GPIO_PIN_13, true, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
gpio_t speaker = PINDEF(GPIOA, GPIO_PIN_4,  true, GPIO_MODE_ANALOG,    GPIO_NOPULL);



#define SPINDLE_PWM_MAX_VALUE 1023
#define SPINDLE_PWM_MIN_VALUE 1

#define SPINDLE_PWM_OFF_VALUE 0
#define SPINDLE_PWM_RANGE (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)



#endif
