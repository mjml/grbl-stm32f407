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

// steppers with closed-loop encoders
extern gpio_t xpul;
extern gpio_t xdir;
extern gpio_t xena;
extern gpio_t xalm;

extern gpio_t ypul;
extern gpio_t ydir;
extern gpio_t yena;
extern gpio_t yalm;

extern gpio_t zpul;
extern gpio_t zdir;
extern gpio_t zena;
extern gpio_t zalm;

extern gpio_t apul;
extern gpio_t adir;
extern gpio_t aena;
extern gpio_t aalm;
extern gpio_t bdir;
extern gpio_t bena;
extern gpio_t balm;
extern gpio_t ylim;
extern gpio_t zlim;

// pendant control / door
extern gpio_t reset;
extern gpio_t feedhld;
extern gpio_t cstart;
extern gpio_t safety;

// probing
extern gpio_t probe;

// coolant
// TODO: I don't know exactly how these will be wired yet
extern gpio_t coolantflood;
extern gpio_t coolantmist;

// atc
// TODO: I also don't know how these will be wired. I'm reserving one for grab and one for some kind of release/flush
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



#define SPINDLE_PWM_MAX_VALUE 1023
#define SPINDLE_PWM_MIN_VALUE 1

#define SPINDLE_PWM_OFF_VALUE 0
#define SPINDLE_PWM_RANGE (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)



#endif

