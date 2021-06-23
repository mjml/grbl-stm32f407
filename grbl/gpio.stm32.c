
#include "grbl.h"

// gpio_t definition and declarations are part of the auto-generated gpio_map.[ch]

void gpio_enable (gpio_t* gpio) {
    GPIO_InitTypeDef init;
    init.Pin       = gpio->pinmask;
    init.Mode      = gpio->mode;
    init.Pull      = gpio->pull;
    init.Speed     = gpio->speed;
    init.Alternate = gpio->alternate;
    HAL_GPIO_Init(gpio->hal, &init);
    gpio->enabled = true;
}


void gpio_disable (gpio_t* gpio) {
    HAL_GPIO_DeInit(gpio->hal, gpio->pinmask);
    gpio->enabled = false;
}


bool gpio_read_pin(gpio_t* gpio) {
  uint8_t value = HAL_GPIO_ReadPin(gpio->hal, gpio->pinmask);
  return ((value == 0) ^ !gpio->polarity) ? 0x0 : 0x1;
}


void gpio_write_pin(gpio_t* gpio, bool value) {
  HAL_GPIO_WritePin(gpio->hal, gpio->pinmask, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void gpio_set_pin(gpio_t* gpio) {
  HAL_GPIO_WritePin(gpio->hal, gpio->pinmask, GPIO_PIN_SET);
}


void gpio_reset_pin(gpio_t* gpio) {
  HAL_GPIO_WritePin(gpio->hal, gpio->pinmask, GPIO_PIN_RESET);
}


void gpio_shadow(gpio_t* gpio) {
  bool pinvalue = gpio_read_pin(gpio);
  if (pinvalue != gpio->shadow_input) {
    gpio->shadow_input = pinvalue;
    return true;
  } else {
    return false;
  }
}


uint16_t gpio_read_port(GPIO_TypeDef* hal) { 
    uint16_t result = 0;
    for (int i=0; i < sizeof(uint16_t)*2; i++) {
        uint16_t curmask = bit(i);
        bool bit = HAL_GPIO_ReadPin(hal, curmask);
        if (bit) {
            result |= curmask;
        }
    }
    return result;
}


uint16_t gpio_read_port_mask(GPIO_TypeDef* hal, uint16_t mask) {
    uint16_t result = 0;
    for (int i=0; i < sizeof(uint16_t)*2; i++) {
        uint16_t curmask = bit(i);
        if (mask & curmask) {
            bool bit = HAL_GPIO_ReadPin(hal, curmask);
            if (bit) {
                result |= curmask;
            }
        }
    }
    return result;
}


void gpio_write_port(GPIO_TypeDef* hal, uint16_t value) {
  for (int i = 0; i < sizeof(uint16_t) * 8; i++) {
    uint16_t curmask = bit(i);
    if (value & curmask) {
      HAL_GPIO_WritePin(hal, curmask, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(hal, curmask, GPIO_PIN_RESET);
    }
  }
}


void gpio_write_port_mask (GPIO_TypeDef* hal, uint16_t value, uint16_t mask) {
  for (int i = 0; i < sizeof(uint16_t) * 8; i++) {
    uint16_t curmask = bit(i);
    if (mask & curmask) {
      if (value & curmask) {
        HAL_GPIO_WritePin(hal, curmask, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(hal, curmask, GPIO_PIN_RESET);
      }
    }
  }
}




