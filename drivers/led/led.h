#ifndef LED_H
#define LED_H

#include "../hal_gpio.h"
#include "../hal_rcc.h"

// TODO: this needs to be uncoupled from stm32h7rs

/* LED DEFINES */
#define LED1_PIN (GPIO_PIN_10) // PD10
#define LED2_PIN (GPIO_PIN_13) // PD13
#define LED3_PIN (GPIO_PIN_7)  // PB7

void led_init(void);
void led_toggle(uint8_t led_num);
void led_cycle(void);
void led_error(void);
void led_reset(void);

// TODO: maybe add later
// void Led_Success(void);

#endif // LED_H
