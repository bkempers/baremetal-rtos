#ifndef LED_H
#define LED_H

#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"

/* LED DEFINES */
#define LED1_PIN (GPIO_PIN_10) // PD10
#define LED2_PIN (GPIO_PIN_13) // PD13
#define LED3_PIN (GPIO_PIN_7)  // PB7

void Led_Init(void);
void Led_Toggle(uint8_t led_num);
void Led_Cycle(void);
void Led_Error(void);
void Led_Reset(void);

// TODO: maybe add later
// void Led_Success(void);

#endif // LED_H
