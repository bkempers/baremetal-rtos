#ifndef BOARD_H
#define BOARD_H

#include "hal_gpio.h"

/* LED DEFINES */
#define LED1_PIN (GPIO_PIN_10) // PD10
#define LED2_PIN (GPIO_PIN_13) // PD13
#define LED3_PIN (GPIO_PIN_7)  // PB7

typedef enum {
    BOARD_LED_GREEN = 0, // LED1, PD10
    BOARD_LED_YELLOW,    // LED2, PD13
    BOARD_LED_RED,       // LED3, PB7
    BOARD_LED_COUNT
} board_led;

int board_init(void);
void board_clock_init(void);

#endif
