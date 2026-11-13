#ifndef BOARD_H
#define BOARD_H

#include "hal_gpio.h"

/* LED DEFINES */
#define LED1_PIN (GPIO_PIN_10) // PD10
#define LED2_PIN (GPIO_PIN_13) // PD13
#define LED3_PIN (GPIO_PIN_7)  // PB7

/* BUTTON DEFINES */
#define BUTTON_PIN (GPIO_PIN_13) // PC13

typedef enum {
    BOARD_LED_GREEN = 0, // LED1, PD10
    BOARD_LED_YELLOW,    // LED2, PD13
    BOARD_LED_RED,       // LED3, PB7
    BOARD_LED_COUNT
} board_led;

typedef enum {
    BOARD_USER_BUTTON = 0,
    BOARD_BUTTON_COUNT
} board_button;

int board_init(void);
void board_clock_init(void);
void board_console_init(void);

#endif
