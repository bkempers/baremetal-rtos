#include "board.h"

#include "arch.h"
#include "hal.h"
#include "hal_clock.h"
#include "led.h"

#include "stm32h7rs_clock.h"

static const led_obj board_leds[BOARD_LED_COUNT] = {
    [BOARD_LED_GREEN] = {
        .gpio = {
            .gpio      = {.port = GPIO_PORT_D, .pin = LED1_PIN},
            .mode      = GPIO_MODE_OUTPUT_PP,
            .speed     = GPIO_SPEED_FREQ_LOW,
            .pull      = GPIO_NOPULL_UP,
            .alternate = 0,
        },
        .active = GPIO_PIN_SET,
    },
    [BOARD_LED_YELLOW] = {
        .gpio = {
            .gpio      = {.port = GPIO_PORT_D, .pin = LED2_PIN},
            .mode      = GPIO_MODE_OUTPUT_PP,
            .speed     = GPIO_SPEED_FREQ_LOW,
            .pull      = GPIO_NOPULL_UP,
            .alternate = 0,
        },
        .active = GPIO_PIN_SET,
    },
    [BOARD_LED_RED] = {
        .gpio = {
            .gpio      = {.port = GPIO_PORT_B, .pin = LED3_PIN},
            .mode      = GPIO_MODE_OUTPUT_PP,
            .speed     = GPIO_SPEED_FREQ_LOW,
            .pull      = GPIO_NOPULL_UP,
            .alternate = 0,
        },
        .active = GPIO_PIN_SET,
    },
};

static const struct clock_cfg board_clock_cfg = {
    .sysclk_source = RCC_SYSCLKSOURCE_HSI,

    .hse_state   = RCC_HSE_OFF,
    .hsi_state   = RCC_HSI_ON,
    .calibration = 0,

    .pll_state  = RCC_PLL_ON,
    .pll_source = RCC_PLLSOURCE_HSI,
    .pllm       = 4,
    .plln       = 12,
    .pllp       = 2,
    .pllq       = 2,
    .pllr       = 2,
    .pll_frac   = 4096,

    .subsys = {
        [RCC_CLKSOURCE_HCLK] = {.source = RCC_CLKSOURCE_HCLK, .divider = RCC_HCLK_DIV1},
        [RCC_CLKSOURCE_APB1] = {.source = RCC_CLKSOURCE_APB1, .divider = RCC_APB1_DIV1},
        [RCC_CLKSOURCE_APB2] = {.source = RCC_CLKSOURCE_APB2, .divider = RCC_APB2_DIV1},
        [RCC_CLKSOURCE_APB4] = {.source = RCC_CLKSOURCE_APB4, .divider = RCC_APB4_DIV1},
        [RCC_CLKSOURCE_APB5] = {.source = RCC_CLKSOURCE_APB5, .divider = RCC_APB5_DIV1},
    },
};

void board_clock_init(void)
{
    clock_init(&board_clock_cfg);
}

int board_init(void)
{
    arch_init(); /* FPU on before any float runs */

    board_clock_init();

    if (led_init(board_leds, BOARD_LED_COUNT) != 0) {
        return -1;
    }

    return 0;
}

/* Strong override of the __weak default in chip/stm32h7rs/system.c. */
void system_error_handle(void)
{
    /* clock_init() calls this, which is before led_init() has registered the
     * table, so drive the pin directly instead of going through led_set(). */
    static const gpio_init red = {
        .gpio      = {.port = GPIO_PORT_B, .pin = LED3_PIN},
        .mode      = GPIO_MODE_OUTPUT_PP,
        .speed     = GPIO_SPEED_FREQ_LOW,
        .pull      = GPIO_NOPULL_UP,
        .alternate = 0,
    };
    gpio pin = red.gpio;

    hal_gpio_init(&red);
    hal_gpio_write(&pin, GPIO_PIN_SET);

    arch_halt();
}
