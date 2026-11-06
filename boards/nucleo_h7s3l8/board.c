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

/* HSI at 64MHz, undivided. The PLL is deliberately absent: its old settings
 * (M=4 N=12 P=2) gave a 192MHz VCO, below this part's 400MHz minimum, and it
 * was never selected as SYSCLK anyway. */
static const struct clock_cfg board_clock_cfg = {
    .sysclk_source = RCC_SYSCLKSOURCE_HSI,

    .subsys = {
        [RCC_CLKSOURCE_SYS]  = {
            .source = RCC_CLKSOURCE_SYS,  
            .divider = LL_RCC_SYSCLK_DIV_1
        },
        [RCC_CLKSOURCE_HCLK] = {
            .source = RCC_CLKSOURCE_HCLK, 
            .divider = LL_RCC_AHB_DIV_1
        },
        [RCC_CLKSOURCE_APB1] = {
            .source = RCC_CLKSOURCE_APB1, 
            .divider = LL_RCC_APB1_DIV_1
        },
        [RCC_CLKSOURCE_APB2] = {
            .source = RCC_CLKSOURCE_APB2, 
            .divider = LL_RCC_APB2_DIV_1
        },
        [RCC_CLKSOURCE_APB4] = {
            .source = RCC_CLKSOURCE_APB4, 
            .divider = LL_RCC_APB4_DIV_1
        },
        [RCC_CLKSOURCE_APB5] = {
            .source = RCC_CLKSOURCE_APB5, 
            .divider = LL_RCC_APB5_DIV_1
        },
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
