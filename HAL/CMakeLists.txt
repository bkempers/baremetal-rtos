add_library(hal STATIC
    src/stm32h7rs_hal.c
    src/stm32h7rs_hal_uart.c
    src/stm32h7rs_hal_gpio.c
    src/stm32h7rs_hal_dma.c
    # Add more as you create them
)

target_include_directories(hal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# HAL can depend on CMSIS (already in global includes)
