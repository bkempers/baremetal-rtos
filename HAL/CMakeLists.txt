add_library(HAL STATIC
    src/stm32h7rs_hal.c
    src/stm32h7rs_hal_rcc.c
    src/stm32h7rs_hal_gpio.c
    src/stm32h7rs_hal_usart.c
)

target_include_directories(HAL PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# HAL can depend on CMSIS (already in global includes)
