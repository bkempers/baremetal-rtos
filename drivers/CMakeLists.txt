add_library(drivers STATIC
    src/stm32h7rs_hal.c
    src/stm32h7rs_hal_rcc.c
    src/stm32h7rs_hal_gpio.c
    src/stm32h7rs_hal_usart.c
    src/stm32h7rs_hal_tim.c
    src/stm32h7rs_hal_i2c.c
    src/stm32h7rs_hal_spi.c

    # console/console.c
    led/led.c
)

#TODO: want to probably make this dynamic and configurable per 'app'

target_include_directories(drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/console
    ${CMAKE_CURRENT_SOURCE_DIR}/led
)

# Drivers (HAL) can depend on CMSIS (already in global includes)
