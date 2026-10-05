add_library(drivers STATIC
    # src/hal.c
    # src/hal_rcc.c
    # src/hal_tim.c
    # src/hal_gpio.c
    # src/stm32h7rs_hal_usart.c
    # src/stm32h7rs_hal_i2c.c
    # src/stm32h7rs_hal_spi.c

    # console/console.c
    led/led.c
)

#TODO: want to probably make this dynamic and configurable per 'app'

target_include_directories(drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    # ${CMAKE_CURRENT_SOURCE_DIR}/console
    ${CMAKE_CURRENT_SOURCE_DIR}/led
    ${CMAKE_SOURCE_DIR}/lib          # compiler.h
)
