#TODO: this needs to be dynamic KConfig/compile variable
add_library(chip STATIC
    stm32h7rs/startup.c
    stm32h7rs/system.c

    stm32h7rs/clock.c
    # stm32h7rs/tim.c
    stm32h7rs/gpio.c
    # stm32h7rs/usart.c
    #i2c/spi/usart/...
)

target_include_directories(chip PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32h7rs
)

# USE_FULL_LL_DRIVER/USE_FULL_ASSERT and the CMSIS headers come via stm32_ll
target_link_libraries(chip PUBLIC drivers stm32_ll)
