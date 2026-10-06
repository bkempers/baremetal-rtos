#TODO: this needs to be dynamic KConfig/compile variable
add_library(chip STATIC
    stm32h7rs/startup.c
    stm32h7rs/system.c

    stm32h7rs/clock.c
    # stm32h7rs/tim.c
    stm32h7rs/gpio.c
    #i2c/spi/usart/...
)

target_include_directories(chip PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32h7rs
    ${CMAKE_SOURCE_DIR}/core/CMSIS/Device/ST/STM32H7RSxx/Include
    ${CMAKE_SOURCE_DIR}/core/CMSIS/Core/Include
)

target_compile_definitions(chip PUBLIC STM32H7S3xx USE_FULL_ASSERT)

target_link_libraries(chip PUBLIC drivers)
