# Boot project
project(Boot C ASM)

# Boot sources
set(BOOT_SOURCES
    main.c
    ${CMAKE_SOURCE_DIR}/startup_stm32h7s3l8hx.c
    ${CMAKE_SOURCE_DIR}/Core/CMSIS/Device/ST/STM32H7RSxx/Source/Templates/system_stm32h7rsxx.c
)

# HAL sources needed for boot
set(BOOT_HAL_SOURCES
    ${CMAKE_SOURCE_DIR}/HAL/src/stm32h7rs_hal.c
    ${CMAKE_SOURCE_DIR}/HAL/src/stm32h7rs_hal_rcc.c
    ${CMAKE_SOURCE_DIR}/HAL/src/stm32h7rs_hal_gpio.c
    ${CMAKE_SOURCE_DIR}/HAL/src/stm32h7rs_hal_xspi.c
)

add_executable(boot.elf ${BOOT_SOURCES} ${BOOT_HAL_SOURCES})

target_include_directories(boot.elf PRIVATE
    inc
    ${CMAKE_SOURCE_DIR}/Core
    ${CMAKE_SOURCE_DIR}/HAL/inc
    ${CMAKE_SOURCE_DIR}/support
)

target_compile_definitions(boot.elf PRIVATE
    STM32H7S3xx
    STM32H7S3L8Hx
    USE_FULL_LL_DRIVER
)

target_compile_options(boot.elf PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -fdata-sections
    -ffunction-sections
    -Wall
    -O2
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Debug>:-Og>
)

target_link_options(boot.elf PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/boot_linker.ld
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/boot.map,--cref
    -Wl,--print-memory-usage
)

# Generate hex and bin files
add_custom_command(TARGET boot.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:boot.elf> ${CMAKE_CURRENT_BINARY_DIR}/boot.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:boot.elf> ${CMAKE_CURRENT_BINARY_DIR}/boot.bin
    COMMENT "Generating boot.hex and boot.bin"
    VERBATIM
)

# Print size
add_custom_command(TARGET boot.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:boot.elf> > ${CMAKE_CURRENT_BINARY_DIR}/boot.size.txt
    COMMENT "Boot: Size analysis"
    VERBATIM
)

# Copy boot.hex to root build directory for easy access
add_custom_command(TARGET boot.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/boot.hex
        ${CMAKE_BINARY_DIR}/boot.hex
    COMMENT "Copying boot.hex to build root"
)
