# Collect all application source files
set(APP_SOURCES
    # Main entry point
    main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/led_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/syscalls.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/systick.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/timer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uart_driver.c
)

set(SYSTEM_SOURCES
    ${CMAKE_SOURCE_DIR}/startup_stm32h7s3l8hx.c
    ${CMAKE_SOURCE_DIR}/Core/CMSIS/Device/ST/STM32H7RSxx/Source/Templates/system_stm32h7rsxx.c
)

# Create executable
add_executable(${PROJECT_NAME}.elf
    ${APP_SOURCES}
    ${SYSTEM_SOURCES}
)

# Add application include directories
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc      # src/inc/
    ${CMAKE_CURRENT_SOURCE_DIR}          # src/ (for main.h if needed)
)

# Link libraries
# target_link_libraries(${PROJECT_NAME}.elf PRIVATE
#     hal
#     rtos
# )

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker_script.ld)
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map
    # floating point linker additions
    -Wl,-u,_printf_float
    -Wl,-u,_scanf_float
)

# Generate hex and bin files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)

# Flash target
set(STM32_PROG "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI")

add_custom_target(flash
    COMMAND ${STM32_PROG} -c port=SWD -w ${PROJECT_NAME}.elf -v -rst -q
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing ${PROJECT_NAME}.elf"
)

add_custom_target(erase
    COMMAND ${STM32_PROG} -c port=SWD -e all
    COMMENT "Erasing flash"
)

add_custom_target(gdb-server
    COMMAND ${STM32_PROG} -c port=SWD -gdb 3333
    COMMENT "Starting GDB server on port 3333"
)
