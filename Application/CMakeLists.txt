# Collect all application source files
set(APP_SOURCES
    # Main entry point
    main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/console.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/task_scheduler.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/adc.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/exti.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/led_driver.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/syscalls.c
    # # ${CMAKE_CURRENT_SOURCE_DIR}/src/sysmem.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/systick.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/timer.c
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/uart_driver.c
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
    ${CMAKE_BINARY_DIR}/Application/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/inc      # src/inc/
    ${CMAKE_CURRENT_SOURCE_DIR}          # src/ (for main.h if needed)
)

# Link libraries
target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    HAL
    # rtos
)

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
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)

# Make the executable depend on git_info updates
add_dependencies(${PROJECT_NAME}.elf git_info_check)

# Explicitly mark git_info.h as a dependency for sources that include it
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/console.c
    PROPERTIES OBJECT_DEPENDS ${CMAKE_BINARY_DIR}/Application/inc/git_info.h
)

# Flash target
set(STM32_PROG "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI")

add_custom_target(flash
    COMMAND ${STM32_PROG} -c port=SWD freq=4000 mode=UR reset=HWrst -w ${PROJECT_NAME}.elf -v -rst -q
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing ${PROJECT_NAME}.elf"
)

add_custom_target(erase
    COMMAND ${STM32_PROG} -c port=SWD -e all
    COMMENT "Erasing flash"
)

set(STLINK_GDBSERVER "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.macos64_2.2.200.202505060755/tools/bin/ST-LINK_gdbserver")
set(STM32_PROG_DIR "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin")

add_custom_target(gdb-server
    COMMAND ${STLINK_GDBSERVER} 
        -p 61234
        -d
        -s
        -cp ${STM32_PROG_DIR}
        -m 1
        --halt
    COMMENT "Starting ST-LINK GDB server on port 61234"
)
