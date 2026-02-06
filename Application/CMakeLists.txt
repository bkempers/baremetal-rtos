# ============================================
# LVGL Configuration
# ============================================
message("-- Adding LVGL v9.4")
set(LV_CONF_BUILD_DISABLE_EXAMPLES ON CACHE BOOL "" FORCE)
set(LV_CONF_BUILD_DISABLE_DEMOS ON CACHE BOOL "" FORCE)

set(CONFIG_LV_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CONFIG_LV_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
add_subdirectory(external/lvgl)

target_include_directories(lvgl SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# ============================================
# Common Configuration Interface Library
# ============================================
add_library(common INTERFACE)
target_include_directories(common INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_BINARY_DIR}/Application/inc      # For generated headers like git_info.h
    ${CMAKE_CURRENT_SOURCE_DIR}/console
    ${CMAKE_CURRENT_SOURCE_DIR}/display
    ${CMAKE_CURRENT_SOURCE_DIR}/led
    ${CMAKE_CURRENT_SOURCE_DIR}/system
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensors
)
target_link_libraries(common INTERFACE
    HAL
    lvgl
)

add_subdirectory(console)
add_subdirectory(system)
add_subdirectory(drivers)
add_subdirectory(tasks)
add_subdirectory(tools)
add_subdirectory(led)
add_subdirectory(display)

set(SYSTEM_SOURCES
    ${CMAKE_SOURCE_DIR}/startup_stm32h7s3l8hx.c
    ${CMAKE_SOURCE_DIR}/Core/CMSIS/Device/ST/STM32H7RSxx/Source/Templates/system_stm32h7rsxx.c
)

# Create executable
add_executable(${PROJECT_NAME}.elf
    main.c
    ${SYSTEM_SOURCES}
)

# Add application include directories
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${CMAKE_BINARY_DIR}/Application/inc
    ${CMAKE_BINARY_DIR}/Application/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/inc      # src/inc/
    ${CMAKE_CURRENT_SOURCE_DIR}          # src/ (for main.h if needed)
)

# Link libraries
target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    system
    console
    drivers
    tasks
    tools
    led
    display
    common
    lvgl
    HAL
    # rtos
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/app_linker.ld)
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,-u,_printf_float
    -Wl,-u,_scanf_float
    # Group libraries so syscalls are found correctly
    -nodefaultlibs
    -lc
    -lm
    -lgcc
)

# Generate hex and bin files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
    VERBATIM
)

# Make the executable depend on git_info updates
add_dependencies(${PROJECT_NAME}.elf git_info_check)

# Explicitly mark git_info.h as a dependency for sources that include it
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/console/console.c
    PROPERTIES OBJECT_DEPENDS ${CMAKE_BINARY_DIR}/Application/inc/git_info.h
)

# Flash target
set(STM32_PROG "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI")

# Flash boot only (to internal flash)
add_custom_target(flash-boot
    COMMAND ${STM32_PROG} -c port=SWD freq=950 mode=UR reset=HWrst 
            -w ${CMAKE_BINARY_DIR}/Boot/boot.hex 
            -v -rst -q
    DEPENDS boot.elf
    COMMENT "Flashing boot.hex to internal flash"
)

# Flash application only (to external flash - requires external loader)
add_custom_target(flash-app
    COMMAND ${STM32_PROG} -c port=SWD freq=950 mode=UR reset=HWrst
            -el ${CMAKE_SOURCE_DIR}/MX25UW25645G_NUCLEO-H7S3L8.stldr
            -w ${CMAKE_BINARY_DIR}/Application/${PROJECT_NAME}.elf
            -v -rst -q
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing ${PROJECT_NAME}.elf to external flash"
)

# Flash both boot and application (default flash target)
add_custom_target(flash
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target flash-boot
    COMMAND ${CMAKE_COMMAND} -E sleep 1
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target flash-app
    DEPENDS boot.elf ${PROJECT_NAME}.elf
    COMMENT "Flashing boot + application"
)

# Erase everything (including external flash)
add_custom_target(erase
    COMMAND ${STM32_PROG} -c port=SWD freq=950 mode=UR
            -el ${CMAKE_SOURCE_DIR}/MX25UW25645G_NUCLEO-H7S3L8.stldr
            -e all
            -q
    COMMENT "Erasing internal and external flash"
)

# add_custom_target(flash
#     COMMAND ${STM32_PROG} -c port=SWD freq=950 mode=UR reset=HWrst -w ${PROJECT_NAME}.elf -v -rst -q
#     DEPENDS ${PROJECT_NAME}.elf
#     COMMENT "Flashing ${PROJECT_NAME}.elf"
# )
#
# add_custom_target(erase
#     COMMAND ${STM32_PROG} -c port=SWD -e all
#     COMMENT "Erasing flash"
# )

# OpenOCD flash target
add_custom_target(openocd_flash
    COMMAND openocd -f ${CMAKE_SOURCE_DIR}/openocd.cfg -c "program ${PROJECT_NAME}.elf verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Flashing with OpenOCD: ${PROJECT_NAME}.elf"
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
