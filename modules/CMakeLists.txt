# Arm CMSIS Core + ST CMSIS Device, replacing the vendored core/CMSIS tree.
# Header-only apart from system_stm32h7rsxx.c, which provides SystemInit().
#TODO: this needs to be dynamic KConfig/compile variable
add_library(cmsis STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hal_stm32/cmsis-device-h7rs/Source/Templates/system_stm32h7rsxx.c
)

# CMAKE_CURRENT_SOURCE_DIR is on the list for cmsis_device.h, the SoC-agnostic
# entry point arch/ includes instead of reaching into chip/.
target_include_directories(cmsis PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis_core/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/hal_stm32/cmsis-device-h7rs/Include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(cmsis PUBLIC STM32H7S3xx)

add_subdirectory(hal_stm32)
