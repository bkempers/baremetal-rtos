cmake_minimum_required(VERSION 3.20)

# Must set before project()
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/stm32h7rs_toolchain.cmake)

project(stm_rtos C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Global definitions
add_compile_definitions(
    STM32H7S3xx
    USE_FULL_ASSERT
)

# Global include directories
include_directories(
    Core/CMSIS/Device/ST/STM32H7RSxx/Include
    Core/CMSIS/Core/Include
)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Debug/Release flags
if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_options(-O0 -g3)
    add_compile_definitions(DEBUG)
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    add_compile_options(-O2)
endif()

# Subdirectories
# add_subdirectory(hal)
# add_subdirectory(rtos)
add_subdirectory(Application)
