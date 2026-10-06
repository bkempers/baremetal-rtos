cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/stm32h7rs_toolchain.cmake)

project(baremetal_rtos LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S" UTC)

# Subdirectories
add_subdirectory(lib)
add_subdirectory(drivers)
add_subdirectory(arch)
add_subdirectory(chip)
add_subdirectory(boards)
add_subdirectory(kernel)

# User application
add_subdirectory(application)
