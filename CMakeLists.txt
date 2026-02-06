cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/stm32h7rs_toolchain.cmake)

# # ============================================
# # LVGL Configuration for v9.4
# # ============================================
set(LV_BUILD_CONF_PATH 
    ${CMAKE_SOURCE_DIR}/Application/display/lv_conf.h
    CACHE STRING "LVGL config path" FORCE
)

project(application LANGUAGES C ASM)

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

# Git information configuration
find_package(Git QUIET)

if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    
    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --format=%cd --date=iso-strict
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_DATE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    
    execute_process(
        COMMAND ${GIT_EXECUTABLE} diff-index --quiet HEAD --
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        RESULT_VARIABLE GIT_DIRTY_RESULT
        ERROR_QUIET
    )
    
    if(GIT_DIRTY_RESULT EQUAL 0)
        set(GIT_DIRTY "clean")
    else()
        set(GIT_DIRTY "dirty")
    endif()
else()
    set(GIT_BRANCH "unknown")
    set(GIT_COMMIT_HASH "unknown")
    set(GIT_COMMIT_SHORT "unknown")
    set(GIT_COMMIT_DATE "unknown")
    set(GIT_DIRTY "unknown")
endif()

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S" UTC)

# Ensure the include directory exists
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/Application/inc)

# Initial configuration of git_info.h
configure_file(
    ${CMAKE_SOURCE_DIR}/support/git_info.h.in
    ${CMAKE_BINARY_DIR}/Application/inc/git_info.h
    @ONLY
)

# Add custom target to update git info on every build
add_custom_target(
    git_info_check ALL
    COMMAND ${CMAKE_COMMAND}
        -DGIT_EXECUTABLE=${GIT_EXECUTABLE}
        -DSOURCE_DIR=${CMAKE_SOURCE_DIR}
        -DBINARY_DIR=${CMAKE_BINARY_DIR}
        -P ${CMAKE_SOURCE_DIR}/cmake/update_git_info.cmake
        BYPRODUCTS ${CMAKE_BINARY_DIR}/Application/inc/git_info.h
    COMMENT "Updating git information..."
)

# Subdirectories
# add_subdirectory(rtos)
add_subdirectory(HAL)
add_subdirectory(Boot)
add_subdirectory(Application)

add_dependencies(application.elf boot.elf)
