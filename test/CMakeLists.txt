cmake_minimum_required(VERSION 3.20)

# Standalone host project — NOT a subdirectory of the firmware build. The
# top-level CMakeLists.txt pins cmake/stm32h7rs_toolchain.cmake, so that tree
# can only be cross-compiled. Unit tests run natively and need their own root:
#
#   cmake -S test -B build/test && ctest --test-dir build/test
#
# or just `make test` from the repo root.
project(baremetal_rtos_tests LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

include(CTest)

# Unity — vendored, see framework/VERSION
add_library(unity STATIC framework/unity.c)
target_include_directories(unity PUBLIC framework)
target_compile_definitions(unity PUBLIC
    UNITY_INCLUDE_PRINT_FORMATTED
    UNITY_INCLUDE_EXEC_TIME
    UNITY_OUTPUT_COLOR
)

set(SANITIZERS -fsanitize=address,undefined -fno-omit-frame-pointer)

# Code under test
add_library(cut_ringbuffer STATIC ${REPO_ROOT}/lib/ringbuffer/ringbuffer.c)
target_include_directories(cut_ringbuffer PUBLIC ${REPO_ROOT}/lib/ringbuffer)
target_compile_options(cut_ringbuffer PRIVATE -Wall -Wextra -g ${SANITIZERS})
target_link_options(cut_ringbuffer PUBLIC ${SANITIZERS})

add_executable(test_ringbuffer unit/test_ringbuffer.c)
target_link_libraries(test_ringbuffer PRIVATE unity cut_ringbuffer)
target_compile_options(test_ringbuffer PRIVATE -Wall -Wextra -g ${SANITIZERS})

add_test(NAME test_ringbuffer COMMAND test_ringbuffer)
