add_library(tools STATIC
    ringbuffer.c
    ringbuffer.h
)
 
target_include_directories(tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(tools PRIVATE -Wall -Wextra -Wpedantic)
