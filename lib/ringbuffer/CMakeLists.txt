add_library(ringbuffer STATIC
    ringbuffer.c
)

target_include_directories(ringbuffer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(ringbuffer PRIVATE -Wall -Wextra -Wpedantic)
