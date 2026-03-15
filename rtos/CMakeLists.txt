add_library(rtos STATIC
    kernel.c
)

target_include_directories(rtos PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# RTOS might need HAL
target_link_libraries(rtos PUBLIC hal)
