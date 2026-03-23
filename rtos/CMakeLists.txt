add_library(rtos STATIC
    kernel.c
    kernel_isr.c
)

target_include_directories(rtos PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(rtos PUBLIC HAL)
