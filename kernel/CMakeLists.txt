add_library(kernel STATIC
    kernel.c
    kernel_isr.c
    time.c
)

target_include_directories(kernel PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(kernel PUBLIC drivers)
