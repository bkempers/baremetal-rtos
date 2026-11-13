add_library(button STATIC
    button.c
    button.h
)
 
target_include_directories(button PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/drivers
)

target_link_libraries(button PUBLIC
    common
)
 
target_compile_options(button PRIVATE -Wall -Wextra -Wpedantic)
