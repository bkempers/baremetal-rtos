add_library(led STATIC
    led.c
    led.h
)
 
target_include_directories(led PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(led PUBLIC
    common
)
 
target_compile_options(led PRIVATE -Wall -Wextra -Wpedantic)
