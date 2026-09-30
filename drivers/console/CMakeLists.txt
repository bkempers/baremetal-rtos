add_library(console STATIC
    console.c
    console.h
)
 
target_include_directories(console PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(console PUBLIC
    common
)

target_compile_options(console PRIVATE -Wall -Wextra -Wpedantic)
