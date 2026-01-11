add_library(system STATIC
    system.c
    sysmem.c
    syscalls.c
    system.h
)
 
target_include_directories(system PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(system PUBLIC
    common
)

target_compile_options(system PRIVATE -Wall -Wextra -Wpedantic)
