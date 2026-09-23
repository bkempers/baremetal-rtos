add_library(tasks STATIC
    task_scheduler.c
    task_scheduler.h
)
 
target_include_directories(tasks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(tasks PUBLIC
    common
)

target_compile_options(tasks PRIVATE -Wall -Wextra -Wpedantic)
