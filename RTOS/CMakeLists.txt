add_library(rtos STATIC
    scheduler.c
    task.c
    semaphore.c
    queue.c
)

target_include_directories(rtos PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# RTOS might need HAL
target_link_libraries(rtos PUBLIC hal)
