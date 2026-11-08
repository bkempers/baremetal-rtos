add_library(drivers STATIC
    led/led.c
    # console/console.c — awaiting the hal_usart rewrite + new ringbuffer
)

#TODO: want to probably make this dynamic and configurable per 'app'

target_include_directories(drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/console
    ${CMAKE_CURRENT_SOURCE_DIR}/led
    ${CMAKE_SOURCE_DIR}/lib          # compiler.h
)

# console.c keeps the rx/tx line buffers
target_link_libraries(drivers PUBLIC ringbuffer)
