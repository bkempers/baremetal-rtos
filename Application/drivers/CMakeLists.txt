add_library(drivers STATIC
    sensors/bme680/bme680.c
    sensors/bme680/bme680_sensor.c
    sensors/bme680/bme680_defs.h
)

target_include_directories(drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(drivers PUBLIC
    common
)
