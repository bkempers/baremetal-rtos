#TODO: this needs to be dynamic KConfig/compile variable
add_library(chip STATIC
    stm32h7rs/startup.c
    stm32h7rs/system.c
)

target_include_directories(chip PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32h7rs
)

target_link_libraries(chip PUBLIC drivers)
