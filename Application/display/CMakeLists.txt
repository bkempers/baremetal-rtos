add_library(display STATIC
    display.c
    display.h
    lv_conf.h
    st7789/st7789.c
    st7789/st7789.h
)
 
target_include_directories(display PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/st7789
)

target_link_libraries(display PUBLIC
    common
)

target_compile_options(display PRIVATE -Wall -Wextra -Wpedantic)
