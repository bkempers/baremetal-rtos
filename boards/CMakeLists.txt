#TODO: this needs to be dynamic KConfig/compile variable
add_library(boards STATIC
    nucleo_h7s3l8/board.c
)

target_include_directories(boards PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/nucleo_h7s3l8
)

target_link_libraries(boards PUBLIC drivers chip arch)
