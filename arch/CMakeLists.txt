#TODO: this needs to be dynamic KConfig/compile variable
add_library(arch STATIC
    arm/cortex_m/cortex_m.c
)

target_include_directories(arch PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/arm/cortex_m
)

# cmsis, not chip: arch must not depend on a specific SoC.
target_link_libraries(arch PRIVATE cmsis)
