# ST LL drivers. Only *_ll_*.c is listed: the HAL sources share Inc/ with the
# LL headers, so keeping them out of the build is what keeps us off the HAL.
add_library(stm32_ll STATIC
    stm32h7rs_hal_driver/Src/stm32h7rsxx_ll_utils.c
    stm32h7rs_hal_driver/Src/stm32h7rsxx_ll_rcc.c
    stm32h7rs_hal_driver/Src/stm32h7rsxx_ll_gpio.c
)

target_include_directories(stm32_ll PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32h7rs_hal_driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# USE_FULL_LL_DRIVER gates the LL_*_InitTypeDef structs and LL_*_Init prototypes.
#TODO: USE_FULL_ASSERT is on in Release too; gate it per build type later
target_compile_definitions(stm32_ll PUBLIC
    USE_FULL_LL_DRIVER
    USE_FULL_ASSERT
)

target_link_libraries(stm32_ll PUBLIC cmsis)
