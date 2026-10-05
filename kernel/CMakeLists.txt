add_library(kernel STATIC
    kernel.c
    # kernel_isr.c — SysTick/PendSV handlers moved to arch/arm/cortex_m
    time.c
)

target_include_directories(kernel PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# drivers for the hal_* contracts, arch for the CPU primitives. Deliberately
# NOT chip: the kernel has no CMSIS dependency left.
target_link_libraries(kernel PUBLIC drivers arch)
