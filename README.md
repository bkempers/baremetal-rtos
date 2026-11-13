# baremetal-rtos

A basic learning project to following industry standard practices for firmware development on extensible and modular architectures. Developing a ground-up system for board archiectures, chips, drivers / hardware abstraction layer, real-time operating system, and basic applications.

## Project Architecture

Each layer may only depend on the layers below it. `drivers/` and `lib/` are cross-cutting: they sit beside the stack and define contracts rather than consume them.

```
   application/    main(), task definitions
        │
   kernel/         scheduler, context switch, tick, delays
        │
   boards/         nucleo_h7s3l8 — pin map, clock tree, board_init()
        │
   chip/           stm32h7rs — hal_* implementations, vectors, startup
        │
   arch/           arm/cortex_m — NVIC, SysTick, PendSV, FPU, critical sections
        │
   modules/        CMSIS core + device headers, ST LL drivers (vendor code)


   drivers/        hal_*.h contracts, portable drivers (led/, console/)
   lib/            ringbuffer, libc shims, git_version
```
