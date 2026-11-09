# baremetal-rtos

A basic learning project to following industry standard practices for firmware development on extensible and modular architectures. Developing a ground-up system for board archiectures, chips, drivers / hardware abstraction layer, real-time operating system, and basic applications.

## Project Architecture

Each layer may only depend on the layers below it. `drivers/` and `lib/` are
cross-cutting: they sit beside the stack and define contracts rather than
consume them.

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

The CMake targets mirror the diagram one-to-one:

| Target        | Depends on                                  |
| ------------- | ------------------------------------------- |
| `application` | `kernel` `boards` `chip` `drivers`          |
| `kernel`      | `drivers` `arch`                            |
| `boards`      | `drivers` `chip` `arch`                     |
| `chip`        | `drivers` `stm32_ll` `ringbuffer` `arch`    |
| `arch`        | `cmsis`                                     |
| `stm32_ll`    | `cmsis`                                     |

Two rules keep the lower layers portable:

- **`arch/` never names an SoC.** CMSIS only exposes the core intrinsics
  (`NVIC_*`, `SysTick_Config`) through a device header, so `arch/` reaches them
  via `modules/cmsis_device.h`, a one-line shim that lives beside the `cmsis`
  target — the same place the device include path and `STM32H7S3xx` are already
  selected. Retargeting a different Cortex-M SoC touches `modules/` and `chip/`
  only.
- **`kernel/` never links `chip`.** It talks to hardware through the `hal_*.h`
  contracts in `drivers/` and the CPU primitives in `arch/`, and has no CMSIS
  dependency of its own.

## Project development roadmap

- [ ] Unit Testing Framework
  - [x] Ceedling Framework {CMock, Unity, CException}
  - [ ] Github CI/CD Pipeline
  - [ ] GCoverage Test Coverage
- [ ] HAL Layer
  - [x] GPIO
  - [x] TIM
  - [x] RCC
  - [x] USART
  - [x] I2C
  - [x] SPI
  - [x] DMA
  - [ ] Ext-Mem
  - [ ] ...
- [ ] RTOS Layer
  - [ ] Task Control Block
  - [ ] System Tick
  - [ ] Context Switching
  - [ ] Scheduler
    - [ ] Round Robin
  - [ ] Kernel Primitives
    - [ ] Mutex
    - [ ] Semaphore
    - [ ] Queue
- [ ] Application Layer
  - [x] LED
  - [x] System
    - [x] Syscalls
    - [x] Sysmem
    - [x] System
  - [x] Console
    - [x] Shell Commands
    - [x] Bi-Directional Console
  - [x] BME680 Driver
  - [ ] LVGL Display Addition
