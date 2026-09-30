# baremetal-rtos

A basic learning project to following industry standard practices for firmware development on extensible and modular architectures. Developing a ground-up system for board archiectures, chips, drivers / hardware abstraction layer, real-time operating system, and basic applications.

## Project Architecture

```

==================
SOC
 |
BOARDS
 |
DRIVERS / HAL
 |
KERNEL (RTOS)
 |
APPLICATION
====================

```

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
