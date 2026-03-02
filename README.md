# baremetal-rtos

A technical approach to designing and implementing a custom HAL (hardware abstraction layer) as well as an RTOS (real-time operating system) on top for preemtive scheduling of tasks.

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
