# baremetal-rtos

A technical approach to designing and implementing a custom HAL (hardware abstraction layer) as well as an RTOS (real-time operating system) on top for preemtive scheduling of tasks.

## Architecture: Two-Stage Boot with External Flash

This project uses a two-stage boot architecture to overcome the STM32H7S3L8's
64KB internal flash limitation:

**Boot Stage (32KB internal flash @ 0x08000000):**

- Initializes system clocks (200-600 MHz)
- Configures XSPI2 interface for external flash
- Enables memory-mapped mode for 256MB Macronix OctoSPI flash
- Jumps to application at 0x70000000

**Application Stage (256MB external flash @ 0x70000000):**

- Full RTOS with peripheral drivers
- Graphics (LVGL), sensors (BME680), console
- Executes in-place (XIP) from external memory
