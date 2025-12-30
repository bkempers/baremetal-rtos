#ifndef ST7789_DRIVER_H
#define ST7789_DRIVER_H

#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_spi.h"
#include <stdbool.h>
#include <stdint.h>

/* Display dimensions */
#define ST7789_WIDTH  170
#define ST7789_HEIGHT 320

/* Common commands */
#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04 /* Read Display ID */
#define ST7789_RDDST   0x09 /* Read Display Status */
#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E /* Read Display Memory */
#define ST7789_MADCTL  0x36
#define ST7789_COLMOD  0x3A

/* RGB565 color definitions */
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F

/* Driver states */
typedef enum {
    ST7789_STATE_IDLE = 0,
    ST7789_STATE_CMD_PENDING,
    ST7789_STATE_DATA_PENDING,
    ST7789_STATE_READ_PENDING,
    ST7789_STATE_BUSY
} ST7789_State;

/* Display handle */
typedef struct {
    /* Hardware */
    SPI_Handle   *hspi;
    GPIO_TypeDef *dc_port;
    uint16_t      dc_pin;
    GPIO_TypeDef *rst_port;
    uint16_t      rst_pin;
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;

    /* Display properties */
    uint16_t width;
    uint16_t height;
    uint8_t  rotation;

    /* State management */
    ST7789_State state;
    bool         busy;

    /* Callback for async operations */
    void (*TransferCpltCallback)(void);
} ST7789_Handle;

/* Public API */
void ST7789_Init(ST7789_Handle *handle);
void ST7789_Reset(ST7789_Handle *handle);

/* Blocking operations (simple, guaranteed to complete) */
void     ST7789_WriteCommand_Blocking(ST7789_Handle *handle, uint8_t cmd);
void     ST7789_WriteData_Blocking(ST7789_Handle *handle, const uint8_t *data, uint16_t len);
uint32_t ST7789_ReadID_Blocking(ST7789_Handle *handle);

/* Interrupt-based operations (async, uses callbacks) */
void ST7789_WriteCommand_IT(ST7789_Handle *handle, uint8_t cmd, void (*callback)(void));
void ST7789_WriteData_IT(ST7789_Handle *handle, const uint8_t *data, uint16_t len, void (*callback)(void));
void ST7789_FillRect_IT(ST7789_Handle *handle, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color, void (*callback)(void));

/* High-level drawing (uses interrupts internally) */
void ST7789_FillScreen(ST7789_Handle *handle, uint16_t color);
void ST7789_DrawPixel(ST7789_Handle *handle, uint16_t x, uint16_t y, uint16_t color);
/* State queries */
bool ST7789_IsBusy(ST7789_Handle *handle);
void ST7789_IRQHandler(ST7789_Handle *handle);

#endif /* ST7789_DRIVER_H */
