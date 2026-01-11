#include "st7789.h"
#include <string.h>

#include "stm32h7rs_hal.h"
#include "system.h"

/* Control macros */
#define CS_LOW(h)   HAL_GPIO_Write((h)->cs_port, (h)->cs_pin, GPIO_PIN_RESET)
#define CS_HIGH(h)  HAL_GPIO_Write((h)->cs_port, (h)->cs_pin, GPIO_PIN_SET)
#define DC_CMD(h)   HAL_GPIO_Write((h)->dc_port, (h)->dc_pin, GPIO_PIN_RESET)
#define DC_DATA(h)  HAL_GPIO_Write((h)->dc_port, (h)->dc_pin, GPIO_PIN_SET)
#define RST_LOW(h)  HAL_GPIO_Write((h)->rst_port, (h)->rst_pin, GPIO_PIN_RESET)
#define RST_HIGH(h) HAL_GPIO_Write((h)->rst_port, (h)->rst_pin, GPIO_PIN_SET)

static uint8_t        tx_buffer[1024]; /* Adjust size as needed */
static ST7789_Handle *display = NULL;

static void ST7789_SetWindow(ST7789_Handle *handle, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
 * @brief  Initialize ST7789 display
 */
void ST7789_Init(ST7789_Handle *handle)
{
    handle->width    = ST7789_WIDTH;
    handle->height   = ST7789_HEIGHT;
    handle->rotation = 0;
    handle->state    = ST7789_STATE_IDLE;
    handle->busy     = false;

    /* Set initial pin states */
    CS_HIGH(handle);
    DC_DATA(handle);
    RST_HIGH(handle);

    /* Hardware reset */
    ST7789_Reset(handle);
    HAL_DelayMS(100);

    /* Software reset */
    ST7789_WriteCommand_Blocking(handle, ST7789_SWRESET);
    HAL_DelayMS(150);

    /* Sleep out */
    ST7789_WriteCommand_Blocking(handle, ST7789_SLPOUT);
    HAL_DelayMS(100);

    /* Color mode - RGB565 */
    ST7789_WriteCommand_Blocking(handle, ST7789_COLMOD);
    uint8_t colmod = 0x55; /* 16-bit/pixel */
    ST7789_WriteData_Blocking(handle, &colmod, 1);

    /* Memory data access control */
    ST7789_WriteCommand_Blocking(handle, ST7789_MADCTL);
    uint8_t madctl = 0x00; /* Normal orientation */
    ST7789_WriteData_Blocking(handle, &madctl, 1);

    /* Display on */
    ST7789_WriteCommand_Blocking(handle, ST7789_DISPON);
    HAL_DelayMS(150);
}

/**
 * @brief  Hardware reset
 */
void ST7789_Reset(ST7789_Handle *handle)
{
    RST_LOW(handle);
    RST_HIGH(handle);
}

/**
 * @brief  Write command in blocking mode
 */
void ST7789_WriteCommand_Blocking(ST7789_Handle *handle, uint8_t cmd)
{
    DC_CMD(handle);
    CS_LOW(handle);
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    CS_HIGH(handle);
}

/**
 * @brief  Write data in blocking mode
 */
void ST7789_WriteData_Blocking(ST7789_Handle *handle, const uint8_t *data, uint16_t len)
{
    DC_DATA(handle);
    CS_LOW(handle);
    HAL_SPI_Transmit(handle->hspi, data, len, 1000);
    CS_HIGH(handle);
}

/**
 * @brief  Read display ID (demonstrates read capability)
 * @return 24-bit ID value
 */
uint32_t ST7789_ReadID_Blocking(ST7789_Handle *handle)
{
    uint8_t  cmd     = ST7789_RDDID;
    uint8_t  data[4] = {0}; /* Dummy byte + 3 ID bytes */
    uint32_t id      = 0;

    DC_CMD(handle);
    CS_LOW(handle);
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);

    /* Switch to data mode for reading */
    DC_DATA(handle);
    HAL_SPI_Receive(handle->hspi, data, 4, 100);
    CS_HIGH(handle);

    /* Combine ID bytes (skip dummy byte) */
    id = (data[1] << 16) | (data[2] << 8) | data[3];
    return id;
}

/**
 * @brief  Write command with interrupt-based transfer
 */
void ST7789_WriteCommand_IT(ST7789_Handle *handle, uint8_t cmd, void (*callback)(void))
{
    if (handle->busy)
        return;

    handle->busy                 = true;
    handle->state                = ST7789_STATE_CMD_PENDING;
    handle->TransferCpltCallback = callback;
    display                      = handle;

    DC_CMD(handle);
    CS_LOW(handle);

    /* Store command in buffer (SPI_Transmit_IT might not accept stack variables) */
    tx_buffer[0] = cmd;
    HAL_SPI_Transmit_IT(handle->hspi, tx_buffer, 1);
}

/**
 * @brief  Write data with interrupt-based transfer
 */
void ST7789_WriteData_IT(ST7789_Handle *handle, const uint8_t *data, uint16_t len, void (*callback)(void))
{
    if (handle->busy)
        return;

    handle->busy                 = true;
    handle->state                = ST7789_STATE_DATA_PENDING;
    handle->TransferCpltCallback = callback;
    display                      = handle;

    DC_DATA(handle);
    CS_LOW(handle);

    /* Copy to buffer if not already in safe memory */
    if (len <= sizeof(tx_buffer)) {
        memcpy(tx_buffer, data, len);
        HAL_SPI_Transmit_IT(handle->hspi, tx_buffer, len);
    } else {
        /* Data too large for buffer - either:
         * 1. Pass pointer directly if you know it's safe
         * 2. Split into chunks
         * 3. Use DMA instead
         */
        HAL_SPI_Transmit_IT(handle->hspi, data, len);
    }
}

/**
 * @brief  Set drawing window (blocking helper)
 */
static void ST7789_SetWindow(ST7789_Handle *handle, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    /* Column address set */
    ST7789_WriteCommand_Blocking(handle, ST7789_CASET);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    ST7789_WriteData_Blocking(handle, data, 4);

    /* Row address set */
    ST7789_WriteCommand_Blocking(handle, ST7789_RASET);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    ST7789_WriteData_Blocking(handle, data, 4);

    /* Memory write */
    ST7789_WriteCommand_Blocking(handle, ST7789_RAMWR);
}

/**
 * @brief  Fill rectangle with color (interrupt-based, chunked)
 */
void ST7789_FillRect_IT(ST7789_Handle *handle, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color, void (*callback)(void))
{
    /* Set window (blocking part) */
    ST7789_SetWindow(handle, x, y, x + w - 1, y + h - 1);

    /* Prepare color buffer */
    uint16_t pixels_per_chunk = sizeof(tx_buffer) / 2; /* 2 bytes per pixel */
    for (uint16_t i = 0; i < pixels_per_chunk; i++) {
        tx_buffer[i * 2]     = color >> 8;
        tx_buffer[i * 2 + 1] = color & 0xFF;
    }

    /* Send first chunk */
    uint32_t total_pixels = (uint32_t) w * h;
    uint16_t chunk_size   = (total_pixels > pixels_per_chunk) ? pixels_per_chunk * 2 : total_pixels * 2;

    handle->TransferCpltCallback = callback;
    ST7789_WriteData_IT(handle, tx_buffer, chunk_size, callback);

    /* Note: For large fills, you'd need to chain multiple transfers
     * This is a simplified version - full implementation would track progress */
}

/**
 * @brief  Fill entire screen (uses interrupts)
 */
void ST7789_FillScreen(ST7789_Handle *handle, uint16_t color)
{
    ST7789_FillRect_IT(handle, 0, 0, handle->width, handle->height, color, NULL);
}

/**
 * @brief  Check if display is busy
 */
bool ST7789_IsBusy(ST7789_Handle *handle)
{
    return handle->busy;
}

/**
 * @brief  SPI transfer complete callback (called from HAL_SPI_TxCpltCallback)
 */
void ST7789_IRQHandler(ST7789_Handle *handle)
{
    CS_HIGH(handle);
    handle->busy  = false;
    handle->state = ST7789_STATE_IDLE;

    /* Call user callback if set */
    if (handle->TransferCpltCallback != NULL) {
        handle->TransferCpltCallback();
        handle->TransferCpltCallback = NULL;
    }
}
