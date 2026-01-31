#include "st7789.h"
#include <string.h>

#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_gpio.h"
#include "system.h"

/* Control macros */
#define CS_LOW(h)   HAL_GPIO_Write((h)->cs_port, (h)->cs_pin, GPIO_PIN_RESET)
#define CS_HIGH(h)  HAL_GPIO_Write((h)->cs_port, (h)->cs_pin, GPIO_PIN_SET)
#define DC_CMD(h)   HAL_GPIO_Write((h)->dc_port, (h)->dc_pin, GPIO_PIN_RESET)
#define DC_DATA(h)  HAL_GPIO_Write((h)->dc_port, (h)->dc_pin, GPIO_PIN_SET)
#define RST_LOW(h)  HAL_GPIO_Write((h)->rst_port, (h)->rst_pin, GPIO_PIN_RESET)
#define RST_HIGH(h) HAL_GPIO_Write((h)->rst_port, (h)->rst_pin, GPIO_PIN_SET)
#define BL_LOW(h)   HAL_GPIO_Write((h)->bl_port, (h)->bl_pin, GPIO_PIN_RESET)
#define BL_HIGH(h)  HAL_GPIO_Write((h)->bl_port, (h)->bl_pin, GPIO_PIN_SET)

static uint8_t        tx_buffer[1024]; /* Adjust size as needed */
static ST7789_Handle *display = NULL;

static void ST7789_SetWindow(ST7789_Handle *handle, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
 * @brief  Initialize ST7789 display
 */
void ST7789_Init(ST7789_Handle *handle)
{
    // handle->width    = ST7789_WIDTH;
    // handle->height   = ST7789_HEIGHT;
    // handle->rotation = 0;
    // handle->state    = ST7789_STATE_IDLE;
    // handle->busy     = false;
    //
    // /* Set initial pin states */
    // CS_HIGH(handle);
    // DC_DATA(handle);
    // RST_HIGH(handle);
    //
    // /* Hardware reset */
    // ST7789_Reset(handle);
    // HAL_DelayMS(100);
    //
    // /* Software reset */
    // ST7789_WriteCommand_Blocking(handle, ST7789_SWRESET);
    // HAL_DelayMS(150);
    //
    //     /* Color mode - RGB565 */
    // ST7789_WriteCommand_Blocking(handle, ST7789_COLMOD);
    // uint8_t colmod = 0x55; /* 16-bit/pixel */
    // ST7789_WriteData_Blocking(handle, &colmod, 1);
    //
    // /* Memory data access control */
    // ST7789_WriteCommand_Blocking(handle, ST7789_MADCTL);
    // uint8_t madctl = 0x00; /* Normal orientation */
    // ST7789_WriteData_Blocking(handle, &madctl, 1);
    //
    // /* Inversion ON */
    // ST7789_WriteCommand_Blocking(handle, ST7789_INVON);
    // HAL_DelayMS(100);
    //
    // /* Sleep out */
    // ST7789_WriteCommand_Blocking(handle, ST7789_SLPOUT);
    // HAL_DelayMS(100);
    //
    // // Normal display mode
    // ST7789_WriteCommand_Blocking(handle, ST7789_NORON);
    // HAL_DelayMS(10);
    //
    // uint8_t data = 0x55;
    // ST7789_WriteCommand_Blocking(handle, 0x3A);
    // ST7789_WriteData_Blocking(handle, &data, 1);
    //
    // /* Display on */
    // ST7789_WriteCommand_Blocking(handle, ST7789_DISPON);
    // HAL_DelayMS(150);
    //
    // ST7789_Toggle_Backlight(handle, GPIO_PIN_SET);
    //
    // PRINT_INFO("Attempted to initialize ST7789");
    handle->width    = ST7789_WIDTH;
    handle->height   = ST7789_HEIGHT;
    handle->rotation = 0;
    handle->state    = ST7789_STATE_IDLE;
    handle->busy     = false;
    
    CS_HIGH(handle);
    DC_DATA(handle);

    // Hardware reset
    RST_HIGH(handle);
    HAL_DelayMS(10);
    RST_LOW(handle);
    HAL_DelayMS(20);
    RST_HIGH(handle);
    HAL_DelayMS(150);

    PRINT_INFO("Starting complete 5394 init...");

    // Software reset
    CS_LOW(handle);
    DC_CMD(handle);
    uint8_t cmd = 0x01;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(150);

    // Sleep out
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0x11;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(255);

    // *** ADDITIONAL COMMANDS FOR 5394 ***

    // Frame rate control
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0xB2;  // FRMCTR2
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t frmctr[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    HAL_SPI_Transmit(handle->hspi, frmctr, 5, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // Gate control
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0xB7;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t gctrl = 0x35;
    HAL_SPI_Transmit(handle->hspi, &gctrl, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // VCOM setting
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0xBB;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t vcoms = 0x19;
    HAL_SPI_Transmit(handle->hspi, &vcoms, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // LCM control
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0xC0;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t lcm = 0x2C;
    HAL_SPI_Transmit(handle->hspi, &lcm, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // VDV/VRH enable
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0xC2;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t vdv = 0x01;
    HAL_SPI_Transmit(handle->hspi, &vdv, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // VRH set
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0xC3;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t vrh = 0x12;
    HAL_SPI_Transmit(handle->hspi, &vrh, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // VDV set
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0xC4;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t vdvset = 0x20;
    HAL_SPI_Transmit(handle->hspi, &vdvset, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // Power control 1
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0xD0;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t pwctrl[] = {0xA4, 0xA1};
    HAL_SPI_Transmit(handle->hspi, pwctrl, 2, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // *** STANDARD COMMANDS ***

    // Color mode
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0x3A;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t colmod = 0x55;
    HAL_SPI_Transmit(handle->hspi, &colmod, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // MADCTL - Try different rotation values
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0x36;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    uint8_t madctl = 0x00;  // Try 0x60, 0x70, 0xA0, 0xC0 if 0x00 doesn't work
    HAL_SPI_Transmit(handle->hspi, &madctl, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // Inversion ON
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0x21;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // Normal display
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0x13;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(10);

    // Display on
    CS_LOW(handle);
    DC_CMD(handle);
    cmd = 0x29;
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    CS_HIGH(handle);
    HAL_DelayMS(100);

    BL_HIGH(handle);

    // ST7789_Debug(handle);
    
    PRINT_INFO("ST7789 initialization complete");
}

/**
 * @brief  Hardware reset
 */
void ST7789_Reset(ST7789_Handle *handle)
{
    // RST_LOW(handle);
    // RST_HIGH(handle);

    CS_HIGH(handle);  // Ensure CS is high during reset
    
    RST_HIGH(handle);
    HAL_DelayMS(10);
    
    RST_LOW(handle);
    HAL_DelayMS(20);
    
    RST_HIGH(handle);
    HAL_DelayMS(150);
}

/**
 * @brief  Write command in blocking mode
 */
void ST7789_WriteCommand_Blocking(ST7789_Handle *handle, uint8_t cmd)
{
    CS_LOW(handle);
    DC_CMD(handle);
    HAL_Status status = HAL_SPI_Transmit(handle->hspi, &cmd, sizeof(cmd), 100);
    PRINT_INFO("TX CMD 0x%02X: %s", cmd, status == HAL_OK ? "OK" : "FAIL");
    CS_HIGH(handle);
}

/**
 * @brief  Write data in blocking mode
 */
void ST7789_WriteData_Blocking(ST7789_Handle *handle, const uint8_t *data, uint16_t len)
{
    CS_LOW(handle);
    DC_DATA(handle);
    HAL_Status status = HAL_SPI_Transmit(handle->hspi, data, len, 1000);
    PRINT_INFO("TX DATA (%d bytes): %s", len, status == HAL_OK ? "OK" : "FAIL");
    CS_HIGH(handle);
}

void ST7789_SendCommandData(ST7789_Handle *handle, uint8_t cmd, uint8_t *data, uint16_t len)
{
    CS_LOW(handle);  // CS low for entire transaction
    
    DC_CMD(handle);
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    
    if (len > 0 && data != NULL) {
        DC_DATA(handle);
        HAL_SPI_Transmit(handle->hspi, data, len, 1000);
    }
    
    CS_HIGH(handle);  // CS high only at end
    
    PRINT_INFO("TX CMD 0x%02X + %d data bytes: OK", cmd, len);
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
    HAL_DelayMS(150);
    HAL_SPI_Receive(handle->hspi, data, 4, 100);
    CS_HIGH(handle);

    /* Combine ID bytes (skip dummy byte) */
    id = (data[1] << 16) | (data[2] << 8) | data[3];
    return id;
}

void Test_ST7789_Write_Only(ST7789_Handle *handle)
{
        PRINT_INFO("Starting simple display test...");
    
    // Test 1: Single pixel
    ST7789_SetWindow(handle, 0, 0, 0, 0);
    
    DC_DATA(handle);
    uint8_t red[] = {0xF8, 0x00};  // Red pixel
    HAL_SPI_Transmit(handle->hspi, red, 2, 100);
    CS_HIGH(handle);
    
    HAL_DelayMS(500);
    PRINT_INFO("Sent 1 red pixel at (0,0)");
    
    // Test 2: Small square (10x10)
    ST7789_SetWindow(handle, 10, 10, 19, 19);
    
    DC_DATA(handle);
    uint8_t green[] = {0x07, 0xE0};  // Green pixel
    for (int i = 0; i < 100; i++) {
        HAL_SPI_Transmit(handle->hspi, green, 2, 100);
    }
    CS_HIGH(handle);
    
    HAL_DelayMS(500);
    PRINT_INFO("Sent 10x10 green square at (10,10)");
    
    // Test 3: Horizontal line
    ST7789_SetWindow(handle, 0, 50, 239, 50);
    
    DC_DATA(handle);
    uint8_t blue[] = {0x00, 0x1F};  // Blue pixel
    for (int i = 0; i < 240; i++) {
        HAL_SPI_Transmit(handle->hspi, blue, 2, 100);
    }
    CS_HIGH(handle);
    
    PRINT_INFO("Sent blue horizontal line at y=50");
    // PRINT_INFO("=== Testing GPIO Control ===");
    //
    // // Test CS
    // PRINT_INFO("CS initial: %lu", (GPIOD->ODR >> 14) & 1);
    // CS_LOW(st7789_handle);
    // PRINT_INFO("CS after LOW: %lu", (GPIOD->ODR >> 14) & 1);
    // CS_HIGH(st7789_handle);
    // PRINT_INFO("CS after HIGH: %lu", (GPIOD->ODR >> 14) & 1);
    //
    // // Test DC
    // PRINT_INFO("DC initial: %lu", (GPIOC->ODR >> 13) & 1);
    // DC_CMD(st7789_handle);
    // PRINT_INFO("DC after CMD: %lu", (GPIOC->ODR >> 13) & 1);
    // DC_DATA(st7789_handle);
    // PRINT_INFO("DC after DATA: %lu", (GPIOC->ODR >> 13) & 1);
    //
    // // Test RST
    // PRINT_INFO("RST initial: %lu", (GPIOC->ODR >> 14) & 1);
    // RST_LOW(st7789_handle);
    // PRINT_INFO("RST after LOW: %lu", (GPIOC->ODR >> 14) & 1);
    // RST_HIGH(st7789_handle);
    // PRINT_INFO("RST after HIGH: %lu", (GPIOC->ODR >> 14) & 1);
    //
    // PRINT_INFO("BL initial: %lu", (GPIOF->ODR >> 5) & 1);
    //
    // PRINT_INFO("\n\n");
    //
    // data = 0xFF;  // All 1's
    // PRINT_INFO("Before transmit:");
    // PRINT_INFO("  MOSI (PA7): %lu", (GPIOB->IDR >> 5) & 1);
    //
    // CS_LOW(st7789_handle);
    // HAL_SPI_Transmit(st7789_handle->hspi, &data, 1, 100);
    //
    // // Read MOSI immediately (might still be high from last bit)
    // PRINT_INFO("Right after transmit:");
    // PRINT_INFO("  MOSI (PA7): %lu", (GPIOB->IDR >> 5) & 1);
    //
    // CS_HIGH(st7789_handle);
    //
    // // Try with 0x00
    // data = 0x00;
    // CS_LOW(st7789_handle);
    // HAL_SPI_Transmit(st7789_handle->hspi, &data, 1, 100);
    //
    // PRINT_INFO("After transmit 0x00:");
    // PRINT_INFO("  MOSI (PA7): %lu", (GPIOB->IDR >> 5) & 1);
    //
    // CS_HIGH(st7789_handle);
    //
    // PRINT_INFO("GPIOB->MODER (PB5): 0x%08lX", GPIOB->MODER);
    // PRINT_INFO("GPIOB->AFR[0] (PB5 AF): 0x%08lX", GPIOB->AFR[0]);
}

void ST7789_Debug(ST7789_Handle *handle)
{
    uint8_t cmd = 0x09;  // RDDST - Read Display Status
    uint8_t status[5] = {0};

    DC_CMD(handle);
    CS_LOW(handle);
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    HAL_SPI_Receive(handle->hspi, status, 5, 100);
    CS_HIGH(handle);

    // Print all bytes to see what you're getting
    PRINT_INFO("Status bytes: %02X %02X %02X %02X %02X", 
           status[0], status[1], status[2], status[3], status[4]);
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
    
    // Apply offsets for 5394
    x0 += ST7789_XSTART;
    x1 += ST7789_XSTART;
    y0 += ST7789_YSTART;  // Important! Row offset = 35
    y1 += ST7789_YSTART;
    
    CS_LOW(handle);
    
    // Column address set
    DC_CMD(handle);
    uint8_t cmd = 0x2A;  // CASET
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    HAL_SPI_Transmit(handle->hspi, data, 4, 100);
    
    // Row address set
    DC_CMD(handle);
    cmd = 0x2B;  // RASET
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
    DC_DATA(handle);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    HAL_SPI_Transmit(handle->hspi, data, 4, 100);
    
    // Memory write
    DC_CMD(handle);
    cmd = 0x2C;  // RAMWR
    HAL_SPI_Transmit(handle->hspi, &cmd, 1, 100);
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
     PRINT_INFO("Filling ENTIRE 320x170 display with RED...");
    
    ST7789_SetWindow(handle, 0, 0, 319, 169);
    
    DC_DATA(handle);
    uint8_t red[2] = {0xF8, 0x00};
    
    // Fill ALL pixels (320 * 170 = 54,400 pixels)
    for (uint32_t i = 0; i < 54400; i++) {
        HAL_SPI_Transmit(handle->hspi, red, 2, 100);
        
        if (i % 10000 == 0) {
            PRINT_INFO("Sent %lu pixels...", i);
        }
    }
    
    CS_HIGH(handle);
    PRINT_INFO("Filled entire display");  
    
    // /* Set window (blocking part) */
    // ST7789_SetWindow(handle, x, y, x + w - 1, y + h - 1);
    //
    // /* Prepare color buffer */
    // uint16_t pixels_per_chunk = sizeof(tx_buffer) / 2; /* 2 bytes per pixel */
    // for (uint16_t i = 0; i < pixels_per_chunk; i++) {
    //     tx_buffer[i * 2]     = color >> 8;
    //     tx_buffer[i * 2 + 1] = color & 0xFF;
    // }
    //
    // /* Send first chunk */
    // uint32_t total_pixels = (uint32_t) w * h;
    // uint16_t chunk_size   = (total_pixels > pixels_per_chunk) ? pixels_per_chunk * 2 : total_pixels * 2;
    //
    // handle->TransferCpltCallback = callback;
    // ST7789_WriteData_IT(handle, tx_buffer, chunk_size, callback);
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

void ST7789_Toggle_Backlight(ST7789_Handle *handle, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) {
        BL_HIGH(handle);
    } else if (state == GPIO_PIN_RESET) {
        BL_LOW(handle);
    }
}

void ST7789_Toggle_ChipSelect(ST7789_Handle *handle, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) {
        CS_HIGH(handle);
    } else if (state == GPIO_PIN_RESET) {
        CS_LOW(handle);
    }
}
