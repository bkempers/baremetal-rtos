#include <string.h>

#include "display.h"
#include "st7789.h"
#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_spi.h"
#include "system.h"

ST7789_Handle st7789_handle;
SPI_Handle    spi1_handle;

static void Display_Flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_buf);

static void display_usage(void)
{
    PRINT_INFO("Display module usage: \r\n \
        - info: Get information regarding the display \r\n");
}

static int display_handler(int argc, char **argv)
{
    if (argc < 2) {
        display_usage();
        return 1;
    }

    if (strcmp(argv[1], "info") == 0) {
        PRINT_INFO("display info");
        return 0;
    }

    if (strlen(*argv) > 2) {
        PRINT_INFO("Unknown system argument %s", argv[1]);
        return 1;
    }

    return 0;
}
SHELL_COMMAND_REGISTER(display, display_handler, "Access LED display information")

SYS_Status Display_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* Configure SPI1 GPIOs */
    GPIO_Init spi1_gpio;
    spi1_gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_6;              // PA5=SCK, PA6=MISO
    spi1_gpio.Mode      = GPIO_MODE_ALT_FUNC_PP;                // Alternate Function Push-Pull
    spi1_gpio.Pull      = GPIO_NOPULL_UP;                       // No pull-up/pull-down
    spi1_gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;            // Max speed for high SPI clock
    spi1_gpio.Alternate = ((uint8_t) 0x05);                     // AF5 = SPI1 function
    HAL_GPIO_Init(GPIOA, &spi1_gpio);

    GPIO_Init spi1_mosi_gpio;
    spi1_mosi_gpio.Pin       = GPIO_PIN_5;                           // PB5=MOSI
    spi1_mosi_gpio.Mode      = GPIO_MODE_ALT_FUNC_PP;                // Alternate Function Push-Pull
    spi1_mosi_gpio.Pull      = GPIO_NOPULL_UP;                       // No pull-up/pull-down
    spi1_mosi_gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;            // Max speed for high SPI clock
    spi1_mosi_gpio.Alternate = ((uint8_t) 0x05);                     // AF5 = SPI1 function
    HAL_GPIO_Init(GPIOB, &spi1_mosi_gpio);

    /* Configure CS SPI */
    GPIO_Init cs_spi_gpio;
    cs_spi_gpio.Pin   = GPIO_PIN_14;
    cs_spi_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    cs_spi_gpio.Pull  = GPIO_NOPULL_UP;
    cs_spi_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &cs_spi_gpio);

    /* Configure BL SPI */
    GPIO_Init bl_spi_gpio;
    bl_spi_gpio.Pin = GPIO_PIN_5;
    bl_spi_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    bl_spi_gpio.Pull = GPIO_PULL_DOWN;
    bl_spi_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &bl_spi_gpio);

    /* Configure RST SPI */
    GPIO_Init rst_spi_gpio;
    rst_spi_gpio.Pin   = GPIO_PIN_13;
    rst_spi_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    rst_spi_gpio.Pull  = GPIO_NOPULL_UP;
    rst_spi_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &rst_spi_gpio);

    /* Configure DC SPI */
    GPIO_Init dc_spi_gpio;
    dc_spi_gpio.Pin   = GPIO_PIN_14;
    dc_spi_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    dc_spi_gpio.Pull  = GPIO_NOPULL_UP;
    dc_spi_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &dc_spi_gpio);

    __HAL_RCC_SPI1_CLK_ENABLE();

    MODIFY_REG(RCC->CCIPR3, (0x7 << 8), (0x4 << 8));

    spi1_handle.Instance               = SPI1;
    spi1_handle.Init.Mode              = SPI_CFG2_MASTER;
    spi1_handle.Init.Direction         = SPI_DIRECTION_2LINES;
    spi1_handle.Init.DataSize          = SPI_DATASIZE_8BIT;
    spi1_handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    spi1_handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    spi1_handle.Init.NSS               = SPI_NSS_SOFT;
    spi1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    spi1_handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    spi1_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
    spi1_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    spi1_handle.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    spi1_handle.Init.CRCPolynomial     = 0x0;
    // spi1_handle.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    // spi1_handle.Init.NSSPMode          = 0x0;
    // spi1_handle.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    // spi1_handle.Init.TxCRCInitializationPattern   = 0x00000000;
    // spi1_handle.Init.RxCRCInitializationPattern   = 0x00000000;
    // spi1_handle.Init.MasterSSIdleness             = 0x0;
    // spi1_handle.Init.MasterInterDataIdleness      = 0x0;
    // spi1_handle.Init.MasterReceiverAutoSusp       = 0x0;
    // spi1_handle.Init.MasterKeepIOState            = 0x0;
    // spi1_handle.Init.IOSwap                       = 0x0;
    // spi1_handle.Init.ReadyMasterManagement        = 0x0;
    // spi1_handle.Init.ReadyPolarity                = 0x0;
    if (HAL_SPI_Init(&spi1_handle) != HAL_OK) {
        return SYS_ERROR;
    }

    // CRITICAL: Explicitly enable SPI peripheral
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);

    __NVIC_SetPriority(SPI1_IRQn, 5);
    __NVIC_EnableIRQ(SPI1_IRQn);
    
    /* Configure the ST7789 Handle */
    st7789_handle.hspi     = &spi1_handle;
    st7789_handle.cs_port  = GPIOD;
    st7789_handle.cs_pin   = GPIO_PIN_14;
    st7789_handle.rst_port = GPIOC;
    st7789_handle.rst_pin  = GPIO_PIN_13;
    st7789_handle.dc_port  = GPIOC;
    st7789_handle.dc_pin   = GPIO_PIN_14;
    st7789_handle.bl_port  = GPIOF;
    st7789_handle.bl_pin   = GPIO_PIN_5;
    ST7789_Init(&st7789_handle);

    ST7789_FillScreen(&st7789_handle, COLOR_WHITE);

    uint32_t display_id = ST7789_ReadID_Blocking(&st7789_handle);
    if (display_id == 0) {
        PRINT_INFO("Failed to initialize ST7789 display.");
        return SYS_ERROR;
    }

    PRINT_INFO("Initialized ST7789 (0x%lX) display module successfully.", display_id);
    return SYS_OK;
}

void LVGL_Display_Init(void) {
    lv_init();

    lv_tick_set_cb(HAL_GetTick);

    lv_display_t * display = lv_display_create(320, 240);

    /* LVGL will render to this 1/10 screen sized buffer for 2 bytes/pixel */
    static uint8_t buf[320 * 240 / 10 * 2];
    lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* This callback will display the rendered image */
    lv_display_set_flush_cb(display, Display_Flush);

    /* Create widgets */
    lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello LVGL!");
}

static void Display_Flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_buf) {
    UNUSED(area);
    UNUSED(px_buf);
    /* Show the rendered image on the display */
    //my_display_update(area, px_buf);

    /* Indicate that the buffer is available.
     * If DMA were used, call in the DMA complete interrupt. */
    lv_display_flush_ready(disp);
}

void LVGL_Display_Task(void) {
    lv_timer_handler();
}
