#include <string.h>

#include "display.h"
#include "drivers/display/st7789/st7789.h"
#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"
#include "system.h"

ST7789_Handle st7789_handle;
SPI_Handle    spi1_handle;

static void display_usage()
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
SHELL_COMMAND_REGISTER(display, display_handler, "Access LED display information");

SYS_Status Display_Init()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure SPI1 GPIOs */
    GPIO_Init spi1_gpio;
    spi1_gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7; // PA5=SCK, PA6=MISO, PA7=MOSI
    spi1_gpio.Mode      = GPIO_MODE_ALT_FUNC_PP;                // Alternate Function Push-Pull
    spi1_gpio.Pull      = GPIO_NOPULL_UP;                       // No pull-up/pull-down
    spi1_gpio.Speed     = GPIO_SPEED_FEQ_VERY_HIGH;             // Max speed for high SPI clock
    spi1_gpio.Alternate = ((uint8_t) 0x05);                     // AF5 = SPI1 function
    HAL_GPIO_Init(GPIOA, &spi1_gpio);

    /* Configure CS SPI */
    GPIO_Init cs_spi_gpio;
    cs_spi_gpio.Pin   = GPIO_PIN_4;
    cs_spi_gpio.Mode  = GPIO_MODE_ALT_FUNC_PP;
    cs_spi_gpio.Pull  = GPIO_NOPULL_UP;
    cs_spi_gpio.Speed = GPIO_SPEED_FREQ_MED;
    HAL_GPIO_Init(GPIOA, &cs_spi_gpio);

    /* Configure DC SPI */
    GPIO_Init dc_spi_gpio;
    dc_spi_gpio.Pin   = GPIO_PIN_13;
    dc_spi_gpio.Mode  = GPIO_MODE_ALT_FUNC_PP;
    dc_spi_gpio.Pull  = GPIO_NOPULL_UP;
    dc_spi_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &dc_spi_gpio);

    /* Configure RST SPI */
    GPIO_Init rst_spi_gpio;
    rst_spi_gpio.Pin   = GPIO_PIN_14;
    rst_spi_gpio.Mode  = GPIO_MODE_ALT_FUNC_PP;
    rst_spi_gpio.Pull  = GPIO_NOPULL_UP;
    rst_spi_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &rst_spi_gpio);

    __HAL_RCC_SPI1_CLK_ENABLE();

    spi1_handle.Instance               = SPI1;
    spi1_handle.Init.Mode              = SPI_CFG2_MASTER;
    spi1_handle.Init.Direction         = SPI_DIRECTION_2LINES;
    spi1_handle.Init.DataSize          = SPI_DATASIZE_16BIT;
    spi1_handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    spi1_handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    spi1_handle.Init.NSS               = SPI_NSS_SOFT;
    spi1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    spi1_handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    spi1_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
    spi1_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    spi1_handle.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;

    if (HAL_SPI_Init(&spi1_handle) != HAL_OK) {
        return SYS_ERROR;
    }

    __NVIC_SetPriority(SPI1_IRQn, 5);
    __NVIC_EnableIRQ(SPI1_IRQn);

    /* Configure the ST7789 Handle */
    st7789_handle.hspi     = &spi1_handle;
    st7789_handle.cs_port  = GPIOA;
    st7789_handle.cs_pin   = GPIO_PIN_4;
    st7789_handle.dc_port  = GPIOC;
    st7789_handle.dc_pin   = GPIO_PIN_13;
    st7789_handle.rst_port = GPIOC;
    st7789_handle.rst_pin  = GPIO_PIN_14;
    ST7789_Init(&st7789_handle);

    uint32_t display_id = ST7789_ReadID_Blocking(&st7789_handle);
    if (display_id == 0) {
        PRINT_INFO("Failed to initialize ST7789 display.");
        return SYS_ERROR;
    }

    PRINT_INFO("Initialized ST7789 (0x%X) display module successfully.", display_id);
    return SYS_OK;
}
