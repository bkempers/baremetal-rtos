#include <string.h>

#include "display.h"
#include "st7789.h"
#include "stm32h7rs_hal_gpio.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rs_hal_spi.h"
#include "system.h"

ST7789_Handle st7789_handle;
SPI_Handle    spi1_handle;

// static void Display_Flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_buf);

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

static void SPI_Clock_Diagnostic(void)
{
    PRINT_INFO("=== SPI1 Clock & Config Diagnostic ===");
    
    // 1. Check if SPI1 clock is enabled
    if (RCC->APB2ENR & RCC_APB2ENR_SPI1EN) {
        PRINT_INFO("✓ SPI1 clock enabled in RCC");
    } else {
        PRINT_INFO("✗ SPI1 clock NOT enabled in RCC!");
        return;
    }
    
    // 2. Check SPI1 clock source (CCIPR3 bits 8-10)
    uint32_t spi1_clk_src = (RCC->CCIPR3 >> 8) & 0x7;
    PRINT_INFO("SPI1 clock source: 0x%lX", spi1_clk_src);
    switch(spi1_clk_src) {
        case 0: PRINT_INFO("  -> PLL1Q"); break;
        case 1: PRINT_INFO("  -> PLL2P"); break;
        case 2: PRINT_INFO("  -> PLL3P"); break;
        case 3: PRINT_INFO("  -> I2S_CKIN"); break;
        case 4: PRINT_INFO("  -> PER_CK (HSI or CSI)"); break;
        default: PRINT_INFO("  -> Reserved/Invalid"); break;
    }
    
    // 3. Check SPI configuration registers
    PRINT_INFO("SPI1->CR1:  0x%08lX", SPI1->CR1);
    PRINT_INFO("  SPE (bit 0):     %ld", (SPI1->CR1 >> 0) & 1);
    PRINT_INFO("  CSTART (bit 9):  %ld", (SPI1->CR1 >> 9) & 1);
    PRINT_INFO("  CSUSP (bit 10):  %ld", (SPI1->CR1 >> 10) & 1);
    
    PRINT_INFO("SPI1->CR2:  0x%08lX", SPI1->CR2);
    PRINT_INFO("  TSIZE: %ld", SPI1->CR2 & 0xFFFF);
    
    PRINT_INFO("SPI1->CFG1: 0x%08lX", SPI1->CFG1);
    uint32_t baudrate = (SPI1->CFG1 >> 28) & 0x7;
    PRINT_INFO("  Baudrate prescaler: %ld (divide by %ld)", baudrate, 2 << baudrate);
    
    PRINT_INFO("SPI1->CFG2: 0x%08lX", SPI1->CFG2);
    PRINT_INFO("  MASTER: %ld", (SPI1->CFG2 >> 22) & 1);
    PRINT_INFO("  COMM: 0x%lX", (SPI1->CFG2 >> 17) & 0x3);
    
    PRINT_INFO("SPI1->SR:   0x%08lX", SPI1->SR);
    PRINT_INFO("  TXP (TX ready):  %ld", (SPI1->SR >> 1) & 1);
    PRINT_INFO("  RXP (RX ready):  %ld", (SPI1->SR >> 0) & 1);
    PRINT_INFO("  EOT (End of TX): %ld", (SPI1->SR >> 3) & 1);
    PRINT_INFO("  BSY (Busy):      %ld", (SPI1->SR >> 7) & 1);
    
    // 4. Check GPIO configuration for SPI pins
    PRINT_INFO("\n=== GPIO Configuration ===");
    
    // PA5 (SCK), PA6 (MISO)
    uint32_t pa5_mode = (GPIOA->MODER >> (5*2)) & 0x3;
    uint32_t pa5_af = (GPIOA->AFR[0] >> (5*4)) & 0xF;
    uint32_t pa6_mode = (GPIOA->MODER >> (6*2)) & 0x3;
    uint32_t pa6_af = (GPIOA->AFR[0] >> (6*4)) & 0xF;
    
    PRINT_INFO("PA5 (SCK):  Mode=%ld %s, AF=%ld", 
               pa5_mode, 
               pa5_mode == 2 ? "ALT FUNC" : "WRONG!",
               pa5_af);
    PRINT_INFO("PA6 (MISO): Mode=%ld %s, AF=%ld", 
               pa6_mode,
               pa6_mode == 2 ? "ALT FUNC" : "WRONG!",
               pa6_af);
    
    // PB5 (MOSI)
    uint32_t pb5_mode = (GPIOB->MODER >> (5*2)) & 0x3;
    uint32_t pb5_af = (GPIOB->AFR[0] >> (5*4)) & 0xF;
    PRINT_INFO("PB5 (MOSI): Mode=%ld %s, AF=%ld", 
               pb5_mode,
               pb5_mode == 2 ? "ALT FUNC" : "WRONG!",
               pb5_af);
    
    // Check control pins
    uint32_t pd14_mode = (GPIOD->MODER >> (14*2)) & 0x3;
    uint32_t pc13_mode = (GPIOC->MODER >> (13*2)) & 0x3;
    uint32_t pc14_mode = (GPIOC->MODER >> (14*2)) & 0x3;
    
    PRINT_INFO("PD14 (CS):  Mode=%ld %s", pd14_mode, pd14_mode == 1 ? "OUTPUT" : "WRONG!");
    PRINT_INFO("PC13 (RST): Mode=%ld %s", pc13_mode, pc13_mode == 1 ? "OUTPUT" : "WRONG!");
    PRINT_INFO("PC14 (DC):  Mode=%ld %s", pc14_mode, pc14_mode == 1 ? "OUTPUT" : "WRONG!");
    
    // 5. Test actual SPI transmission
    PRINT_INFO("\n=== Live SPI Test ===");
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);  // Enable SPI first!

    uint8_t test_byte = 0xAA;
    
    PRINT_INFO("Before TX - SR: 0x%08lX", SPI1->SR);
    
    // Manual transmit to see what happens
    MODIFY_REG(SPI1->CR2, SPI_CR2_TSIZE, 1);
    SET_BIT(SPI1->CR1, SPI_CR1_CSTART);
    
    uint32_t timeout = 10000;
    while (!(SPI1->SR & SPI_SR_TXP) && timeout--);
    
    if (SPI1->SR & SPI_SR_TXP) {
        PRINT_INFO("✓ TXP flag set - writing 0xAA");
        *((__IO uint8_t *)&SPI1->TXDR) = test_byte;
        
        // Wait a bit
        HAL_DelayMS(1);
        PRINT_INFO("After TX - SR: 0x%08lX", SPI1->SR);
        
        // Wait for EOT
        timeout = 10000;
        while (!(SPI1->SR & SPI_SR_EOT) && timeout--);
        
        if (SPI1->SR & SPI_SR_EOT) {
            PRINT_INFO("✓ EOT flag set - transmission complete");
        } else {
            PRINT_INFO("✗ EOT flag never set - transmission hung!");
        }
    } else {
        PRINT_INFO("✗ TXP flag never set - SPI not ready!");
    }
}

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
    HAL_GPIO_Write(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

    /* Configure SD CS SPI */
    GPIO_Init sdcs_spi_gpio;
    sdcs_spi_gpio.Pin   = GPIO_PIN_4;
    sdcs_spi_gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    sdcs_spi_gpio.Pull  = GPIO_NOPULL_UP;
    sdcs_spi_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &sdcs_spi_gpio);
    HAL_GPIO_Write(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);

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
    spi1_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
    //
    if (HAL_SPI_Init(&spi1_handle) != HAL_OK) {
        return SYS_ERROR;
    }

    ST7789_Debug(&st7789_handle);
    SPI_Clock_Diagnostic();

    // CRITICAL: Explicitly enable SPI peripheral
    PRINT_INFO("SPI1 enabled? %u", (SPI1->CR1 & SPI_CR1_SPE));
    SET_BIT(SPI1->CR1, SPI_CR1_SPE);
    PRINT_INFO("SPI1 enabled %u", (SPI1->CR1 & SPI_CR1_SPE));

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

    uint32_t display_id = ST7789_ReadID_Blocking(&st7789_handle);
    if (display_id == 0) {
        PRINT_INFO("Failed to initialize ST7789 display.");
        return SYS_ERROR;
    }

    ST7789_FillScreen(&st7789_handle, COLOR_RED);

    PRINT_INFO("Initialized ST7789 (0x%lX) display module successfully.", display_id);
    return SYS_OK;
}

// void LVGL_Display_Init(void) {
//     lv_init();
//
//     lv_tick_set_cb(HAL_GetTick);
//
//     lv_display_t * display = lv_display_create(320, 240);
//
//     /* LVGL will render to this 1/10 screen sized buffer for 2 bytes/pixel */
//     static uint8_t buf[320 * 240 / 10 * 2];
//     lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
//
//     /* This callback will display the rendered image */
//     lv_display_set_flush_cb(display, Display_Flush);
//
//     /* Create widgets */
//     lv_obj_t * label = lv_label_create(lv_screen_active());
//     lv_label_set_text(label, "Hello LVGL!");
// }
//
// static void Display_Flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_buf) {
//     UNUSED(area);
//     UNUSED(px_buf);
//     /* Show the rendered image on the display */
//     //my_display_update(area, px_buf);
//
//     /* Indicate that the buffer is available.
//      * If DMA were used, call in the DMA complete interrupt. */
//     lv_display_flush_ready(disp);
// }
//
// void LVGL_Display_Task(void) {
//     lv_timer_handler();
// }
