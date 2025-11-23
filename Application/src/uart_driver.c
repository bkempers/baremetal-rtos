/*
 * uart_driver.c
 *
 *  Created on: Aug 15, 2025
 *      Author: benkempers
 */

#include "uart_driver.h"

/* UART DRIVER */

/*
 * USART GPDMA
 */
void gpdma1_usart3_init(uint32_t src, uint32_t dst, uint32_t len)
{
    /* enable clock access to gpdma1 */
    RCC->AHB1ENR |= GPDMA1EN;

    /* disable gpdma1 channel 0 */
    GPDMA1_Channel0->CCR &= ~GPDMA1_CCR_EN;

    /* wait until gpdma1 channel 0 disabled */
    while (GPDMA1_Channel0->CCR & GPDMA1_CCR_EN) {
    }

    /* clear all interrupt flags */
    GPDMA1_Channel0->CFCR |= GPDMA_CFCR_ALL_FLAGS;

    /* set destination buffer */
    GPDMA1_Channel0->CDAR = dst;

    /* set source buffer */
    GPDMA1_Channel0->CSAR = src;

    /* set length */
    GPDMA1_Channel0->CBR1 = (GPDMA1_Channel0->CBR1 & ~0xFFFF) | (len & 0xFFFF);

    /* Enable memory increment (source increment) */
    GPDMA1_Channel0->CTR1 = 0;                  // Clear all first
    GPDMA1_Channel0->CTR1 |= GPDMA1_CTR1_SINC;  // Increment source address
    GPDMA1_Channel0->CTR1 &= ~GPDMA1_CRT1_DINC; // Don't increment destination

    //    /* Set data width to 8-bit for both source and destination */
    //    GPDMA1_Channel0->CTR1 &= ~(GPDMA1_CTR1_SDW_LOG2_BYTE |
    //    GPDMA1_CTR1_DDW_LOG2_BYTE);

    /* Configure transfer direction (destination request) */
    GPDMA1_Channel0->CTR2 |= GPDMA1_CTR2_DREQ; // Destination request (peripheral triggers)

    /* Select channel and set USART3_TX request (replaces stream/channel
     * selection) */
    GPDMA1_Channel0->CTR2 &= ~(0x7F << 0); // Clear REQSEL
    GPDMA1_Channel0->CTR2 |= (78 << 0);    // USART3_TX request ID

    /* enable DMA transfer complete interrupt */
    GPDMA1_Channel0->CCR = 0; // Clear all first
    //    GPDMA1_Channel0->CCR |= GPDMA1_CCR_TCIE | GPDMA1_CCR_DTEIE |
    //    GPDMA1_CCR_USEIE;
    GPDMA1_Channel0->CCR |= GPDMA1_CCR_TCIE;

    /* Enable USART3 transmit DMA */
    USART3->CR1 |= USART_CR1_UE;
    USART3->CR3 |= USART_CR3_DMAT;

    /* Enable USART interrupt within NVIC IRQ */
    __NVIC_SetPriority(GPDMA1_Channel0_IRQn, 3); // Set reasonable priority
    __NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

    /* Clean cache if using data cache */
#ifdef __DCACHE_PRESENT
    SCB_CleanDCache_by_Addr((uint32_t *) src, len);
#endif

    /* Enable GPDMA1 channel 0 */
    GPDMA1_Channel0->CCR |= GPDMA1_CCR_EN;
}

void GPDMA1_CH0_IRQHandler(void)
{
    uint32_t status = GPDMA1_Channel0->CSR;

    if (status & GPDMA1_CSR_TCF) {
        GPDMA1_Channel0->CFCR |= GPDMA1_CFCR_TCF;
        led_toggle(1);
    }

    if (status & GPDMA1_CCR_DTEIE || status & GPDMA1_CCR_USEIE) {
        GPDMA1_Channel0->CFCR |= GPDMA_CFCR_DTEF0;
        GPDMA1_Channel0->CFCR |= GPDMA_CFCR_USEF0;

        led_toggle(3);
    }

    // Clear all other possible flags
    GPDMA1_Channel0->CFCR |= (0x7F << 8);
}

static void debug_dma_status(void)
{
    // Check if DMA is enabled
    if (GPDMA1_Channel0->CCR & GPDMA1_CCR_EN) {
        printf("gpdma enabled\n\r");
    }
    // Check current transfer count (should decrease as transfer progresses)
    uint32_t remaining = GPDMA1_Channel0->CBR1 & 0xFFFF;
    if (remaining != 31) {
        printf("gpdma transfer progressing\n\r");
    }
    // Check status flags
    uint32_t status = GPDMA1_Channel0->CSR;
    if (status & (1U << 8))
        printf("gpdma TCF\n\r"); // TCF - Transfer Complete
    if (status & (1U << 9))
        printf("gpdma HTF\n\r"); // HTF - Half Transfer
    if (status & (1U << 10))
        printf("gpdma DTEF\n\r"); // DTEF - Data Transfer Error
    if (status & (1U << 11))
        printf("gpdma ULEF\n\r"); // ULEF - Update Link Error
    if (status & (1U << 12))
        printf("gpdma USEF\n\r"); // USEF - User Setting Error
    if (status & (1U << 13))
        printf("gpdma SUSPF\n\r"); // SUSPF - Suspend
}

static void dump_dma_config()
{
    printf("=== DMA Configuration Dump ===\n\r");
    printf("CCR:  0x%08lX\n\r", GPDMA1_Channel0->CCR);
    printf("CTR1: 0x%08lX\n\r", GPDMA1_Channel0->CTR1);
    printf("CTR2: 0x%08lX\n\r", GPDMA1_Channel0->CTR2);
    printf("CBR1: 0x%08lX\n\r", GPDMA1_Channel0->CBR1);
    printf("CSAR: 0x%08lX\n\r", GPDMA1_Channel0->CSAR);
    printf("CDAR: 0x%08lX\n\r", GPDMA1_Channel0->CDAR);
    printf("CSR:  0x%08lX\n\r", GPDMA1_Channel0->CSR);

    // Decode CTR2 fields
    uint32_t reqsel = GPDMA1_Channel0->CTR2 & 0x7F;
    uint32_t swreq  = (GPDMA1_Channel0->CTR2 >> 9) & 1;
    uint32_t dreq   = (GPDMA1_Channel0->CTR2 >> 9) & 1;
    uint32_t sinc   = (GPDMA1_Channel0->CTR2 >> 3) & 1;
    uint32_t dinc   = (GPDMA1_Channel0->CTR2 >> 19) & 1;

    printf("REQSEL=%lu, SWREQ=%lu, DREQ=%lu, SINC=%lu, DINC=%lu\n\r", reqsel, swreq, dreq, sinc, dinc);
}

void usart_dma_driver()
{
    led_init();
    usart_rxtx_init();

    static __attribute__((aligned(32))) char message[32] = "Hello from STM32 DMA transfer\n\r";
    printf("Message address: 0x%08lX\n\r", (uint32_t) message);

    gpdma1_usart3_init((uint32_t) message, (uint32_t) &USART3->TDR, 31);

    led_toggle(2);
    printf("DMA transfer initiated. Watching for completion...\n\r");

    while (1) {
    }
}

/*
 * UART INTERRUPT
 */

void usart_rxtx_interrupt_init(void)
{
    /* configure GPIO pin */
    // enable clock access
    RCC->AHB4ENR |= GPIODEN;

    /* TX Enable for USART3 GPIO */
    // set PD8 to alternate function
    GPIOD->MODER &= ~(1U << 16); // set bit 16 to 0
    GPIOD->MODER |= (1U << 17);  // set bit 17 to 1

    // set PD8 to alternate function type UART_TX AF7 (0111)
    GPIOD->AFR[1] &= ~(0xF << 0); // Clear bits first
    GPIOD->AFR[1] |= (7U << 0);   // Set AF7

    /* RX Enable for USART3 GPIO */
    // set PD9 to alternate function
    GPIOD->MODER &= ~(1U << 18); // set bit 18 to 0
    GPIOD->MODER |= (1U << 19);  // set bit 19 to 1

    // set PD9 to alternate function type UART_RX AF7 (0111)
    GPIOD->AFR[1] &= ~(0xF << 4); // Clear bits first
    GPIOD->AFR[1] |= (7U << 4);   // Set AF7

    /* configure UART module */
    // enable clock access to UART3
    RCC->APB1ENR1 |= USART3EN;

    // configure baudrate
    uart_set_baudrate(USART3, ABP1_CLK, UART_BAUDRATE);

    // configure transfer direction
    USART3->CR1 = CR1_FIFOEN_DISABLE | CR1_TE | CR1_RE;

    /* enable RXNE interrupt */
    USART3->CR1 |= CR1_RXNEIE;

    /* ENABLE USART3 interrupt in NVIC */
    __NVIC_EnableIRQ(USART3_IRQn);

    // enable uart module
    USART3->CR1 |= CR1_UE;

    for (int i = 0; i < 1000000; i++) {
    }
}

static void uart_callback(void)
{
    char key = USART3->RDR;

    if (key == '1') {
        led_toggle(1);
    } else {
        led_toggle(2);
    }
}

void USART3_IRQHandler(void)
{
    if (USART3->ISR & ISR_ALT_RXNE) {
        uart_callback();
    }
}

void uart_interrupt_driver()
{
    led_init();
    usart_rxtx_interrupt_init();

    while (1) {
    }
}

/*
 * UART NORMAL
 */

void usart_rxtx_init(void)
{
    /* configure GPIO pin */
    // enable clock access
    RCC->AHB4ENR |= GPIODEN;

    /* TX Enable for USART3 GPIO */
    // set PD8 to alternate function
    GPIOD->MODER &= ~(1U << 16); // set bit 16 to 0
    GPIOD->MODER |= (1U << 17);  // set bit 17 to 1

    // set PD8 to alternate function type UART_TX AF7 (0111)
    GPIOD->AFR[1] &= ~(0xF << 0); // Clear bits first
    GPIOD->AFR[1] |= (7U << 0);   // Set AF7

    /* RX Enable for USART3 GPIO */
    // set PD9 to alternate function
    GPIOD->MODER &= ~(1U << 18); // set bit 18 to 0
    GPIOD->MODER |= (1U << 19);  // set bit 19 to 1

    // set PD9 to alternate function type UART_RX AF7 (0111)
    GPIOD->AFR[1] &= ~(0xF << 4); // Clear bits first
    GPIOD->AFR[1] |= (7U << 4);   // Set AF7

    /* configure UART module */
    // enable clock access to UART3
    RCC->APB1ENR1 |= USART3EN;

    // configure baudrate
    uart_set_baudrate(USART3, ABP1_CLK, UART_BAUDRATE);

    // configure transfer direction
    USART3->CR1 = CR1_FIFOEN_DISABLE | CR1_TE | CR1_RE;

    // enable uart module
    USART3->CR1 |= CR1_UE;

    for (int i = 0; i < 1000000; i++) {
    }
}

char uart3_read(void)
{
    // make sure rx data reg is not empty
    while (!(USART3->ISR & ISR_ALT_RXNE)) {
    }

    // read data
    return USART3->RDR;
}

void uart3_write(int ch)
{
    // make sure tx data reg is empty
    while (!(USART3->ISR & ISR_ALT_TXE)) {
    }
    // write to transmit data reg
    USART3->TDR = (ch & 0xFF);

    // Delay between characters
    for (volatile int i = 0; i < 10000; i++) {
    }
}

int __io_putchar(int ch)
{
    uart3_write(ch);
    return ch;
}

void uart_driver(int ch)
{
    led_init();
    usart_rxtx_init();

    while (1) {
        printf("Hello STM32H7....\n\r");
        led_toggle(1);
        for (int i = 0; i < 1000000; i++) {
        }
    }
}
