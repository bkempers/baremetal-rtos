#include "hal_usart.h"
// #include "ringbuffer.h"

#include "stm32_assert.h"
#include "stm32h7rsxx_ll_bus.h"
#include "stm32h7rsxx_ll_rcc.h"
#include "stm32h7rsxx_ll_usart.h"

#define USART_IRQ_PRIO 5u
#define USART_TX_RX_TIMEOUT 1000U

typedef enum { USART_BUS_APB1, USART_BUS_APB2 } usart_bus;

static const struct {
    USART_TypeDef *base;
    usart_bus      bus;
    uint32_t       periph;  /* LL_APB*_GRP1_PERIPH_* */
    uint32_t       clk_src; /* LL_RCC_USART*_CLKSOURCE_* */
    IRQn_Type      irqn;
} usart_hw[USART_COUNT] = {
    [USART_1] = {
        USART1, 
        USART_BUS_APB2, 
        LL_APB2_GRP1_PERIPH_USART1,
        LL_RCC_USART1_CLKSOURCE_HSI, 
        USART1_IRQn
    },
    [USART_2] = {
        USART2, 
        USART_BUS_APB1, 
        LL_APB1_GRP1_PERIPH_USART2,
        LL_RCC_USART234578_CLKSOURCE_HSI, 
        USART2_IRQn
    },
    [USART_3] = {
        USART3, 
        USART_BUS_APB1, 
        LL_APB1_GRP1_PERIPH_USART3,
        LL_RCC_USART234578_CLKSOURCE_HSI, 
        USART3_IRQn
    },
};

static struct {
    ringbuffer   tx;
    ringbuffer   rx;
    usart_errors errors;
} usart_state[USART_COUNT];

static const uint32_t usart_ll_wordlen[] = {
    [USART_WORDLEN_7] = LL_USART_DATAWIDTH_7B,
    [USART_WORDLEN_8] = LL_USART_DATAWIDTH_8B,
    [USART_WORDLEN_9] = LL_USART_DATAWIDTH_9B
};

static const uint32_t usart_ll_parity[] = {
    [USART_PARITY_NONE] = LL_USART_PARITY_NONE,
    [USART_PARITY_EVEN] = LL_USART_PARITY_EVEN,
    [USART_PARITY_ODD]  = LL_USART_PARITY_ODD
};

static const uint32_t usart_ll_stopbits[] = {
    [USART_STOPBITS_1] = LL_USART_STOPBITS_1,
    [USART_STOPBITS_2] = LL_USART_STOPBITS_2
};

static const uint32_t usart_ll_direction[] = {
    [USART_MODE_RX]    = LL_USART_DIRECTION_RX,
    [USART_MODE_TX]    = LL_USART_DIRECTION_TX,
    [USART_MODE_TX_RX] = LL_USART_DIRECTION_TX_RX
};

int usart_init(const usart_config *usart)
{
    if (usart == NULL || usart->id <= USART_NONE || usart->id >= USART_COUNT) {
        return -1;
    }

    // assert_param(usart->wordlen < ARRAY_COUNT(usart_ll_wordlen));
    /* ...same for parity/stopbits/mode... */

    const typeof(usart_hw[0]) *hw = &usart_hw[usart->id];

    ringbuffer_init(&usart_state[usart->id].tx);
    ringbuffer_init(&usart_state[usart->id].rx);

    hal_gpio_init(&usart->tx_pin);
    hal_gpio_init(&usart->rx_pin);

    /* HSI keeps baud independent of the clock tree. Must be set before
    * LL_USART_Init, which reads it via LL_RCC_GetUSARTClockFreq. */
    LL_RCC_SetUSARTClockSource(hw->clk_src);

    if (hw->bus == USART_BUS_APB1) {
      LL_APB1_GRP1_EnableClock(hw->periph);
    } else {
      LL_APB2_GRP1_EnableClock(hw->periph);
    }

    LL_USART_InitTypeDef init = {
        .PrescalerValue      = LL_USART_PRESCALER_DIV1,
        .BaudRate            = usart->baudrate,
        .DataWidth           = usart_ll_wordlen[usart->wordlen],
        .StopBits            = usart_ll_stopbits[usart->stopbits],
        .Parity              = usart_ll_parity[usart->parity],
        .TransferDirection   = usart_ll_direction[usart->mode],
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
        .OverSampling        = LL_USART_OVERSAMPLING_16,
    };

    if (LL_USART_Init(hw->base, &init) != SUCCESS) {
        return -1;
    }

    LL_USART_Enable(hw->base);
    if (usart->mode & USART_MODE_TX) {
        while (!LL_USART_IsActiveFlag_TEACK(hw->base)) { }
    }

    if (usart->mode & USART_MODE_RX) {
        while (!LL_USART_IsActiveFlag_REACK(hw->base)) { }
        LL_USART_EnableIT_RXNE_RXFNE(hw->base);
    }
    LL_USART_EnableIT_ERROR(hw->base);

    NVIC_SetPriority(hw->irqn, USART_IRQ_PRIO);
    NVIC_EnableIRQ(hw->irqn);

    return 0;
}

size_t usart_write(usart_id id, const uint8_t *data, size_t len)
{
    size_t n = 0;

    arch_irq_disable();
    while (n < len && !ringbuffer_is_full(&usart_state[id].tx)) {
        ringbuffer_put(&usart_state[id].tx, data[n++]);
    }
    arch_irq_enable();

    if (n > 0) {
        LL_USART_EnableIT_TXE_TXFNF(usart_hw[id].base);
    }

    return n;
}

size_t usart_read(usart_id id, uint8_t *data, size_t len)
{
    size_t n = 0;

    arch_irq_disable();
    while (n < len && !ringbuffer_is_empty(&usart_state[id].rx)) {
        data[n++] = ringbuffer_get(&usart_state[id].rx);
    }
    arch_irq_enable();

    return n;
}

/* For panic paths and pre-scheduler boot, where the ISR cannot run. */
void usart_write_blocking(usart_id id, const uint8_t *data, size_t len)
{
    USART_TypeDef *base = usart_hw[id].base;

    for (size_t i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE_TXFNF(base)) { }
        LL_USART_TransmitData8(base, data[i]);
    }

    while (!LL_USART_IsActiveFlag_TC(base)) { }
}

static void usart_isr(usart_id id)
{
    USART_TypeDef *base = usart_hw[id].base;
    usart_errors  *err  = &usart_state[id].errors;

    /* Errors first: ORE must be cleared or RXNE never asserts again. */
    if (LL_USART_IsActiveFlag_ORE(base)) { err->overrun++; LL_USART_ClearFlag_ORE(base); }
    if (LL_USART_IsActiveFlag_FE(base))  { err->framing++; LL_USART_ClearFlag_FE(base); }
    if (LL_USART_IsActiveFlag_PE(base))  { err->parity++;  LL_USART_ClearFlag_PE(base); }
    if (LL_USART_IsActiveFlag_NE(base))  { err->noise++;   LL_USART_ClearFlag_NE(base); }

    if (LL_USART_IsActiveFlag_RXNE_RXFNE(base)) {
        uint8_t b = LL_USART_ReceiveData8(base); /* the read clears RXNE */
        if (!ringbuffer_is_full(&usart_state[id].rx)) {
            ringbuffer_put(&usart_state[id].rx, b);
        } else {
            err->overrun++; /* software overrun, distinct from ORE */
        }
    }
    if (LL_USART_IsEnabledIT_TXE_TXFNF(base) && LL_USART_IsActiveFlag_TXE_TXFNF(base)) {
}

/* Matches startup.c's naming. Phase 5's *_IRHandler -> *_IRQHandler rename has to update this too. */
void USART3_IRHandler(void) { usart_isr(USART_3); }
