#ifndef HAL_USART_H
#define HAL_USART_H

#include <stddef.h>
#include <stdint.h>

#include "hal_gpio.h"

typedef enum {
    USART_PARITY_NONE = 0,
    USART_PARTIY_EVEN,
    USART_PARITY_ODD,
} usart_parity;

typedef enum {
    USART_STOPBITS_1 = 0,
    USART_STOPBITS_2,
} usart_stopbits;

typedef enum {
    USART_WORDLEN_7 = 0,
    USART_WORDLEN_8,
    USART_WORDLEN_9
} usart_wordlen;

typedef enum {
    USART_MODE_RX = 0x1,
    USART_MODE_TX = 0x2,
    USART_MODE_TX_RX = 0x3
} usart_mode;

typedef struct {
    // possible ID for unique usarts
    uint32_t baudrate;
    usart_wordlen wordlen;
    usart_parity parity;
    usart_stopbits stopbits;
    usart_mode mode;

    //clk polarity, phase, last bit, clock prescaler

    gpio_init tx_pin;
    gpio_init rx_pin;
} usart_config;

//TODO: maybe look at adding multiple usart_id classifier

int usart_init(const usart_config *usart);
void usart_deinit();

/* non-blocking */
size_t usart_write(const uint8_t *data, size_t len);
size_t usart_read(uint8_t *data, size_t len);

size_t usart_write_space();
size_t usart_read_available();

/* blocking */
void usart_write_blocking(const uint8_t *data, size_t len);
void usart_drain();

typedef struct {
    uint32_t overrun;
    uint32_t framing;
    uint32_t parity;
    uint32_t noise;
} usart_errors;

void usart_get_errors(usart_errors *out);

#endif // HAL_USART_H
