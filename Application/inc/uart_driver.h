/*
 * uart_driver.h
 *
 *  Created on: Jul 24, 2025
 *      Author: benkempers
 */

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include "includes.h"

/* LED DEBUG */

void led_init(void);

void led_toggle(int led_num);

/* UART DRIVER */

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return ((PeriphClk + (BaudRate/2U)) / BaudRate);
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}

void usart_rxtx_init(void);
void usart_rxtx_interrupt_init(void);
char uart3_read(void);
void uart3_write(int ch);

int __io_putchar(int ch);

void USART3_IRQHandler(void);

void uart_driver(int ch);
void uart_interrupt_driver();

/* GPDMA section */

void gpdma1_usart3_init(uint32_t src, uint32_t dst, uint32_t len);
void GPDMA1_CH0_IRQHandler(void);
void usart_dma_driver();


#endif /* UART_DRIVER_H_ */
