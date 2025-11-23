/*
 * exti.h
 *
 *  Created on: Sep 8, 2025
 *      Author: benkempers
 */

#ifndef EXTI_H_
#define EXTI_H_

#include "includes.h"

void pc13_exti_init(void);
void EXTI13_IRQHandler(void);
void gpio_interrupt_driver(void);

#endif /* EXTI_H_ */
