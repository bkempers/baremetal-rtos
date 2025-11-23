/*
 * systick.h
 *
 *  Created on: Aug 18, 2025
 *      Author: benkempers
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "includes.h"

void systickDelayMS(int delay);
void systick_driver(int delay);

void systick_interrupt_driver();
void systickInterrupt();
void SysTick_Handler();
void systick_callback();

#endif /* SYSTICK_H_ */
