/*
 * adc.h
 *
 *  Created on: Aug 15, 2025
 *      Author: benkempers
 */

#ifndef ADC_H_
#define ADC_H_

#include "includes.h"

void pa4_adc_init();

void pa4_adc_interrupt_init();
void ADC_IRQHandler();

void start_conversion();

uint32_t adc_read();

void adc_driver();
void adc_inerrupt_driver();

#endif /* ADC_H_ */
