/*
 * led_driver.h
 *
 *  Created on: Jul 24, 2025
 *      Author: benkempers
 */

#ifndef LED_DRIVER_H_
#define LED_DRIVER_H_

#include "includes.h"

void led_init(void);
void led_toggle(int led_num);
void run_gpio_led(bool button);

#endif /* LED_DRIVER_H_ */
