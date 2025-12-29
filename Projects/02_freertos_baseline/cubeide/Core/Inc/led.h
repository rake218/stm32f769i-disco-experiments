/*
 * led.h
 *
 *  Created on: Dec 29, 2025
 *      Author: rakesh
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#ifndef LED_H
#define LED_H

#include "stm32f7xx_hal.h"
#include "cmsis_os.h"

/*
 * Board-specific LED mapping
 * STM32F769I-DISCO
 * LEDs are active-low
 * GPIO must be initialized in MX_GPIO_Init()
 */

#define LED1_GPIO_PORT GPIOJ
#define LED1_GPIO_PIN  GPIO_PIN_13

#define LED2_GPIO_PORT GPIOJ
#define LED2_GPIO_PIN  GPIO_PIN_5

/* RTOS task prototypes */
void LedTask1(void const * argument);
void LedTask2(void const * argument);

#endif /* LED_H */


#endif /* INC_LED_H_ */
