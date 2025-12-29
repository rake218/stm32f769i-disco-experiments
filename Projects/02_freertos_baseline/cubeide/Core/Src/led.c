/*
 * led.c
 *
 *  Created on: Dec 29, 2025
 *      Author: rakesh
 */


#include "led.h"

void LedTask1(void const * argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN);
    osDelay(500);
  }
}

void LedTask2(void const * argument)
{
  for (;;)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN);
    osDelay(1000);
  }
}
