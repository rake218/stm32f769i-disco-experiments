/*
 * led.c
 *
 *  Created on: Dec 29, 2025
 *      Author: rakesh
 */


#include "led.h"
#include "cmsis_os.h"

void LedTask1(void const * argument)
{
  UBaseType_t stackWatermark;
  uint8_t stackArr[900], a_u8_lc;

  for (;;)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN);
    osDelay(500);
    stackWatermark = uxTaskGetStackHighWaterMark(NULL);
    (void)stackWatermark;
    for(a_u8_lc = 0; a_u8_lc< 240; a_u8_lc++)
    {
    	stackArr[a_u8_lc] = 240-a_u8_lc;
    }
    (void)stackArr;
  }
}

void LedTask2(void const * argument)
{
  UBaseType_t stackWatermark;

  for (;;)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN);
    osDelay(1000);
    stackWatermark = uxTaskGetStackHighWaterMark(NULL);
    (void)stackWatermark;
  }
}
