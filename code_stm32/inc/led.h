/*
 * led.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_LED_H_
#define APPLICATION_USER_HAL_LED_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


#define LED_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_13

enum LED_STATE
{
    LED_OFF = 0,
    LED_ON = 1
};

/* HAL Functions ------------------------------------------------------------------*/

void HAL_Led_Add(HAL_LED_HandleTypeDef * hled, GPIO_TypeDef * port, uint16_t pin)
void HAL_Led_Set(HAL_LED_HandleTypeDef * hled);
void HAL_Led_Reset(HAL_LED_HandleTypeDef * hled);
void HAL_Led_Toggle(HAL_LED_HandleTypeDef * hled);
int HAL_Led_Get(HAL_LED_HandleTypeDef * hled);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_HAL_LED_H_ */
