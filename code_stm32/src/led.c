/*
 * led.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */


/* Includes ------------------------------------------------------------------*/
#include "Led.h"
#include <string.h>

/* Private data --------------------------------------------------------------*/
#define HAL_LED_max_handles 6
static HAL_LED_HandleTypeDef * HAL_LED_handles[HAL_LED_max_handles];
static uint32_t HAL_LED_handles_count = 0;

/* HAL functions ---------------------------------------------------------*/

void HAL_Led_Init()
{

}

void HAL_Led_Add(HAL_LED_HandleTypeDef * hled, GPIO_TypeDef * port, uint16_t pin)
{
	hled->port = port;
	hled->pin = pin;
	hled->state = LED_OFF;
	HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
}

void HAL_Led_Set(HAL_LED_HandleTypeDef * hled)
{
	hled->state = LED_ON;
	HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
}

void HAL_Led_Reset(HAL_LED_HandleTypeDef * hled)
{
	hled->state = LED_OFF;
	HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
}

void HAL_Led_Toggle(HAL_LED_HandleTypeDef * hled)
{
	if(hled->state == LED_OFF)
	{
		hled->state = LED_ON;
	}
	else
	{
		hled->state = LED_OFF;
	}
	HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
}

int HAL_Led_Get(HAL_LED_HandleTypeDef * hled)
{
	return hled->state;
}


