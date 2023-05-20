/*
 * user_function.c
 *
 *  Created on: Feb 12, 2023
 *      Author: 08809
 */

#include "user_function.h"

// USER DEFINE FUNCTION BEGIN
void blink(GPIO_TypeDef * GPIOx, uint16_t GPIO_PIN_x, uint16_t Rate) {
	static uint32_t BlinkTime = 0;
	if (BlinkTime == 0) BlinkTime = HAL_GetTick() + Rate;
	if (HAL_GetTick() >= BlinkTime)
	{
		HAL_GPIO_TogglePin(GPIOx, GPIO_PIN_x);
		BlinkTime += Rate;
	}
}

float ComputeLowpassConstant(uint16_t CutoffFreq, uint16_t SamplingFreq) {
	return CutoffFreq / ((float) (CutoffFreq + SamplingFreq));
}
// USER DEFINE FUNCTION END
