/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : user_function.h
  * @brief          : Header for user_function.c file.
  *                   This file contains user define function.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef USER_FUNCTION
#define USER_FUNCTION

#include "main.h"
#include "stdio.h"
#include "string.h"

void blink(GPIO_TypeDef * GPIOx, uint16_t GPIO_PIN_x, uint16_t Rate);

float ComputeLowpassConstant(uint16_t CutoffFreq, uint16_t SamplingFreq);

#endif
