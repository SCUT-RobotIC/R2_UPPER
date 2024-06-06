/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "motorctrl.h"
#include "delay.h"
#include "throwball.h"
extern ang_dir MotorSignal[4];
extern PhotogateSpd Spds;
PhotogateAng ANGs;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pins : PDPin PDPin */
  GPIO_InitStruct.Pin = KEY2_Pin|KEY_HIGH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG4 PGPin PGPin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|KEY3_Pin|KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_LOW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t KEYNUM)
{
	if(KEYNUM==KEY1_Pin)
	{
		ANGs.ang[0]=(int)MotorSignal[0].thetas;
		ANGs.flag[0]=1;
		delay_us(5000);
	}
	
	if(KEYNUM==KEY2_Pin)
	{
		ANGs.ang[1]=(int)MotorSignal[1].thetas;
		ANGs.flag[1]=1;
		delay_us(5000);
	}
		if(KEYNUM==KEY3_Pin)
	{
		ANGs.ang[2]=(int)MotorSignal[2].thetas;
		ANGs.flag[2]=1;
		delay_us(5000);
	}
		if(KEYNUM==KEY4_Pin)
	{
		ANGs.ang[3]=(int)MotorSignal[3].thetas;
		ANGs.flag[3]=1;
		delay_us(5000);
	}
	
	if (KEYNUM==KEY_HIGH_Pin)
		
	{Spds.flag[0]=1;
	delay_us(5000);
	}
	if (KEYNUM==KEY_LOW_Pin)
		
	{Spds.flag[1]=1;
	delay_us(5000);
	}
}
/* USER CODE END 2 */
