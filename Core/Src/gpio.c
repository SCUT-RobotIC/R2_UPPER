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
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
extern ang_dir MotorSignal[4];
extern PhotogateSpd Spds;
extern motor_measure_t   *motor_data[8];
extern motor_measure_t   *motor_data1[8];

extern double ReductionRatio3508;
extern double ReductionRatoiGear;
extern double ElecExac3508;
extern double HelmInit[4];
int PhoTheta[2]={105,5};
PhotogateAng ANGs;
int AngInit;

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

  /*Configure GPIO pins : PGPin PGPin PGPin PG8 */
  GPIO_InitStruct.Pin = KEY4_Pin|KEY_LOW_Pin|KEY3_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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

		if(((motor_data1[0]->circle*8191+motor_data1[0]->ecd)>(50*36*8191*93/(35*360)))&&ANGs.flag[0]==0)
		{
			HelmInit[0]=motor_data1[0]->circle*8191+motor_data1[0]->ecd;
		}
		ANGs.flag[0]=1;
		if(((motor_data1[0]->circle*8191+motor_data1[0]->ecd)<(50*36*8191*93/(35*360))))
		{
		ANGs.flag[0]=0;
		}
		delay_us(5000);
	}
	
	if(KEYNUM==KEY2_Pin)
	{
		if(((motor_data1[1]->circle*8191+motor_data1[1]->ecd)>(50*36*8191*93/(35*360)))&&ANGs.flag[1]==0)
		{
			HelmInit[1]=motor_data1[1]->circle*8191+motor_data1[1]->ecd;
		}
		ANGs.flag[1]=1;
				if(((motor_data1[1]->circle*8191+motor_data1[1]->ecd)<(50*36*8191*93/(35*360))))
		{
		ANGs.flag[1]=0;
		}
		
		delay_us(5000);
	}
		if(KEYNUM==KEY3_Pin)
	{
		if(((motor_data1[2]->circle*8191+motor_data1[2]->ecd)>(50*36*8191*93/(35*360)))&&ANGs.flag[2]==0)
		{
			HelmInit[2]=motor_data1[2]->circle*8191+motor_data1[0]->ecd;
		}
		ANGs.flag[2]=1;
				if(((motor_data1[2]->circle*8191+motor_data1[2]->ecd)<(50*36*8191*93/(35*360))))
		{
		ANGs.flag[2]=0;
		}
		delay_us(5000);
	}
		if(KEYNUM==KEY4_Pin)
	{
		if(((motor_data1[3]->circle*8191+motor_data1[3]->ecd)>(50*36*8191*93/(35*360)))&&ANGs.flag[3]==0)
		{
			HelmInit[3]=motor_data1[3]->circle*8191+motor_data1[3]->ecd;
		}
		ANGs.flag[3]=1;
				if(((motor_data1[3]->circle*8191+motor_data1[3]->ecd)<(50*36*8191*93/(35*360))))
		{
		ANGs.flag[3]=0;
		}
		delay_us(5000);
	}
	
	if (KEYNUM==KEY_HIGH_Pin)
		
	{Spds.flag[0]=1;
		if(Spds.flag[1]!=1)
		AngInit=motor_data[6]->ecd+motor_data[6]->circle*8191+PhoTheta[0]*ElecExac3508*ReductionRatoiGear*ReductionRatio3508;
	delay_us(5000);
	}
	if (KEYNUM==KEY_LOW_Pin)
		
	{Spds.flag[1]=1;
	 if(Spds.flag[0]!=1)
		AngInit=motor_data[6]->ecd+motor_data[6]->circle*8191+PhoTheta[1]*ElecExac3508*ReductionRatoiGear*ReductionRatio3508;
	delay_us(5000);
	}
}
/* USER CODE END 2 */
