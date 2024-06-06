/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "pidctl.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "CALCULATE.h"
#include "math.h"
#include "delay.h"
#include "throwball.h"
#include "solve.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define VEL      1
#define ANG      2
double LowAng,HighAng;
extern int condition[10];
extern motor_measure_t   *motor_data[8];
extern motor_measure_t   *motor_data1[8];
extern TGT_COOR TC;
extern REAL_COOR RC;
extern ang_dir MotorSignal[4];
extern PhotogateAng ANGs;
extern PhotogateSpd Spds;
typedef struct __FILE FILE;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


extern int angtemp[4];
extern int flagf[4];
extern double Vx,Vy,omega;
extern int swich[4];

extern int receivefactor[4];
int factor[100]={0};



int flag=0;
int flag1=0;
extern int dir[4];
extern int ang[4];


extern double theta[4];

extern DataPacket DataRe;
extern int16_t lx,ly,rx,ry,lp,rp;
extern uint8_t B1,B2;
extern uint8_t Cal_Parity;
	
int i;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ThrowBall */
osThreadId_t ThrowBallHandle;
const osThreadAttr_t ThrowBall_attributes = {
  .name = "ThrowBall",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Rise */
osThreadId_t RiseHandle;
const osThreadAttr_t Rise_attributes = {
  .name = "Rise",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for Photoate */
osThreadId_t PhotoateHandle;
const osThreadAttr_t Photoate_attributes = {
  .name = "Photoate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Throw(void *argument);
void RiseBall(void *argument);
void ChassisControl(void *argument);
void RiseBallPhotoate(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ThrowBall */
  ThrowBallHandle = osThreadNew(Throw, NULL, &ThrowBall_attributes);

  /* creation of Rise */
  RiseHandle = osThreadNew(RiseBall, NULL, &Rise_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(ChassisControl, NULL, &Chassis_attributes);

  /* creation of Photoate */
  PhotoateHandle = osThreadNew(RiseBallPhotoate, NULL, &Photoate_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {	
		for(int i=0;i<4;i++){
		if(ANGs.flag[i]!=1&&receivefactor[0]==1){
				MotorSignal[i].thetas=MotorSignal[i].thetas+0.3;

		}
}
					
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Throw */
/**
* @brief Function implementing the ThrowBall thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Throw */
void Throw(void *argument)
{
  /* USER CODE BEGIN Throw */
  /* Infinite loop */
  for(;;)
  {		if(condition[5]==1){//按下按键6
		set_getball(3390);
		movement_take();
	}
		if(condition[0]==1)
		   motor_off();
osDelay(1);

  }

  /* USER CODE END Throw */
}

/* USER CODE BEGIN Header_RiseBall */
/**
* @brief Function implementing the Rise thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RiseBall */
void RiseBall(void *argument)
{
  /* USER CODE BEGIN RiseBall */
  /* Infinite loop */
  for(;;)
  {

		if(condition[1]==1){
				rtU.yaw_target4=-3000;
				while(1)
					{
if(fabs((motor_data[4]->ecd+motor_data[4]->circle*8191)-HighAng)<8191)
{
	osDelay(500);
	throw_ball();
	  rtU.yaw_target4=2000;

	break;
}
}
		}	
				//	Spds.flag[5]=0;
osDelay(1);
  }
  /* USER CODE END RiseBall */
}

/* USER CODE BEGIN Header_ChassisControl */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisControl */
void ChassisControl(void *argument)
{
  /* USER CODE BEGIN ChassisControl */
  /* Infinite loop */
  for(;;)
  {

		if(swich[1]==0){

		Vx=(rx)/9;
		Vy=(ry)/9;
		omega=-(lx)/9;
	if(fabs(Vx)<1000)
		Vx=0;
	if(fabs(Vy)<1000)
		Vy=0;
 if(fabs(omega)<1000)
		omega=0;
		}			
		else if(swich[1]==1)
		{
		Vx=RC.Vx;
		Vy=RC.Vy;
		omega=RC.omega;	
		}
			factor[1]++;
		
		if(receivefactor[0]==0)//没接收到就增加标志位
			factor[0]++;
		if(factor[0]>300){
			Vx=0;Vy=0;omega=0;rx=0;ry=0;lx=0;ly=0;
			factor[0]=301;
		}//1s没收到就全部停下
		if(receivefactor[0]==1)//接收到就标志位置0
			factor[0]=0;
		
		if (factor[1]==50){
			receivefactor[0]=0;//0.05s更新1次确定为没接收到
			factor[1]=0;
		}

		
//		if(swich[1]==1){
//			factor[3]++;
//		
//		if(receivefactor[1]==0)//没接收到就增加标志位
//			factor[2]++;
//		if(factor[2]>300){
//			Vx=0;Vy=0;omega=0;
//			factor[2]=301;
//		}//1s没收到就全部停下
//		if(receivefactor[1]==1)//接收到就标志位置0
//			factor[2]=0;
//		
//		if (factor[3]==1000){
//			receivefactor[1]=0;//1s更新1次确定为没接收到
//			factor[3]=0;
//		}
//	}
		HAL_IWDG_Refresh(&hiwdg);//喂狗
		
		/* SPD TEST */
		

		get_msgn();
		
		

		ctrlmotor( Vx,  Vy,  omega,dir[0],dir[1],dir[2],dir[3],flag1);
		

		rtU.yaw_target7=0;
		rtU.yaw_target8    = theta[0]*36*8191*47/(17*360); 
		rtU.yaw_target9    = theta[1]*36*8191*47/(17*360); 
		rtU.yaw_target10   = theta[2]*36*8191*47/(17*360); 
		rtU.yaw_target11   = theta[3]*36*8191*47/(17*360); 
		rtU.yaw_target12=1000;
		rtU.yaw_target13=2000;
		rtU.yaw_target14=3000;
		rtU.yaw_target15=4000;
    osDelay(1);
  }
  /* USER CODE END ChassisControl */
}

/* USER CODE BEGIN Header_RiseBallPhotoate */
/**
* @brief Function implementing the Photoate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RiseBallPhotoate */
void RiseBallPhotoate(void *argument)
{
  /* USER CODE BEGIN RiseBallPhotoate */
  /* Infinite loop */
  for(;;)
  {
	
	for(int i=0;i<4;i++){
      if(ANGs.flag[i]==1&&flagf[i]!=1){
						ang[i]=ang[i]+ANGs.ang[i];
				    angtemp[i]=ANGs.ang[i];
						MotorSignal[i].thetas=0;
				    flagf[i]=1;
			}
		if(flagf[i]==1&&angtemp[i]<50)
		{
			ANGs.flag[i]=0;
			ANGs.ang[i]=0;
			flagf[i]=0;
		}
}

		if((fabs(Vx)>100||fabs(Vy)>100||fabs(omega)>100)&&flagf[0]==1&&flagf[1]==1&&flagf[2]==1&&flagf[3]==1){
		MotorSignal[0].thetan=atan2(Vy-omega*cos(atan(3.0/4.0)), Vx-omega*sin(atan(3.0/4.0)))*180/PI;
		MotorSignal[1].thetan=atan2(Vy-omega*cos(atan(3.0/4.0)), Vx+omega*sin(atan(3.0/4.0)))*180/PI;
		MotorSignal[2].thetan=atan2(Vy+omega*cos(atan(3.0/4.0)), Vx+omega*sin(atan(3.0/4.0)))*180/PI;			
		MotorSignal[3].thetan=atan2(Vy+omega*cos(atan(3.0/4.0)), Vx-omega*sin(atan(3.0/4.0)))*180/PI;

			for(int i=0;i<4;i++){
			if(fabs(MotorSignal[i].thetan-90)<1)
				MotorSignal[i].thetan=91;
			if(fabs(MotorSignal[i].thetan+90)<1)
				MotorSignal[i].thetan=-89;}
			
			flag1=1;
		}else if (flagf[0]==1&&flagf[1]==1&&flagf[2]==1&&flagf[3]==1){

		flag1=0;
		}

for(int i=0;i<4;i++){
      cala_d(i);
			if(fabs((double)((int)MotorSignal[i].thetas%360-90))<1)
				MotorSignal[i].thetas=91+(int)MotorSignal[i].thetas/360*360;
			if(fabs((double)((int)MotorSignal[i].thetas%360+90))<1)
				MotorSignal[i].thetas=-89+(int)MotorSignal[i].thetas/360*360;
			if(fabs((double)((int)MotorSignal[i].thetas%360-270))<1)
				MotorSignal[i].thetas=271+(int)MotorSignal[i].thetas/360*360;
			if(fabs((double)((int)MotorSignal[i].thetas%360+270))<1)
				MotorSignal[i].thetas=-269+(int)MotorSignal[i].thetas/360*360;
			theta[i]=MotorSignal[i].thetas+ang[i];
			dir[i]=MotorSignal[i].dir;
      MotorSignal[i].thetal=MotorSignal[i].thetan;
}
    osDelay(1);
  }
  /* USER CODE END RiseBallPhotoate */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

