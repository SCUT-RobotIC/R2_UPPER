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
#include "PID_MODEL.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "calculate.h"
#include "math.h"
#include "delay.h"
#include "throwball.h"
#include "solve.h"

// #include "R1_Ball_MODEL.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VEL 1
#define ANG 2

/**
 * @brief motor definition
 * motor_data[0] - motor_data[3]: chassis motors
 * motor_data[4]: ball shooter motor
 * motor_data[5]: ball right motor
 * motor_data[6]: ball left motor
 * motor_data[7]: ball lift motor
 */

#define Throwfai        100//极坐标定义舵�?
#define Throwtheta      115 
double ReductionRatio3508=3591/187;
double ReductionRatoiGear=143/58;
double ElecExac3508=8191/360;
double LowAng,HighAng;
extern int AngInit;
extern int angtemp[4];
extern int flagf[4];
extern int dir[4];
extern int ang[4];
extern int condition[10];
extern motor_measure_t   *motor_data[8];
extern motor_measure_t   *motor_data1[8];
extern TGT_COOR TC;
extern REAL_COOR RC;
extern ang_dir MotorSignal[4];
extern PhotogateAng ANGs;
extern PhotogateSpd Spds;
extern double Vx,Vy,omega;
extern int swich[4];
extern int receivefactor[4];
extern DataPacket DataRe;
extern int16_t lx,ly,rx,ry,lp,rp;
extern uint8_t B1,B2;
extern uint8_t Cal_Parity;

typedef struct __FILE FILE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


int actionFlag[10];
int statue[10]={0};
int factor[100]={0};
int conditionCounter[10];
int conditionFlag[10];
int AngInitFlag;
int flag=0;
int flag1=0;

int buff_len;
char TransmitBuffer[100];

double HelmInit[4]={0};


extern double theta[4];

int McKnumFlag=0;

	
int i[10];
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
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for cha */
osThreadId_t chaHandle;
const osThreadAttr_t cha_attributes = {
  .name = "cha",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for logicchassis */
osThreadId_t logicchassisHandle;
const osThreadAttr_t logicchassis_attributes = {
  .name = "logicchassis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for Transmit */
osThreadId_t TransmitHandle;
const osThreadAttr_t Transmit_attributes = {
  .name = "Transmit",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Rise */
osThreadId_t RiseHandle;
const osThreadAttr_t Rise_attributes = {
  .name = "Rise",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for Throwball */
osThreadId_t ThrowballHandle;
const osThreadAttr_t Throwball_attributes = {
  .name = "Throwball",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void throw_ball(){
	if(AngInit!=0){
    rtU.yaw_status_CH1_7=2;	
		Set_servo(&htim3, TIM_CHANNEL_2, Throwtheta, 20000, 20);//夹住
		vTaskDelay(500);
		Set_servo(&htim3, TIM_CHANNEL_1, 10, 20000, 20);//抬起
		vTaskDelay(500);
		for(int i=0;i<110;i++){
		rtU.yaw_target_CH1_7= i*ElecExac3508*ReductionRatoiGear*ReductionRatio3508+AngInit;
		vTaskDelay(3);
		}
		Set_servo(&htim3, TIM_CHANNEL_1, 0, 20000, 20);//抬起
		vTaskDelay(200);
		Set_servo(&htim3, TIM_CHANNEL_2, 0, 20000, 20);//撒手
		vTaskDelay(1300);
	  Set_servo(&htim3, TIM_CHANNEL_1, 10, 20000, 20);//抬起
		vTaskDelay(200);
		for(int i=110;i>0;i--){
		rtU.yaw_target_CH1_7= i*ElecExac3508*ReductionRatoiGear*ReductionRatio3508+AngInit;
		vTaskDelay(3);
			
		}
		Set_servo(&htim3, TIM_CHANNEL_1, Throwfai, 20000, 20);
		
	}
}

void put_ball(){
		if(AngInit!=0){
		Set_servo(&htim3, TIM_CHANNEL_2, Throwtheta, 20000, 20);//夹住
		vTaskDelay(500);
		Set_servo(&htim3, TIM_CHANNEL_1, 0, 20000, 20);//抬起
		vTaskDelay(500);
		Set_servo(&htim3, TIM_CHANNEL_2, 0, 20000, 20);//撒手
		vTaskDelay(2000);
		Set_servo(&htim3, TIM_CHANNEL_1, Throwfai, 20000, 20);
		}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void chasfunc(void *argument);
void chalogic(void *argument);
void StartTransmit(void *argument);
void StartRiseBall(void *argument);
void StartThrowball(void *argument);

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

  /* creation of cha */
  chaHandle = osThreadNew(chasfunc, NULL, &cha_attributes);

  /* creation of logicchassis */
  logicchassisHandle = osThreadNew(chalogic, NULL, &logicchassis_attributes);

  /* creation of Transmit */
  TransmitHandle = osThreadNew(StartTransmit, NULL, &Transmit_attributes);

  /* creation of Rise */
  RiseHandle = osThreadNew(StartRiseBall, NULL, &Rise_attributes);

  /* creation of Throwball */
  ThrowballHandle = osThreadNew(StartThrowball, NULL, &Throwball_attributes);

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
  for (;;)
  {
		
		if(condition[1]==1)
			conditionCounter[1]+=1;

		if(condition[4]==1)
			conditionCounter[4]+=1;

		
		if(conditionCounter[1]>1000||conditionFlag[1]==1){
			conditionFlag[1]=1;
			if(HelmInit[0]==0)
			rtU.yaw_target_CH2_1=-5000;
				if(HelmInit[1]==0)
			rtU.yaw_target_CH2_2=-5000;
				if(HelmInit[2]==0)
			rtU.yaw_target_CH2_3=-5000;
				if(HelmInit[3]==0)
			rtU.yaw_target_CH2_4=-5000;
		
	
		
		
		if(AngInit==0)
		{
			rtU.yaw_target_CH1_7=-1000;
		}
		else if(AngInitFlag==0&&AngInit!=0)
		{
			AngInitFlag=1;
			Set_servo(&htim3, TIM_CHANNEL_1, 100, 20000, 20);//fai(极坐标表示的舵机)
			rtU.yaw_status_CH1_7=2;
//			
//			if(AngInit>0)
//				{
//		for(int i=0;i*ElecExac3508*ReductionRatoiGear*ReductionRatio3508<AngInit;i++)
//					{
//		      rtU.yaw_target_CH1_7= i*ElecExac3508*ReductionRatoiGear*ReductionRatio3508;
//		      vTaskDelay(100);
//				  }
//			 }
//				else if(AngInit<0)
//				{
         rtU.yaw_target_CH1_7=AngInit;
//			 }
		}
		}

	if(conditionCounter[4]>1000)
	{
		__disable_irq(); //关闭�?有中�? 
		NVIC_SystemReset(); //复位
	}
		

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_chasfunc */
/**
* @brief Function implementing the cha thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chasfunc */
void chasfunc(void *argument)
{
  /* USER CODE BEGIN chasfunc */
  /* Infinite loop */
  for(;;)
  {
		
if(swich[1]==0){

		Vx=(rx)/8;
		Vy=(ry)/8;
		omega=(lx)/8;
	if(fabs(Vx)<500)
		Vx=0;
	if(fabs(Vy)<500)
		Vy=0;
 if(fabs(omega)<500)
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

		
		if(swich[1]==1){
			factor[3]++;
		
		if(receivefactor[1]==0)//没接收到就增加标志位
			factor[2]++;
		if(factor[2]>300){
			Vx=0;Vy=0;omega=0;
			factor[2]=301;
		}//1s没收到就全部停下
		if(receivefactor[1]==1)//接收到就标志位置0
			factor[2]=0;
		
		if (factor[3]==1000){
			receivefactor[1]=0;//1s更新1次确定为没接收到
			factor[3]=0;
		}
	}
		HAL_IWDG_Refresh(&hiwdg);//喂狗
		
		/* SPD TEST */
		

		get_msgn();
		
		

		ctrlmotor( Vx,  Vy,  omega,dir[0],dir[1],dir[2],dir[3],flag1);
		
if(HelmInit[0]!=0)
{
	rtU.yaw_status_CH2_1=2;
		rtU.yaw_target_CH2_1 = MotorSignal[0].thetas*36*8191*93/(35*360)+HelmInit[0]; 
}
if(HelmInit[1]!=0) {    
	rtU.yaw_status_CH2_2=2;	
		rtU.yaw_target_CH2_2 = MotorSignal[1].thetas*36*8191*93/(35*360)+HelmInit[1]; 
}
if(HelmInit[2]!=0)  {
	rtU.yaw_status_CH2_3=2;	
		rtU.yaw_target_CH2_3 = MotorSignal[2].thetas*36*8191*93/(35*360)+HelmInit[2]; 
}
if(HelmInit[3]!=0) {
	rtU.yaw_status_CH2_4=2;	
		rtU.yaw_target_CH2_4 =MotorSignal[3].thetas *36*8191*93/(35*360)+HelmInit[3]; 
}
    osDelay(1);
  }
  /* USER CODE END chasfunc */
}

/* USER CODE BEGIN Header_chalogic */
/**
* @brief Function implementing the logicchassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chalogic */
void chalogic(void *argument)
{
  /* USER CODE BEGIN chalogic */
  /* Infinite loop */
  for(;;)
  {

		if((fabs(Vx)>500||fabs(Vy)>500||fabs(omega)>500)&&HelmInit[0]!=0&&HelmInit[1]!=0&&HelmInit[2]!=0&&HelmInit[3]!=0){
		MotorSignal[0].thetan=atan2(Vy-omega*cos(atan(1)), Vx-omega*sin(atan(1)))*180/PI;
		MotorSignal[1].thetan=atan2(Vy-omega*cos(atan(1)), Vx+omega*sin(atan(1)))*180/PI;
		MotorSignal[2].thetan=atan2(Vy+omega*cos(atan(1)), Vx+omega*sin(atan(1)))*180/PI;			
		MotorSignal[3].thetan=atan2(Vy+omega*cos(atan(1)), Vx-omega*sin(atan(1)))*180/PI;

			for(int i=0;i<4;i++){
			if(fabs(MotorSignal[i].thetan-90)<1)
				MotorSignal[i].thetan=91;
			if(fabs(MotorSignal[i].thetan+90)<1)
				MotorSignal[i].thetan=-89;}
			
			flag1=1;
		}else if(HelmInit[0]!=0&&HelmInit[1]!=0&&HelmInit[2]!=0&&HelmInit[3]!=0){
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
  /* USER CODE END chalogic */
}

/* USER CODE BEGIN Header_StartTransmit */
/**
* @brief Function implementing the Transmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmit */
void StartTransmit(void *argument)
{
  /* USER CODE BEGIN StartTransmit */
  /* Infinite loop */
  for(;;)
  {
//		buff_len = sprintf(TransmitBuffer,"fg %d %d %d %d\r\n",motor_data[0]->speed_rpm,motor_data[1]->speed_rpm,motor_data[2]->speed_rpm,motor_data[3]->speed_rpm);
//			HAL_UART_Transmit_DMA(&huart2,(uint8_t *)TransmitBuffer,buff_len);	
		buff_len = sprintf(TransmitBuffer,"fg %d %d %d %d %d %d %d %d\r\n",motor_data[0]->speed_rpm,motor_data[1]->speed_rpm,motor_data[2]->speed_rpm,motor_data[3]->speed_rpm,(int)rtU.yaw_target_CH1_1,(int)rtU.yaw_target_CH1_2,(int)rtU.yaw_target_CH1_3,(int)rtU.yaw_target_CH1_4);
		HAL_UART_Transmit_DMA(&huart2,(uint8_t *)TransmitBuffer,buff_len);

    osDelay(1);
  }
  /* USER CODE END StartTransmit */
}

/* USER CODE BEGIN Header_StartRiseBall */
/**
* @brief Function implementing the Rise thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRiseBall */
void StartRiseBall(void *argument)
{
  /* USER CODE BEGIN StartRiseBall */
  /* Infinite loop */

  for(;;)
  {
		if(swich[1]==0){
		if(condition[0]==1)
			conditionCounter[0]+=1;
		if(condition[1]==1)
			conditionCounter[1]+=1;
		
		
		if(conditionCounter[0]>50&&conditionFlag[0]==0)
		{
			statue[0]=1-statue[0];
			conditionFlag[0]=1;
		}
		else if(conditionCounter[0]==0){
		conditionFlag[0]=0;
		}
			
		if(statue[0]==0)
		{
			McKnumFlag=0;
			rtU.yaw_target_CH1_6=0;
			rtU.yaw_target_CH1_5=0;
		}
		else if(statue[0]==1&&McKnumFlag==0)
		{
			McKnumFlag=1;
			rtU.yaw_target_CH1_6=3000;//麦轮
			rtU.yaw_target_CH1_5=9000;
		}
		
		
		
//		if(conditionCounter[1]>100&&conditionFlag[1]==0)
//		{
//			statue[1]=1-statue[1];
//			conditionFlag[1]=1;
//		}
//		else if(conditionCounter[1]==0){
//		conditionFlag[1]=0;
//		}
//			
//		if(statue[1]==0)
//		{
//			rtU.yaw_target_CH1_6=0;
//		}
//		else
//		{
//			rtU.yaw_target_CH1_6=3000;
//		}
		    
	}
		else if (swich[1]==1){
			if(RC.action==1&&McKnumFlag==0){
				McKnumFlag=1;
			rtU.yaw_target_CH1_6=3000;//麦轮
			rtU.yaw_target_CH1_5=9000;

			}else if(RC.action!=1)
			{
				McKnumFlag=0;
		  rtU.yaw_target_CH1_6=0;//麦轮
			rtU.yaw_target_CH1_5=0;
			}
		}
osDelay(1);
  }
  /* USER CODE END StartRiseBall */
}

/* USER CODE BEGIN Header_StartThrowball */
/**
* @brief Function implementing the Throwball thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartThrowball */
void StartThrowball(void *argument)
{
  /* USER CODE BEGIN StartThrowball */
  /* Infinite loop */
  for(;;)
  {
		if(swich[1]==0){
		if(condition[2]==1)
			conditionCounter[2]+=1;
		if(condition[3]==1)
			conditionCounter[3]+=1;
		
		if(conditionCounter[2]>100){
        throw_ball();
		}
		if(conditionCounter[3]>100){
        put_ball();

		}
	}
		else if(swich[1]==1)
		{
			if(RC.action==2&&actionFlag[2]==0){
        throw_ball();
				actionFlag[2]=1;
			}
			else if(!(RC.action==2))
				actionFlag[2]=0;
			
			if(RC.action==3&&actionFlag[3]==0){
        put_ball();
				actionFlag[3]=1;
			}
			else if(!(RC.action==3))
				actionFlag[3]=0;
			
		}
		

    osDelay(1);
  }
  /* USER CODE END StartThrowball */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

