#include "throwball.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "arm_math.h"
#include "bsp_can.h"
#include "motorctrl.h"
#include "stdio.h"
#include "CALCULATE.h"
#include "math.h"
#include "delay.h"
#include "throwball.h"
PhotogateSpd Spds;
extern double LowAng,HighAng;
extern motor_measure_t   *motor_data[8];
void RiseBall_Init(){
//	rtU.yaw_target4=-2000;
//	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,1950);//左降
//		__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,1750);//右降
//	delay_us(500000);
//	Spds.flag[0]=0;
//	Spds.flag[1]=0;
//	Spds.flag[4]=1;
//		rtU.yaw_target4=1000;
//				Spds.flag[2]=0;
//				Spds.flag[3]=0;
//	}
//void set_getball(uint16_t speed){
//		rtU.yaw_target5 = speed;
//		rtU.yaw_target6 = -(speed);
//}
//void motor_off(){
//		__HAL_TIM_SET_COMPARE (&htim2,TIM_CHANNEL_2,1928);
//		rtU.yaw_target5 = 0;
//		rtU.yaw_target6  = 0;
}


//R2坡度上升下降连贯，间隔3s
void movement_take(){
//		__HAL_TIM_SET_COMPARE (&htim2,TIM_CHANNEL_2,1890);
//		

}

//R2坡度下降
void ramp_down(){
//		__HAL_TIM_SET_COMPARE (&htim2,TIM_CHANNEL_2,1884);
}
	

//R2坡度抬起
void ramp_up(){
//		__HAL_TIM_SET_COMPARE (&htim2,TIM_CHANNEL_2,1928);
}




//R2上方舵机扔球


void PhotogateSet()
{
//		if(Spds.flag[4]==1){
//			if(Spds.flag[0]==1&&Spds.flag[2]==0&&rtU.yaw_target4<0)
//		{
//			HighAng=motor_data[4]->ecd+motor_data[4]->circle*8191;
//				Spds.flag[2]=1;
//			Spds.flag[0]=0;
//		}
//		if(Spds.flag[1]==1&&Spds.flag[3]==0&&rtU.yaw_target4>0)
//		{
//			Spds.flag[1]=0;	
//			LowAng=motor_data[4]->ecd+motor_data[4]->circle*8191;
//			if(Spds.flag[6]==0){
//			HighAng=LowAng-234568;
//			Spds.flag[6]=1;
//			}
//			Spds.flag[3]=1;
//			
//		}
//if((motor_data[4]->ecd+motor_data[4]->circle*8191>LowAng&&rtU.yaw_target4>0)&&Spds.flag[3]==1)
//{rtU.yaw_target4=0;

//}
//if((motor_data[4]->ecd+motor_data[4]->circle*8191<HighAng&&rtU.yaw_target4<0)&&Spds.flag[2]==1)
//{rtU.yaw_target4=0;
//}
//}
}

void Set_servo(TIM_HandleTypeDef *htim, uint32_t Channel, double angle, uint32_t countPeriod, uint32_t CycleTime)
{
  uint16_t compare_value = 0;
  if (angle <= 180)
  {
    compare_value = 0.5 * countPeriod / CycleTime + angle * countPeriod / CycleTime / 90;
    __HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
  }
}
