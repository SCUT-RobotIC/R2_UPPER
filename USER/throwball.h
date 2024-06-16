#ifndef __THROWBALL_H__
#define __THROWBALL_H__
#include "main.h"
#include "tim.h"
typedef struct
{
	int flag[100];

} PhotogateSpd;
void RiseBall_Init(void);

void ramp_down(void);
void ramp_up(void);
void movement_take(void);
void set_getball(uint16_t speed);
void motor_off(void);
void PhotogateSet(void);
void Set_servo(TIM_HandleTypeDef *htim, uint32_t Channel, double angle, uint32_t countPeriod, uint32_t CycleTime);
#endif


