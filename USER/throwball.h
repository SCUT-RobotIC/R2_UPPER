#ifndef __THROWBALL_H__
#define __THROWBALL_H__
#include "main.h"
#include "tim.h"
typedef struct
{
	int flag[100];

} PhotogateSpd;
void RiseBall_Init(void);
void throw_ball(void);
void ramp_down(void);
void ramp_up(void);
void movement_take(void);
void set_getball(uint16_t speed);
void motor_off(void);
void PhotogateSet(void);
#endif


