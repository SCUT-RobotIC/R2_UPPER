#ifndef MOTORCTRL_H
#define MOTORCTRL_H
#include "main.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
typedef struct
{
	int dir;
	double thetal;
	double thetan;
	double thetas;
	double err;
	double err1;
	double err2;
	double errl;
	
} ang_dir;

void cala_d(int i);
void get_msgn(void);
void assign_output(void);
void set_mode(int mode0,int mode1,int mode2,int mode3,int mode4,int mode5,int mode6,int mode7,
	            int mode8,int mode9,int mode10,int mode11,int mode12,int mode13,int mode14,int mode15);

void ctrlmotor(double Vx, double Vy, double omega,int dir1,int dir2,int dir3,int dir4,int flag) ;
void pid3508(double p, double i, double d);
void Apid3508(double p, double i, double d);
void pid2006(double p, double i, double d);
void Apid2006(double p, double i, double d);
#endif
