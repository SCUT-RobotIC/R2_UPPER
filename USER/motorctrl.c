#include "motorctrl.h"
ang_dir MotorSignal[4];
extern motor_measure_t   *motor_data[8];
extern motor_measure_t   *motor_data1[8];
double output[16];
extern PhotogateAng ANGs;
extern int flagf[4];
extern int flag;
extern int flag1;
extern int dir[4];
extern int ang[4];
extern int angtemp[4];
extern double theta[4];
extern int swich[4];
int BrakeAng[4];
int BrakeFlag=0;
double mult=1;
double temp[3];

void ctrlmotor(double Vx, double Vy, double omega,int dir1,int dir2,int dir3,int dir4,int flag) {

//for(int i=0;i<4;i++){
//      if(fabs((double)((int)MotorSignal[i].thetas%360-90))<1)
//				MotorSignal[i].thetas=90+(int)MotorSignal[i].thetas/360*360;
//			if(fabs((double)((int)MotorSignal[i].thetas%360+90))<1)
//				MotorSignal[i].thetas=-90+(int)MotorSignal[i].thetas/360*360;
//			if(fabs((double)((int)MotorSignal[i].thetas%360-270))<1)
//				MotorSignal[i].thetas=270+(int)MotorSignal[i].thetas/360*360;
//			if(fabs((double)((int)MotorSignal[i].thetas%360+270))<1)
//				MotorSignal[i].thetas=-270+(int)MotorSignal[i].thetas/360*360;
//}
	if(flag==1){
	while(  (fabs(sqrt(pow((Vy-omega*cos(atan(3.0/4.0))),2)+pow((Vx-omega*sin(atan(3.0/4.0))),2))*mult)>4000)||
		      (fabs(sqrt(pow((Vy+omega*cos(atan(3.0/4.0))),2)+pow((Vx-omega*sin(atan(3.0/4.0))),2))*mult)>4000)||
	        (fabs(sqrt(pow((Vy+omega*cos(atan(3.0/4.0))),2)+pow((Vx+omega*sin(atan(3.0/4.0))),2))*mult)>4000)||
	        (fabs(sqrt(pow((Vy-omega*cos(atan(3.0/4.0))),2)+pow((Vx+omega*sin(atan(3.0/4.0))),2))*mult)>4000))
	  {mult=0.98*mult;

		}

			BrakeFlag=0;
			rtU.yaw_status0=1;
			rtU.yaw_status1=1;
			rtU.yaw_status2=1;
			rtU.yaw_status3=1;
			rtU.yaw_target0= sqrt(pow((Vy-omega*cos(atan(3.0/4.0))),2)+pow((Vx-omega*sin(atan(3.0/4.0))),2))*mult;
			rtU.yaw_target1 =sqrt(pow((Vy-omega*cos(atan(3.0/4.0))),2)+pow((Vx+omega*sin(atan(3.0/4.0))),2))*mult;
			rtU.yaw_target2= sqrt(pow((Vy+omega*cos(atan(3.0/4.0))),2)+pow((Vx+omega*sin(atan(3.0/4.0))),2))*mult;	
			rtU.yaw_target3 =sqrt(pow((Vy+omega*cos(atan(3.0/4.0))),2)+pow((Vx-omega*sin(atan(3.0/4.0))),2))*mult;
	if(dir1==1)
		rtU.yaw_target0=-rtU.yaw_target0;
  if(dir2==1)
		rtU.yaw_target1=-rtU.yaw_target1;
	if(dir3==1)
		rtU.yaw_target2=-rtU.yaw_target2;
	if(dir4==1)
		rtU.yaw_target3=-rtU.yaw_target3;		
	}
	     else{

			rtU.yaw_status0=2;
			rtU.yaw_status1=2;
			rtU.yaw_status2=2;
			rtU.yaw_status3=2;
				if(BrakeFlag==0&&motor_data[0]->ecd<8191&&motor_data[1]->ecd<8191
		&&motor_data[2]->ecd<8191&&motor_data[3]->ecd<8191){
					  BrakeAng[0]=motor_data[0]->ecd+motor_data[0]->circle*8191;
					  BrakeAng[1]=motor_data[1]->ecd+motor_data[1]->circle*8191;
					  BrakeAng[2]=motor_data[2]->ecd+motor_data[2]->circle*8191;
					  BrakeAng[3]=motor_data[3]->ecd+motor_data[3]->circle*8191;
					 BrakeFlag=1;
				 }
				rtU.yaw_target0=BrakeAng[0];
				rtU.yaw_target1=BrakeAng[1];
				rtU.yaw_target2=BrakeAng[2];
				rtU.yaw_target3=BrakeAng[3];
	       }

mult=1;
}

void cala_d(int i){
	
		if((MotorSignal[i].thetan<=180&&MotorSignal[i].thetan>90)||(MotorSignal[i].thetan<-90&&MotorSignal[i].thetan>-180))
				MotorSignal[i].dir=1;
			else
				MotorSignal[i].dir=0;
 if(MotorSignal[i].thetan<-90&&MotorSignal[i].thetan>-180)
		 {
			 MotorSignal[i].thetan=MotorSignal[i].thetan+180;

		 }
		 else if(MotorSignal[i].thetan<=180&&MotorSignal[i].thetan>90)
		 {
			 MotorSignal[i].thetan=MotorSignal[i].thetan-180;

		 }

		 MotorSignal[i].err=MotorSignal[i].thetan-MotorSignal[i].thetal;
      if(MotorSignal[i].thetan-MotorSignal[i].thetal<0)
			{
				MotorSignal[i].err1=MotorSignal[i].err+180;
				MotorSignal[i].err2=MotorSignal[i].err;
				
			}
			else if(MotorSignal[i].thetal-MotorSignal[i].thetan<=0)
			{
				MotorSignal[i].err1=MotorSignal[i].err-180;
				MotorSignal[i].err2=MotorSignal[i].err;
				
			}
			if(fabs(MotorSignal[i].err1)>=fabs(MotorSignal[i].err2)){
				MotorSignal[i].err=MotorSignal[i].err2;
			}
			else{
				MotorSignal[i].err=MotorSignal[i].err1;
        MotorSignal[i].dir=1-MotorSignal[i].dir;
			}
			MotorSignal[i].thetas+=MotorSignal[i].err;
			

			if(((int)MotorSignal[i].thetas%360<=270&&(int)MotorSignal[i].thetas%360>=90)||((int)MotorSignal[i].thetas%360<=-90&&(int)MotorSignal[i].thetas%360>=-270)){
					MotorSignal[i].dir=1-MotorSignal[i].dir;
				}

}




void get_msgn(void)
{
		rtU.yaw_speed_rpm0 = motor_data[0]->speed_rpm;
		rtU.yaw_ecd0       = motor_data[0]->ecd;
		rtU.yaw_last_ecd0  = motor_data[0]->last_ecd;
		rtU.yaw_circle0    = motor_data[0]->circle;
		

		rtU.yaw_speed_rpm1 = motor_data[1]->speed_rpm;
		rtU.yaw_ecd1       = motor_data[1]->ecd;
		rtU.yaw_last_ecd1  = motor_data[1]->last_ecd;
		rtU.yaw_circle1    = motor_data[1]->circle;
	

		rtU.yaw_speed_rpm2 = motor_data[2]->speed_rpm;
		rtU.yaw_ecd2       = motor_data[2]->ecd;
		rtU.yaw_last_ecd2  = motor_data[2]->last_ecd;
		rtU.yaw_circle2    = motor_data[2]->circle;
	

		rtU.yaw_speed_rpm3 = motor_data[3]->speed_rpm;
		rtU.yaw_ecd3       = motor_data[3]->ecd;
		rtU.yaw_last_ecd3  = motor_data[3]->last_ecd;
		rtU.yaw_circle3    = motor_data[3]->circle;
	
		

		rtU.yaw_speed_rpm4 = motor_data[4]->speed_rpm;
		rtU.yaw_ecd4       = motor_data[4]->ecd;
		rtU.yaw_last_ecd4  = motor_data[4]->last_ecd;
		rtU.yaw_circle4    = motor_data[4]->circle;
		

		rtU.yaw_speed_rpm5 = motor_data[5]->speed_rpm;
		rtU.yaw_ecd5       = motor_data[5]->ecd;
		rtU.yaw_last_ecd5  = motor_data[5]->last_ecd;
		rtU.yaw_circle5    = motor_data[5]->circle;
		

		rtU.yaw_speed_rpm6 = motor_data[6]->speed_rpm;
		rtU.yaw_ecd6       = motor_data[6]->ecd;
		rtU.yaw_last_ecd6  = motor_data[6]->last_ecd;
		rtU.yaw_circle6    = motor_data[6]->circle;
		

		rtU.yaw_speed_rpm7 = motor_data[7]->speed_rpm;
		rtU.yaw_ecd7       = motor_data[7]->ecd;
		rtU.yaw_last_ecd7  = motor_data[7]->last_ecd;
		rtU.yaw_circle7    = motor_data[7]->circle;
	
			
		rtU.yaw_speed_rpm8 = motor_data1[0]->speed_rpm;
		rtU.yaw_ecd8       = motor_data1[0]->ecd;
		rtU.yaw_last_ecd8  = motor_data1[0]->last_ecd;
		rtU.yaw_circle8    = motor_data1[0]->circle;
		
		rtU.yaw_speed_rpm9 = motor_data1[1]->speed_rpm;
		rtU.yaw_ecd9       = motor_data1[1]->ecd;
		rtU.yaw_last_ecd9  = motor_data1[1]->last_ecd;
		rtU.yaw_circle9    = motor_data1[1]->circle;
		
		rtU.yaw_speed_rpm10 = motor_data1[2]->speed_rpm;
		rtU.yaw_ecd10       = motor_data1[2]->ecd;
		rtU.yaw_last_ecd10 = motor_data1[2]->last_ecd;
		rtU.yaw_circle10   = motor_data1[2]->circle;
		
		rtU.yaw_speed_rpm11 = motor_data1[3]->speed_rpm;
		rtU.yaw_ecd11       = motor_data1[3]->ecd;
		rtU.yaw_last_ecd11  = motor_data1[3]->last_ecd;
		rtU.yaw_circle11    = motor_data1[3]->circle;
		
		
		rtU.yaw_speed_rpm12 = motor_data1[4]->speed_rpm;
		rtU.yaw_ecd12       = motor_data1[4]->ecd;
		rtU.yaw_last_ecd12  = motor_data1[4]->last_ecd;
		rtU.yaw_circle12    = motor_data1[4]->circle;
		

		rtU.yaw_speed_rpm13 = motor_data1[5]->speed_rpm;
		rtU.yaw_ecd13       = motor_data1[5]->ecd;
		rtU.yaw_last_ecd13  = motor_data1[5]->last_ecd;
		rtU.yaw_circle13    = motor_data1[5]->circle;
		

		rtU.yaw_speed_rpm14 = motor_data1[6]->speed_rpm;
		rtU.yaw_ecd14       = motor_data1[6]->ecd;
		rtU.yaw_last_ecd14  = motor_data1[6]->last_ecd;
		rtU.yaw_circle14    = motor_data1[6]->circle;
		

		rtU.yaw_speed_rpm15 = motor_data1[7]->speed_rpm;
		rtU.yaw_ecd15       = motor_data1[7]->ecd;
		rtU.yaw_last_ecd15  = motor_data1[7]->last_ecd;
		rtU.yaw_circle15    = motor_data1[7]->circle;
	
}
	



void assign_output(void){
	if(rtU.yaw_status0==1)
	   output[0] = rtY.yaw_SPD_OUT0 ; 
	else
		output[0] = rtY.yaw_ANG_OUT0 ; 
	
	if(rtU.yaw_status1==1)
	   output[1] = rtY.yaw_SPD_OUT1 ; 
	else
		output[1] = rtY.yaw_ANG_OUT1 ; 
	
	if(rtU.yaw_status2==1)
	   output[2] = rtY.yaw_SPD_OUT2 ; 
	else
		output[2] = rtY.yaw_ANG_OUT2 ; 
	
	if(rtU.yaw_status3==1)
	   output[3] = rtY.yaw_SPD_OUT3 ; 
	else
		output[3] = rtY.yaw_ANG_OUT3 ; 
	if(rtU.yaw_status4==1)
	   output[4] = rtY.yaw_SPD_OUT4 ; 
	else
		output[4] = rtY.yaw_ANG_OUT4 ; 
		if(rtU.yaw_status5==1)
	   output[5] = rtY.yaw_SPD_OUT5 ; 
	else
		output[5] = rtY.yaw_ANG_OUT5 ; 
			if(rtU.yaw_status6==1)
	   output[6] = rtY.yaw_SPD_OUT6 ; 
	else
		output[6] = rtY.yaw_ANG_OUT6 ; 
			if(rtU.yaw_status7==1)
	   output[7] = rtY.yaw_SPD_OUT7 ; 
	else
		output[7] = rtY.yaw_ANG_OUT7 ; 
			if(rtU.yaw_status8==1)
	   output[8] = rtY.yaw_SPD_OUT8 ; 
	else
		output[8] = rtY.yaw_ANG_OUT8 ; 
			if(rtU.yaw_status9==1)
	   output[9] = rtY.yaw_SPD_OUT9 ; 
	else
		output[9] = rtY.yaw_ANG_OUT9 ; 
			if(rtU.yaw_status10==1)
	   output[10] = rtY.yaw_SPD_OUT10 ; 
	else
		output[10] = rtY.yaw_ANG_OUT10 ; 
			if(rtU.yaw_status11==1)
	   output[11] = rtY.yaw_SPD_OUT11 ; 
	else
		output[11] = rtY.yaw_ANG_OUT11 ; 
			if(rtU.yaw_status12==1)
	   output[12] = rtY.yaw_SPD_OUT12 ; 
	else
		output[12] = rtY.yaw_ANG_OUT12 ; 
			if(rtU.yaw_status13==1)
	   output[13] = rtY.yaw_SPD_OUT13 ; 
	else
		output[13] = rtY.yaw_ANG_OUT13 ; 
			if(rtU.yaw_status14==1)
	   output[14] = rtY.yaw_SPD_OUT14 ; 
	else
		output[14] = rtY.yaw_ANG_OUT14 ; 
			if(rtU.yaw_status15==1)
	   output[15] = rtY.yaw_SPD_OUT15 ; 
	else
		output[15] = rtY.yaw_ANG_OUT15 ; 
	
	
		
		CAN1_cmd_chassis(output[0], output[1],output[2],output[3],output[4],output[5],output[6],output[7]);
		CAN2_cmd_chassis(output[8], output[9],output[10],output[11],output[12],output[13],output[14],output[15]);
	
}


void set_mode(int mode0,int mode1,int mode2,int mode3,int mode4,int mode5,int mode6,int mode7,
	            int mode8,int mode9,int mode10,int mode11,int mode12,int mode13,int mode14,int mode15){
		rtU.yaw_status0    = mode0 ;
		rtU.yaw_status1    = mode1 ;
		rtU.yaw_status2    = mode2 ;
		rtU.yaw_status3    = mode3 ;
		rtU.yaw_status4    = mode4 ;
		rtU.yaw_status5    = mode5      ;
		rtU.yaw_status6    = mode6      ;
		rtU.yaw_status7    = mode7      ;
		rtU.yaw_status8    = mode8      ;// ANG
		rtU.yaw_status9    = mode9      ;// ANG
		rtU.yaw_status10   = mode10     ;// ANG
		rtU.yaw_status11   = mode11      ;// ANG
		rtU.yaw_status12   = mode12     ;
		rtU.yaw_status13   = mode13     ;
		rtU.yaw_status14   = mode14     ;
		rtU.yaw_status15   = mode15      ;
							}

void pid3508(double p, double i, double d) {
		rtP.Kp3508= p ;
		rtP.Ki3508= i;
		rtP.Kd3508= d ;
		
}
void Apid3508(double p, double i, double d) {
		rtP.KAp3508= p ;
		rtP.KAi3508= i;
		rtP.KAd3508= d ;
		
}
							
void pid2006(double p, double i, double d) {
		rtP.Kp2006= p ;
		rtP.Ki2006= i;
		rtP.Kd2006= d ;

}

void Apid2006(double p, double i, double d) {
		rtP.KAp2006= p ;
		rtP.KAi2006= i;
		rtP.KAd2006= d ;
}
