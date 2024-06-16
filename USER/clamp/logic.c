#include "logic.h"
#include "bsp_can.h"
#include "arm_math.h"
#include "tim.h"

// 目标角度监测
void target_monitor(void)
{
	// 2006
	// 当前角度
	current_target[M_2006] = (motor_data1[M_2006]->circle) * 8191 + (motor_data1[M_2006]->ecd);
	// 角度误差
	target_error[M_2006] = (current_target[M_2006] - ANG_TGT[M_2006]) > 0 ? (current_target[M_2006] - ANG_TGT[M_2006]) : 0;
	
	// 判断动作是否完成
	if (target_error[M_2006] < 2000 && move_FLAG[M_2006] == MOVE)
	{
		move_FLAG[M_2006] = FINISH;
	}

	// 3508
	// 当前角度
	current_target[M_3508] = (motor_data1[M_3508]->circle) * 8191 + (motor_data1[M_3508]->ecd);
	current_ang = current_target[M_3508] * (187 * 360) / (3591 * 8191);
	// 角度误差
	target_error[M_3508] = (current_target[M_3508] - ANG_TGT[M_3508]) > 0 ? (current_target[M_3508] - ANG_TGT[M_3508]) : 0;
	// 判断动作是否完成
	if (target_error[M_3508] < 7000 && move_FLAG[M_3508] == MOVE)
	{
		move_FLAG[M_3508] = FINISH;
	}
}

// 夹
void LOGIC(void)
{
	switch (current_state)
	{
		// S_0 初始位置0
	case S_0:
		// 双电机运动完成
		if (move_FLAG[M_2006] == FINISH && move_FLAG[M_3508] == FINISH)
		{
			// 舵机动作完成
			if (servo_FLAG == FINISH)
			{
				// 苗数量小于5时，允许执行夹操作
				if (logic_change == PINCH && num_state < 3)
				{
					current_state = S_1;
					num_state++;
					Logic_FLAG = PINCH;
					logic_change = FREE;
					servo_FLAG = FREE;
				} // 苗数量大于0时，允许执行放操作
				else if (logic_change == PLACE && num_state > 0)
				{
					current_state = S_3;
					num_state--;
					Logic_FLAG = PLACE;
					logic_change = FREE;
					servo_FLAG = FREE;
				}
				else
				{
					return;
				}
				// 重置电机状态为FREE
				move_FLAG[M_2006] = FREE;
				move_FLAG[M_3508] = FREE;
			}
			else if (servo_FLAG != FINISH)
			{
				HAL_Delay(Delay_Time);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_LO); // 舵机松开
				HAL_Delay(Delay_Time);
				servo_FLAG = FINISH;
			}
		}
		else
		{
			// 移动2006
			if (move_FLAG[M_2006] == FREE)
			{
				HAL_Delay(Delay_Time);
				YAW_TGT[M_2006] = 29.98;
				move_FLAG[M_2006] = MOVE;
				HAL_Delay(Delay_Time);
			}
			// 移动3508
			else if (move_FLAG[M_2006] == FINISH && move_FLAG[M_3508] == FREE)
			{
				HAL_Delay(Delay_Time);
				YAW_TGT[M_3508] = 83;
				move_FLAG[M_3508] = MOVE;
			}
		}
		break;
		// S_1 3508-100mm（夹取位置1，放置位置2）
	case S_1:
		// 选择对应的苗仓库
		switch (num_state)
		{
			//				case 1:
			//					TEMP_YAW_TGT_2006=H_1;
			//					break;
			//				case 2:
			//					TEMP_YAW_TGT_2006=H_2;
			//					break;
		case 1:
			TEMP_YAW_TGT_2006 = H_3;
			break;
		case 2:
			TEMP_YAW_TGT_2006 = H_4;
			break;
		case 3:
			TEMP_YAW_TGT_2006 = H_5;
			break;
		}
		// 如果已经在S_1位置，转到S_2状态
		if (move_FLAG[M_2006] == FINISH && move_FLAG[M_3508] == FINISH)
		{
			if (servo_FLAG == FINISH)
			{
				// S_1作为夹取和放置的中转状态，需要通过当前逻辑判断去S_2或S_0
				current_state = (Logic_FLAG == PINCH) ? S_2 : S_0;
				// 重置电机状态和舵机状态为FREE
				move_FLAG[M_2006] = FREE;
				move_FLAG[M_3508] = FREE;
				servo_FLAG = FREE;
			}
			else if (Logic_FLAG == PINCH)
			{
				HAL_Delay(Delay_Time);
				servo_FLAG = FINISH;
			}
			else if (Logic_FLAG == PLACE)
			{
				HAL_Delay(Delay_Time);
				servo_FLAG = FINISH;
			}
		}
		else
		{
			// 移动3508
			if (Logic_FLAG == PINCH)
			{
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_CA); // 舵机夹紧
				HAL_Delay(Delay_Time);
			}
			if (move_FLAG[M_3508] == FREE)
			{
				HAL_Delay(Delay_Time);
				YAW_TGT[M_3508] = S_1_3508;
				move_FLAG[M_3508] = MOVE;
			}
			// 移动2006
			else if (move_FLAG[M_3508] == FINISH && move_FLAG[M_2006] == FREE && Logic_FLAG == PINCH)
			{
				HAL_Delay(Delay_Time);
				YAW_TGT[M_2006] = TEMP_YAW_TGT_2006;
				move_FLAG[M_2006] = MOVE;
			}
			else if (move_FLAG[M_3508] == FINISH && move_FLAG[M_2006] == FREE && Logic_FLAG == PLACE)
			{
				HAL_Delay(Delay_Time);
				move_FLAG[M_2006] = FINISH;
			}
		}
		break;
		// S_2 3508-90mm （夹取位置2）
	case S_2:
		// 如果已经在DI_2位置，转到D_0状态
		if (move_FLAG[M_2006] == FINISH && move_FLAG[M_3508] == FINISH)
		{
			if (servo_FLAG == FINISH)
			{
				current_state = S_0;
				// 重置电机状态和舵机状态为FREE
				move_FLAG[M_2006] = FREE;
				move_FLAG[M_3508] = FREE;
				servo_FLAG = FREE;
			}
			else if (Logic_FLAG == PINCH)
			{
				//HAL_Delay(Delay_Time);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_LO); // 舵机松开
				HAL_Delay(Delay_Time);
				servo_FLAG = FINISH;
			}
		}
		else
		{
			// 移动3508
			if (move_FLAG[M_3508] == FREE)
			{
				HAL_Delay(Delay_Time);
				YAW_TGT[M_3508] = S_2_3508;
				move_FLAG[M_3508] = MOVE;
			}
			// 2006不动
			if (move_FLAG[M_3508] == FINISH && move_FLAG[M_2006] == FREE && Logic_FLAG == PINCH)
			{
				HAL_Delay(Delay_Time);
				move_FLAG[M_2006] = FINISH;
			}
		}
		break;

		// S_3 3508-50mm （放置位置1）
	case S_3:
		switch (num_state)
		{
			//			case 0:
			//				TEMP_YAW_TGT_2006=H_1;
			//				break;
			//			case 1:
			//				TEMP_YAW_TGT_2006=H_2;
			//				break;
		case 0:
			TEMP_YAW_TGT_2006 = H_3;
			break;
		case 1:
			TEMP_YAW_TGT_2006 = H_4;
			break;
		case 2:
			TEMP_YAW_TGT_2006 = H_5;
			break;
		}
		// 如果已经在S_3位置，转到S_1状态
		if (move_FLAG[M_2006] == FINISH && move_FLAG[M_3508] == FINISH)
		{
			if (servo_FLAG == FINISH)
			{
				current_state = S_1;
				// 重置电机状态和舵机状态为FREE
				move_FLAG[M_2006] = FREE;
				move_FLAG[M_3508] = FREE;
				servo_FLAG = FREE;
			}
			else if (Logic_FLAG == PLACE)
			{
				HAL_Delay(Delay_Time);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_CA); // 舵机夹紧
				HAL_Delay(Delay_Time);
				servo_FLAG = FINISH;
			}
		}
		else
		{
			if (Logic_FLAG == PLACE)
			{
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo_LO); // 舵机松开
				HAL_Delay(Delay_Time);
			} // 移动3508
			if (move_FLAG[M_3508] == FREE)
			{
				HAL_Delay(Delay_Time);
				YAW_TGT[M_3508] = S_3_3508;
				move_FLAG[M_3508] = MOVE;
			} // 移动2006
			else if (move_FLAG[M_3508] == FINISH && move_FLAG[M_2006] == FREE)
			{
				HAL_Delay(Delay_Time);
				YAW_TGT[M_2006] = TEMP_YAW_TGT_2006;
				move_FLAG[M_2006] = MOVE;
			}
		}
		break;
	}
}

/*
上电初始化部分
（待调试，现已不用）
*/

// void error_count_2006()
//{
//		//2006
//		if (stl_counter[M_2006] == 1) {
//				STL_BGN_ANG[M_2006] = (motor_data[M_2006]->circle)*8191+motor_data[M_2006]->ecd;
//		}
//		else if (stl_counter[M_2006] == 100) {
//				STL_END_ANG[M_2006] = (motor_data[M_2006]->circle)*8191+motor_data[M_2006]->ecd;

//				stl_error[M_2006] = fabs(STL_END_ANG[M_2006] - STL_BGN_ANG[M_2006]);
//				if(stl_error[M_2006]<100)
//				{
//					int ang=(motor_data[M_2006]->circle)*8191+(motor_data[M_2006]->ecd);
//					ZERO_POINT[M_2006]=ang*(17*360)/(36*8191*35);
//					rtP.Kp_s/=2;
//					rtP.Ki_s/=10;
//					motor_state[M_2006]=2;
//					YAW_TGT[M_2006]=10+ZERO_POINT[M_2006];
//					init_FLAG[M_2006]=1;
//				}
//				stl_counter[M_2006] = 0;
//		}
//		stl_counter[M_2006]++;

//}

// void error_count_3508()
//{
//	//3508
//		if (stl_counter[M_3508] == 1) {
//				STL_BGN_ANG[M_3508] = (motor_data[M_3508]->circle)*8191+motor_data[M_3508]->ecd;
//		}
//		else if (stl_counter[M_3508] == 100) {
//				STL_END_ANG[M_3508] = (motor_data[M_3508]->circle)*8191+motor_data[M_3508]->ecd;

//				stl_error[M_3508] = fabs(STL_END_ANG[M_3508] - STL_BGN_ANG[M_3508]);
//				if(stl_error[M_3508]<300)
//				{
//					int ang=(motor_data[M_3508]->circle)*8191+(motor_data[M_3508]->ecd);
//					ZERO_POINT[M_3508]=ang*(17*360)/(36*8191*35);
//					rtP.Kp_s_1/=4;
//					rtP.Ki_s_1/=2;
//					motor_state[M_3508]=2;
//					YAW_TGT[M_3508]=-180+ZERO_POINT[M_3508];
//					init_FLAG[M_3508]=1;
//				}
//				stl_counter[M_3508] = 0;
//		}
//		stl_counter[M_3508]++;
//}

// void init_jaw()
//{
//	rtP.Kp_s*=2;
//	rtP.Ki_s*=10;
//     motor_state[M_2006]=1;
//	SPD_TGT[M_2006] = -500;
//	HAL_Delay(500);
//	while(init_FLAG[M_2006]==0)
//	{
//		error_count_2006();
//		HAL_Delay(1);
//	}
//	rtP.Kp_s_1*=4;
//	rtP.Ki_s_1*=2;
//     motor_state[M_3508]=1;
//	SPD_TGT[M_3508] = 1000;
//	HAL_Delay(500);
//	while(init_FLAG[M_3508]==0)
//	{
//		error_count_3508();
//		HAL_Delay(1);
//	}
//
// }
