#include "main.h"
#include "Motor_bsp.h"
#include "bsp_can.h"
#include "chassis_task.h"
Motor_t Motor_2,Motor_3,Motor_1;


static Motor_t* Motor_Init_t(Motor_t *motor,uint8_t ID,float pid[],uint16_t ols_order,uint8_t PID_Improvement);



/*
* @brief 电机初始化
* @param  无
* @retval 无
* @attention 无
用于设置电机初始化，包括电机ID和PID参数的初始化
*/
void Motor_Init(void)
{
    chassis.motor[0].param.Motor_ID=0x201;
	chassis.motor[1].param.Motor_ID=0x202;
	chassis.motor[2].param.Motor_ID=0x203;
	PID_Init(&chassis.motor[0].param.PID, 6000.0, 1000.0,0.0, 3.5f,   0.10f,  0.012f, 0.0, 0.0, 0.0, 0.0,0.0,0x00);
	PID_Init(&chassis.motor[1].param.PID, 6000.0, 1000.0,0.0, 3.7f,   0.02f, 0.020f,  0.0, 0.0, 0.0, 0.0,0.0,0x00);
	PID_Init(&chassis.motor[2].param.PID, 6000.0, 1000.0,0.0, 3.356f, 1.00f,  0.013f, 0.0, 0.0, 0.0, 0.0,0.0,0x00);
	
//    Motor_2.param.Motor_ID = 0x202;
//    PID_Init(&Motor_2.param.PID, 5000.0, 1000.0,0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0x00);
//    Motor_3.param.Motor_ID = 0x203;
//    PID_Init(&Motor_3.param.PID, 5000.0, 1000.0,0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0x00);
//    Motor_1.param.Motor_ID = 0x201;
//    PID_Init(&Motor_1.param.PID, 5000.0, 1000.0,0.0, 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0x00);
}

/*
* @brief 电机初始化
* @param  无
* @retval 无
* @attention 无
用于设置电机初始化，包括电机ID和PID参数的初始化
*/
static Motor_t* Motor_Init_t(Motor_t *motor,uint8_t ID,float pid[],uint16_t ols_order,uint8_t PID_Improvement)
{
	motor->param.Motor_ID=0x200+(uint8_t)ID;
	PID_Init(&motor->param.PID, pid[0], pid[1], pid[2], pid[3], pid[4], pid[5], pid[6], pid[7], pid[8], pid[9],ols_order,PID_Improvement);
	return motor;
}

void Motor_Rx_data(uint8_t data[],CAN_RxHeaderTypeDef *rxFrame)
{
	switch(rxFrame->StdId)
	{
		case CAN_2006Motor1_ID:
		{
			chassis.motor[0].Actual.Actual_angle=(uint16_t)(data[0]<<8)|(data[1]);
			chassis.motor[0].Actual.Actual_speed=(uint16_t)(data[2]<<8)|(data[3]);
			chassis.motor[0].Actual.Actual_torque=(uint16_t)(data[4]<<8)|(data[5]);
			memcpy(&Motor_1,&chassis.motor[0],sizeof(Motor_t));
			break;
		}
		case CAN_2006Motor2_ID:
		{
			chassis.motor[1].Actual.Actual_angle=(data[0]<<8)|(data[1]);
			chassis.motor[1].Actual.Actual_speed=(data[2]<<8)|(data[3]);
			chassis.motor[1].Actual.Actual_torque=(data[4]<<8)|(data[5]);
			memcpy(&Motor_2,&chassis.motor[1],sizeof(Motor_t));
			break;
		}
		case CAN_2006Motor3_ID:
		{
			chassis.motor[2].Actual.Actual_angle=(data[0]<<8)|(data[1]);
			chassis.motor[2].Actual.Actual_speed=(data[2]<<8)|(data[3]);
			chassis.motor[2].Actual.Actual_torque=(data[4]<<8)|(data[5]);
			memcpy(&Motor_3,&chassis.motor[2],sizeof(Motor_t));
			break;
		}
	}
}