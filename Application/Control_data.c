#include "Control_data.h"
#include <stdint.h>
#include "In_na_task.h"


static void clamp_to_zero(float *value, float threshold);
static bool checkBtnStartChange(uint8_t data, uint8_t last_xbox_data);
/*
 * @brief 数据处理函数,暂时放到这里，在写新的任务函数，将任务模块化
 * @param  data:遥控器数据
 * @param  chassis:底盘数据
 * @retval None
*/
void Rc_task(xbox_data_t *data,Chassis_t *chassis,Pack_rx_t *pack_rx
			 ,Clamp_Task_t *Clamp_Task)
{
	//遥控器执行运动模式切换
	if(checkBtnStartChange(data->btnStart,last_xbox_data.btnStart))
	{
		chassis->mode++;
		if(chassis->mode>CHASSIS_MODE_VISION)
		{
			chassis->mode = CHASSIS_MODE_POWERLESS;
		}
	}
	if(checkBtnStartChange(data->btnX,last_xbox_data.btnX))
	{
		if(chassis->mode == CHASSIS_MODE_POWERLESS || chassis->mode == CHASSIS_MODE_XBOX)
		{
			chassis->mode=CHASSIS_MODE_VISION;
		}
		else if(chassis->mode==CHASSIS_MODE_VISION)
		{
			chassis->mode=CHASSIS_MODE_XBOX;
		}
	}
	if(checkBtnStartChange(data->btnSelect,last_xbox_data.btnSelect))
	{
		chassis->mode=CHASSIS_MODE_POWERLESS;
//		Clamp_Init();
//		Posion_init(&position);
//		Motor_Init();
//		chassis->x_y[0]=0;
//		chassis->x_y[1]=0;
	}
	//下列为任务执行或者移动功能
	switch(chassis->mode)
	{
		//无力模式
		case CHASSIS_MODE_POWERLESS:
		{
			chassis->x = 0;
			chassis->y = 0;
			chassis->z = 0;
			chassis->Target_angle=chassis->Actual_angle;
			break;
		}
		//手柄模式
		case CHASSIS_MODE_XBOX:
		{
			if(chassis->xbox_status==XBOX_LINK)
			{
				chassis->x=(max_data_xbox-(float)data->joyLVert)/max_data_xbox*max_x_speed;
				clamp_to_zero(&chassis->x,limit_x);
				chassis->y=((float)data->joyLHori-max_data_xbox)/max_data_xbox*max_y_speed;
				clamp_to_zero(&chassis->y,limit_y);
				
				chassis->Add_angle=((float)data->joyRHori-max_data_xbox)/max_data_xbox*max_z_speed/2.0;
				clamp_to_zero(&chassis->Add_angle,limit_z);
				chassis->Target_angle+=chassis->Add_angle;
				if(chassis->Target_angle-chassis->Actual_angle>180)
					chassis->Target_angle -= 360;
				if(chassis->Target_angle-chassis->Actual_angle<-180)
					chassis->Target_angle += 360;
				chassis->z=PID_Calculate(&chassis->yaw_pid,chassis->Actual_angle,chassis->Target_angle);
//				chassis->z=((float)data->joyRHori-max_data_xbox)/max_data_xbox*max_z_speed;
//				clamp_to_zero(&chassis->z,limit_z);
				
				if(data->btnA&0x01&&(data->btnA!=last_xbox_data.btnA))
				{
					Clamp_Task->flag_task = !Clamp_Task->flag_task;
				}
				if(data->btnDirUp&0x01)
				{
					chassis->x=100.0f;
				}
				else if(data->btnDirUp&0x00)
				{
					chassis->x=0.0f;
				}
				if(data->btnDirDown&0x01)
				{
					chassis->x=-100.0f;
				}
				else if(data->btnDirDown&0x00)
				{
					chassis->x=0.0f;
				}
				if(data->btnDirRight&0x01)
				{
					chassis->y=100.0f;
				}
				else if(data->btnDirRight&0x00)
				{
					chassis->y=0.0f;
				}
				if(data->btnDirLeft&0x01)
				{
					chassis->y=-100.0f;
				}
				else if(data->btnDirLeft&0x00)
				{
					chassis->y=0.0f;
				}
			}
			else if(chassis->xbox_status==XBOX_UNLINK)
			{
				chassis->x = 0;
				chassis->y = 0;
				chassis->z = 0;
			}
			break;
		}
		//视觉模式
		case CHASSIS_MODE_VISION:
		{
			chassis->x = position.x_out;
			chassis->y = position.y_out;
			chassis->z = position.z_out;
			chassis->Target_angle=position.Target_yaw;
		    break;
		}
		default:
		{
			break;
		}
	}
}

/**
 * @brief limit the value to zero if it is below the threshold
 * @param  value: the value to be limited
 * @param  threshold: the threshold
 * @retval None
*/
void clamp_to_zero(float *value, float threshold)
{
    if (fabs(*value) < threshold) {
        *value = 0.0;
    }
}


/**
 * @brief 检测手柄启动按键是否改变
 * @param  data:遥控器数据
 * @param  last_xbox_data:上一次的遥控器数据
*/
bool checkBtnStartChange(uint8_t data, uint8_t last_xbox_data) 
{
    return (data & 0x01) && (data != last_xbox_data);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_0)
	{
		if(chassis.mode == CHASSIS_MODE_POWERLESS || chassis.mode == CHASSIS_MODE_XBOX)
		{
			chassis.mode=CHASSIS_MODE_VISION;
		}
		else if(chassis.mode==CHASSIS_MODE_VISION)
		{
			chassis.mode=CHASSIS_MODE_XBOX;
		}
	}
}
