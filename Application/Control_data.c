#include "Control_data.h"
#include <stdint.h>

static void clamp_to_zero(float *value, float threshold);
static bool checkBtnStartChange(uint8_t data, uint8_t last_xbox_data);
/*
 * @brief 数据处理函数
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
	//下列为任务执行或者移动功能
	switch(chassis->mode)
	{
		//无力模式
		case CHASSIS_MODE_POWERLESS:
		{
			chassis->x = 0;
			chassis->y = 0;
			chassis->z = 0;
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
				chassis->z=((float)data->joyRHori-max_data_xbox)/max_data_xbox*max_z_speed;
				clamp_to_zero(&chassis->z,limit_z);
				
				if(data->btnA&0x01&&(data->btnA!=last_xbox_data.btnA))
				{
					Clamp_Task->flag_task = !Clamp_Task->flag_task;
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
			chassis->x = pack_rx->speed;
			chassis->z = pack_rx->yaw_add*0.01f;
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


/*
 * @brief 检测手柄启动按键是否改变
 * @param  data:遥控器数据
 * @param  last_xbox_data:上一次的遥控器数据
*/
bool checkBtnStartChange(uint8_t data, uint8_t last_xbox_data) 
{
    return (data & 0x01) && (data != last_xbox_data);
}
