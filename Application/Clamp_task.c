#include "Clamp_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

Clamp_Task_t Clamp_Task_1;

static uint8_t  Clamp_angle_task(Clamp_Task_t *clamp);
/*
 * @brief  ƹ
 * @param  None
 * @retval None
 * @note   None
 * @author ־zwx
*/

void Clamp_task(void  * argument)
{
	Clamp_Init();
    while(1)
    {
        static uint16_t clamp_time = 0;
        Servo_Set_Angle(&htim1,TIM_CHANNEL_1,Clamp_Task_1.angle[0]);
		Servo_Set_Angle(&htim1,TIM_CHANNEL_2,Clamp_Task_1.angle[1]);
        if(clamp_time % 5 ==0)
        {
            
			Clamp_angle_task(&Clamp_Task_1);
        }
        clamp_time++;
        if(clamp_time == 1000)
        {
            clamp_time = 0;
        }
        vTaskDelay(20);
    }
}


void Clamp_Init()
{
	Clamp_Task_1.angle[0]=Clamp_120_angle_1;
	Servo_Set_Angle(&htim1,TIM_CHANNEL_1,Clamp_Task_1.angle[0]);
	Clamp_Task_1.angle[1]=Clamp_175_angle_1;
	Servo_Set_Angle(&htim1,TIM_CHANNEL_2,Clamp_Task_1.angle[1]);
}

static uint8_t Clamp_angle_task(Clamp_Task_t *clamp)
{
	 // 检查 flag_task 是否改变
    if (clamp->flag_task != clamp->last_flag_task) 
	{
        if (clamp->flag_task == 1) 
		{
            clamp->angle[0] = Clamp_120_angle_2;
			clamp->angle[1] = Clamp_175_angle_2;
        } else if (clamp->flag_task == 0) 
		{
            clamp->angle[0] = Clamp_120_angle_1;
			clamp->angle[1] = Clamp_175_angle_1;
        }
        // 更新 last_flag_task 为当前 flag_task
        clamp->last_flag_task = clamp->flag_task;
    }
}