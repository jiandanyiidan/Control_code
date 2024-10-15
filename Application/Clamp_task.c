#include "Clamp_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

Clamp_Task_t Clamp_Task_1;

static uint8_t  Clamp_angle_task(Clamp_Task_t *clamp);

void Clamp_task(void  * argument)
{
	Clamp_Init();
    while(1)
    {
        static uint16_t clamp_time = 0;
        Clamp_angle_task(&Clamp_Task_1);
        if(clamp_time % 50 ==0)
        {
            Servo_Set_Angle(&htim1,TIM_CHANNEL_1,Clamp_Task_1.angle);
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
	Clamp_Task_1.angle=18.0f;
	Servo_Set_Angle(&htim1,TIM_CHANNEL_1,Clamp_Task_1.angle);
}

static uint8_t  Clamp_angle_task(Clamp_Task_t *clamp)
{
	if(clamp->flag_task==1)
	{
		clamp->angle=Clamp_Open_angle;
	}
	else if(clamp->flag_task==0)
	{
		clamp->angle=Clamp_Close_angle;
	}
}