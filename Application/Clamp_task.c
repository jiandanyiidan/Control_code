#include "Clamp_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

Clamp_Task_t Clamp_Task_1;
void Clamp_task(void  * argument)
{
	Clamp_Init();
    while(1)
    {
        static uint16_t clamp_time = 0;
        if(clamp_time % 100 ==0)
        {
            Servo_Set_Angle(&htim1,TIM_CHANNEL_1,Clamp_Task_1.angle);
        }
        clamp_time++;
        if(clamp_time == 1000)
        {
            clamp_time = 0;
        }
        vTaskDelay(10);
    }
}


void Clamp_Init()
{
	Clamp_Task_1.angle=18.0f;
	Servo_Set_Angle(&htim1,TIM_CHANNEL_1,Clamp_Task_1.angle);
}