#ifndef _CLAMP_TASK_H
#define _CLAMP_TASK_H

#include "bsp_servo.h"
#include <stdint.h>

#define Start_angle_1 120
#define Start_angle_2 120

#define ADD_angle 55


#define Clamp_120_angle_1 60
#define Clamp_175_angle_1 155

#define Clamp_120_angle_2 142
#define Clamp_175_angle_2 72
//#define Clamp_120_angle_1 120
//#define Clamp_175_angle_1 175


typedef struct
{
    float angle[2];
    uint8_t flag_task;
	uint8_t last_flag_task;
}Clamp_Task_t;

extern Clamp_Task_t Clamp_Task_1;

void Clamp_task(void  * argument);
void Clamp_Init();
#endif /* _CLAMP_TASK_H */