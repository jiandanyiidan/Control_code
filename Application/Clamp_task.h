#ifndef _CLAMP_TASK_H
#define _CLAMP_TASK_H

#include "bsp_servo.h"
#include <stdint.h>

#define Clamp_Open_angle 90
#define Clamp_Close_angle 18


typedef struct
{
    float angle;
    uint8_t flag_task;
}Clamp_Task_t;

extern Clamp_Task_t Clamp_Task_1;

void Clamp_task(void  * argument);
void Clamp_Init();
#endif /* _CLAMP_TASK_H */