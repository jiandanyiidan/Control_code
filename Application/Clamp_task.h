#ifndef _CLAMP_TASK_H
#define _CLAMP_TASK_H

#include "bsp_servo.h"
#include <stdint.h>

typedef struct
{
    float angle;
    uint8_t flag_task;
}Clamp_Task_t;

void Clamp_task(void  * argument);
void Clamp_Init();
#endif /* _CLAMP_TASK_H */