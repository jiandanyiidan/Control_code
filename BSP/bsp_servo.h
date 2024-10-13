#ifndef _BSP_SERVO_H
#define _BSP_SERVO_H

#include "tim.h"

void servo_init(TIM_HandleTypeDef* htim,uint32_t TIM_CHANNEL);
void Servo_Set_Angle(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, float angle);


#endif /* _BSP_SERVO_H */