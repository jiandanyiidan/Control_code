#include    "bsp_servo.h"
#include <stdint.h>

void servo_init(TIM_HandleTypeDef* htim,uint32_t TIM_CHANNEL)
{
    HAL_TIM_Base_Start(htim);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL);
}

/*
* @brief  设置舵机角度
* @param  tim_pwmHandle: TIM_HandleTypeDef 指针
* @param  Channel: TIM_CHANNEL_x
* @param  angle: 0~180
*/
void Servo_Set_Angle(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, float angle)
{
    uint16_t pwm_value = 0;
    pwm_value = (uint16_t)((angle/180.0f)*2000)+500;
    __HAL_TIM_SetCompare(tim_pwmHandle, Channel, pwm_value);
}