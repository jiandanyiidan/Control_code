#ifndef _MOTOR_BSP_H_
#define _MOTOR_BSP_H_
#include "main.h"
#include <stdint.h>
#include "controller.h"

typedef __packed  struct
{
    uint32_t Motor_ID;
    uint8_t Motor_temp;
    PID_t PID;
} paramenter_t;

typedef __packed struct
{
    int16_t Target_speed;
    int16_t Target_angle;
    int16_t Target_current;
}target_t;

typedef __packed struct
{
    int16_t Actual_speed;
    int16_t Actual_angle;
    int16_t Actual_torque;
}actual_t;
typedef __packed struct
{
    paramenter_t param;
    target_t Target;
    actual_t Actual;
    int16_t Error[3];
}Motor_t;

void Motor_Init(void);

void Motor_Rx_data(uint8_t data[],CAN_RxHeaderTypeDef *rxFrame);
//extern Motor_t Motor_1, Motor_2,Motor_3;
#endif 
