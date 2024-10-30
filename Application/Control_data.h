#ifndef _CONTROL_DATA_H
#define _CONTROL_DATA_H

#include "xbox_rx_task.h"
#include "Clamp_Task.h"
#include "chassis_task.h"
#include "USB_Commucation.h"
//数据处理限制
#define limit_x 10.0f
#define limit_y 10.0f
#define limit_z 0.2f

#define max_data_xbox 32767.0f
#define max_x_speed 100.0f
#define max_y_speed 100.0f
#define max_z_speed 1.0f

void Rc_task(xbox_data_t *data,Chassis_t *chassis,Pack_rx_t *pack_rx,Clamp_Task_t *Clamp_Task);
#endif // _CONTROL_DATA_H
