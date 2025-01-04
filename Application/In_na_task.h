#ifndef _IN_NA_TASK_H
#define _IN_NA_TASK_H

#include "ins_task.h"
#include "controller.h"
#include "USB_Commucation.h"
#include "Clamp_task.h"

#define SAFE_D 160
#define SAFE_W 30
#define SAFE_L 60

/*
 目标位置枚举变量部分
 分别位 出发区位置
		中心位置
		安全区位置
		对面出发区位置
		对面安全区位置
*/
typedef enum
{
	Departure_position_task = 0,
	Central_position_task,
	Safe_area_locaton,
	Opposite_departure,
	Opposite_Safe,
	actual_posion,
}Target_posion_t;

/*
	视觉检测任务：
	1.未检测到目标
	2.检测到目标
	3.目标丢失（暂时丢失，识别）
	4.捕获目标
	5.运输至安全区
*/
typedef enum
{
	UNDETECTED = 0,
	DETECTED,
	TEMOIRARY_LOSS,
	PICK_UP,
	TRANSPORT,
	TRAN_OK,
}Task_status_t;


//坐标任务判断是否到达当前位置
typedef enum
{
	TASK_OFF = 0,
	TASK_ON,
}task_switch_t;

typedef enum
{
	OFF,
	ON
}CRUISE_t;

typedef __packed struct
{   
	Target_posion_t target_posion;//目标位置，用来做定位处理
	Task_status_t Task_status; //任务状态，用于视觉配合
	
	task_switch_t task_switch;//任务开启或关闭
	
	float Target_coordinate[2];//目标坐标
	float Actual_coordinate[2];//实际坐标
	
	PID_t Position[3]; //使用PID来进行位置处理，分别为xyz速度，输入角度或者距离，输出速度和角速度
	
	float Target_yaw;//初始化的时候的角度，用于处理判断
	float Actual_yaw;//yaw轴相对角度
	
	float vx;	//x坐标轴方向速度（平面坐标系）
	float vy;	//y坐标轴方向速度（平面坐标系）
	
	float x_out;//输出x轴方向线速度（车坐标系）
	float y_out;//输出y轴方向线速度（车坐标系）
	float z_out;//输出z轴的角速度，顺时正方向（车的速度）
	
	float x_add_speed;
	
	CRUISE_t cruise;
	
	uint32_t tran_ok;
}POSITION_t;


extern POSITION_t position;	
void Posion_init(POSITION_t *posion);
void IN_Task(void  * argument);
void handleUndetected(POSITION_t *posion);
void handleDetected(POSITION_t *posion, Pack_rx_t *pack, float dt);
void handleTemporaryLoss(POSITION_t *posion);
void handlePickUp(POSITION_t *posion, Clamp_Task_t *Clamp_Task);
void handlePickUp(POSITION_t *posion, Clamp_Task_t *Clamp_Task);
void handleTransport(POSITION_t *posion);
void handleTransportOk(POSITION_t *posion, Clamp_Task_t *Clamp_Task);

#endif /* _IN_NA_TASK_H */