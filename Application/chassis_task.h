#ifndef  CHASSIS_TASK_H
#define   CHASSIS_TASK_H

#include "main.h"
#include "Motor_bsp.h"
#include "xbox_rx_task.h"

#define MAX_MOTOR_SPEED   14336				//������ת�٣��궨�巽���޸�   ��Χ0 - 10000   15336   
#define VEL2RPM 2.48033f										//���ٶ�תת��
#define M2006_REDUCTION_RATIO 36.000000f		//��������ٱ�
#define RPM2VEL 0.403171f										//ת��ת���ٶ�		vel = rpn*pi*D/60  cm/s
#define MAX_D_SPEED_X       1.5f    // ��� x �����ٶȱ仯��
#define MAX_D_SPEED_Y       1.5f    // ��� y �����ٶȱ仯��
#define MAX_D_SPEED_X_STOP  5.0f   // ��� x ����ֹͣ�ٶȱ仯��
#define MAX_D_SPEED_Y_STOP  5.0f   // ��� y ����ֹͣ�ٶȱ仯��

typedef enum
{
    CHASSIS_MODE_POWERLESS = 0,//无力模式
    CHASSIS_MODE_XBOX = 1,//手柄模式
    CHASSIS_MODE_VISION = 2//视觉模式
}Chassis_Mode_t;

typedef struct
{
	Chassis_Mode_t mode;//控制模式
	xbox_link_status_t xbox_status;	//手柄状态

    float x;
    float y;
    float z;
	
	Motor_t motor[3];
	uint8_t tx_data_text[8];
}Chassis_t;

typedef __packed struct
{
	float linear_vel;			//���ٶ�
	float rpm;					//ת��Ȧÿ����
}Speed_t;

typedef __packed struct
{
	Speed_t target_speed;			
	Speed_t actual_speed;						
}Wheel_t;

//���̼������ĵ���/���ٶ�
typedef __packed struct
{
	float linear_x;	//m/s
	float linear_y;
	float angular_z; //���ٶ�rpm
}Velocities_t;

typedef __packed struct
{
	Wheel_t wheel1;
	Wheel_t wheel2;
	Wheel_t wheel3;
	
	Velocities_t target_velocities;		//Ŀ�����ٶ�
	Velocities_t actual_velocities; 	//ʵ�����ٶ�
}Kinematics_t;


extern Chassis_t chassis;

static void chassis_speed_control(Chassis_t *chassis);
static void Speed_optimize(float vx_desired, float vy_desired, float *vx_optimized, float *vy_optimized);
void Chassis_Task(void  * argument);
int find_max(void);
void current_task(uint8_t data[]);
void Chassis_PID();
void BaseVel_To_WheelVel(Chassis_t *chassis);
void Get_Base_Velocities(void  * argument);
#endif
