#include "chassis_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "Motor_bsp.h"
#include "USB_Commucation.h"

Chassis_t chassis;
Kinematics_t Kinematics;

void Chassis_Task(void  * argument)
{
	chassis.mode=CHASSIS_MODE_POWERLESS;
	Motor_Init();
	while(1)
	{
		USB_TX();
		Speed_optimize(chassis.x,chassis.y,&chassis.x,&chassis.y);
		chassis_speed_control(&chassis);
		Chassis_PID();
		platform_trans(&hcan1,chassis.tx_data_text);
		vTaskDelay(20);
	}
}

/**
  * @breif         �˶�ѧ�ֽ⣬���������ĵ��ٶ�ת��Ϊ�ĸ����ӵ��ٶ�
  * @param[in]     speed_x��x�����ٶ�
	* @param[in]     speed_y��y�����ٶ�
	* @param[in]     speed_r����ת�ٶ�
	* @param[out]    �ĸ������Ŀ���ٶ�
  * @retval        none     
  */
static void chassis_speed_control(Chassis_t *chassis)
{
	int max;
		//�ٶȻ��㣬�˶�ѧ�ֽ�
	BaseVel_To_WheelVel(chassis);
	
	//Ӧ��û���ã���ʱ��ע��
	max=find_max();
	if(max>MAX_MOTOR_SPEED)
	{
		chassis->motor[0].Target.Target_speed=(int)(chassis->motor[0].Target.Target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis->motor[1].Target.Target_speed=(int)(chassis->motor[1].Target.Target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis->motor[2].Target.Target_speed=(int)(chassis->motor[2].Target.Target_speed*MAX_MOTOR_SPEED*1.0/max);
	}
}	

void Chassis_PID()
{
	chassis.motor[0].Target.Target_current=PID_Calculate(&chassis.motor[0].param.PID,chassis.motor[0].Actual.Actual_speed,chassis.motor[0].Target.Target_speed);
	chassis.motor[1].Target.Target_current=PID_Calculate(&chassis.motor[1].param.PID,chassis.motor[1].Actual.Actual_speed,chassis.motor[1].Target.Target_speed);
	chassis.motor[2].Target.Target_current=PID_Calculate(&chassis.motor[2].param.PID,chassis.motor[2].Actual.Actual_speed,chassis.motor[2].Target.Target_speed);
	current_task(chassis.tx_data_text);
}

void current_task(uint8_t data[])
{
	data[0]=chassis.motor[0].Target.Target_current>>8;
	data[1]=chassis.motor[0].Target.Target_current;
	data[2]=chassis.motor[1].Target.Target_current>>8;
	data[3]=chassis.motor[1].Target.Target_current;
	data[4]=chassis.motor[2].Target.Target_current>>8;
	data[5]=chassis.motor[2].Target.Target_current;
}
void BaseVel_To_WheelVel(Chassis_t *chassis)
{
	float vz;

	Kinematics.wheel1.target_speed.linear_vel=(chassis->z*15.0f)-chassis->y;
	Kinematics.wheel2.target_speed.linear_vel=(chassis->z*15.0f)+chassis->y*cos(3.14f/3)+chassis->x*sin(3.14f/3);
	Kinematics.wheel3.target_speed.linear_vel=(chassis->z*15.0f)+chassis->y*cos(3.14f/3)-chassis->x*sin(3.14f/3);
		
	//���ٶ� cm/s  תת��  RPM 
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel3.target_speed.rpm = Kinematics.wheel3.target_speed.linear_vel * VEL2RPM;
		
	chassis->motor[0].Target.Target_speed = (int)(Kinematics.wheel1.target_speed.rpm * M2006_REDUCTION_RATIO);
	chassis->motor[1].Target.Target_speed = (int)(Kinematics.wheel2.target_speed.rpm * M2006_REDUCTION_RATIO);
	chassis->motor[2].Target.Target_speed = (int)(Kinematics.wheel3.target_speed.rpm * M2006_REDUCTION_RATIO);	
}
int find_max(void)
{
  int temp=0;
  
  temp=abs(chassis.motor[0].Target.Target_speed);
  if(abs(chassis.motor[1].Target.Target_speed)>temp)
    temp=abs(chassis.motor[1].Target.Target_speed);
  if(abs(chassis.motor[2].Target.Target_speed)>temp)
    temp=abs(chassis.motor[2].Target.Target_speed);
  return temp;
}

/**
 * @brief   �����ٶ��Ż���ʹ�ٶȱ仯��ƽ��
 * @param[in]  vx_desired: ������ x �����ٶ�
 * @param[in]  vy_desired: ������ y �����ٶ�
 * @param[out] vx_optimized: �Ż���� x �����ٶ�
 * @param[out] vy_optimized: �Ż���� y �����ٶ�
 */
float last_vx,last_vy;
static void Speed_optimize(float vx_desired, float vy_desired, float *vx_optimized, float *vy_optimized)
{
    // �ٶ��Ż�
    if (vx_desired != 0.0f || vy_desired != 0.0f)
    {
        if (fabsf(vx_desired - last_vx) > MAX_D_SPEED_X)
        {
            *vx_optimized = last_vx + (vx_desired > last_vx ? MAX_D_SPEED_X : -MAX_D_SPEED_X);
        }
        else
        {
            *vx_optimized = vx_desired;
        }

        if (fabsf(vy_desired - last_vy) > MAX_D_SPEED_Y)
        {
            *vy_optimized = last_vy + (vy_desired > last_vy ? MAX_D_SPEED_Y : -MAX_D_SPEED_Y);
        }
        else
        {
            *vy_optimized = vy_desired;
        }

        last_vx = *vx_optimized;
        last_vy = *vy_optimized;
    }
    else
    {
        // ֹͣ�ٶ��Ż�
        if (last_vx > 0.0f)
        {
            last_vx -= MAX_D_SPEED_X_STOP;
            last_vx = (last_vx < 0.0f) ? 0.0f : last_vx;
        }
        else if (last_vx < 0.0f)
        {
            last_vx += MAX_D_SPEED_X_STOP;
            last_vx = (last_vx > 0.0f) ? 0.0f : last_vx;
        }

        if (last_vy > 0.0f)
        {
            last_vy -= MAX_D_SPEED_Y_STOP;
            last_vy = (last_vy < 0.0f) ? 0.0f : last_vy;
        }
        else if (last_vy < 0.0f)
        {
            last_vy += MAX_D_SPEED_Y_STOP;
            last_vy = (last_vy > 0.0f) ? 0.0f : last_vy;
        }

        *vx_optimized = last_vx;
        *vy_optimized = last_vy;
    }
}

//���˶�ѧ��ʽ
//ͨ����̥��ʵ��ת�ټ�����̼������ĵ������ٶ�
void Get_Base_Velocities(void  * argument)
{
	while(1)
	{
		//���ݵ��ת�ٲ�������ת��
		Kinematics.wheel1.actual_speed.rpm =  (float)chassis.motor[0].Actual.Actual_speed / M2006_REDUCTION_RATIO;
		Kinematics.wheel2.actual_speed.rpm =  -(float)chassis.motor[1].Actual.Actual_speed / M2006_REDUCTION_RATIO;
		Kinematics.wheel3.actual_speed.rpm =   (float)chassis.motor[2].Actual.Actual_speed / M2006_REDUCTION_RATIO;
		//����ת��ת��Ϊ�������ٶ�
		Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
		Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
		Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
		
		//�������ٶ�ת��Ϊ��������������ٶ�
		Kinematics.actual_velocities.angular_z = -( Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel\
					+ Kinematics.wheel3.actual_speed.linear_vel )/(4.0f*15.0f);
		Kinematics.actual_velocities.linear_x  = (Kinematics.wheel2.actual_speed.linear_vel*sin(3.14/3)+Kinematics.wheel2.actual_speed.linear_vel*sin(3.14/3))/(2.0f);
		Kinematics.actual_velocities.linear_y  = -( Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel*sin(3.14/6)\
					- Kinematics.wheel3.actual_speed.linear_vel*sin(3.14/6))/(4.0f);
		vTaskDelay(13);
	}
}