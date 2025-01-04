#include "chassis_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "Motor_bsp.h"
#include "USB_Commucation.h"
#include "bsp_dwt.h"
#include "ins_task.h"



Chassis_t chassis;
Kinematics_t Kinematics;

void Chassis_Task(void  * argument)
{
	chassis.mode=CHASSIS_MODE_POWERLESS;
	Motor_Init();
	while(1)
	{
		Speed_optimize(chassis.x,chassis.y,&chassis.x,&chassis.y);
		chassis_speed_control(&chassis);
		Chassis_PID();
		platform_trans(&hcan1,chassis.tx_data_text);
		vTaskDelay(20);
	}
}

/**
  * @breif         运动学分解，将底盘中心的速度转换为四个轮子的速度
  * @param[in]     speed_x：x方向速度
	* @param[in]     speed_y：y方向速度
	* @param[in]     speed_r：自转速度
	* @param[out]    四个电机的目标速度
  * @retval        none     
  */
static void chassis_speed_control(Chassis_t *chassis)
{
	int max;
	
	
	//速度换算，运动学分解
	BaseVel_To_WheelVel(chassis);
	
	//应该没有用，暂时不注释
	max=find_max();
	if(max>MAX_MOTOR_SPEED)
	{
		chassis->motor[0].Target.Target_speed=(int)(chassis->motor[0].Target.Target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis->motor[1].Target.Target_speed=(int)(chassis->motor[1].Target.Target_speed*MAX_MOTOR_SPEED*1.0/max);
		chassis->motor[2].Target.Target_speed=(int)(chassis->motor[2].Target.Target_speed*MAX_MOTOR_SPEED*1.0/max);
	}
}	

/**
 * 将角度转换为弧度。
 * 
 * 此函数接受一个角度值（以度为单位），然后将其转换为弧度值。
 * 
 * @param degrees 输入的角度值，单位为度。
 * @return 计算得到的弧度值。
 * 
 * 示例：
 * deg2rad(90) 返回 1.5707963267948966
 */
float deg2rad(float degrees) 
{
    return (degrees * PI)/180.0;
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
	
	Kinematics.target_velocities.linear_x=chassis->x;
	Kinematics.target_velocities.linear_y=chassis->y;
	Kinematics.target_velocities.angular_z=chassis->z;
	
	Kinematics.wheel1.target_speed.linear_vel=(chassis->z*15.0f)-chassis->y;
	Kinematics.wheel2.target_speed.linear_vel=(chassis->z*15.0f)+chassis->y*cos(3.1415926f/3)+chassis->x*sin(3.1415926f/3);
	Kinematics.wheel3.target_speed.linear_vel=(chassis->z*15.0f)+chassis->y*sin(3.1415926f/6)-chassis->x*cos(3.1415926f/6);
		
	//线速度 cm/s  转转度  RPM 
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel3.target_speed.rpm = Kinematics.wheel3.target_speed.linear_vel * VEL2RPM;
		
	chassis->motor[0].Target.Target_speed = (int)(Kinematics.wheel1.target_speed.rpm * M2006_REDUCTION_RATIO);
	chassis->motor[1].Target.Target_speed = (int)(Kinematics.wheel2.target_speed.rpm * M2006_REDUCTION_RATIO);
	chassis->motor[2].Target.Target_speed = (int)(Kinematics.wheel3.target_speed.rpm * M2006_REDUCTION_RATIO);	

	chassis->x=0;
	chassis->y=0;
	chassis->z=0;
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
 * @brief   中心速度优化，使速度变化更平滑
 * @param[in]  vx_desired: 期望的 x 方向速度
 * @param[in]  vy_desired: 期望的 y 方向速度
 * @param[out] vx_optimized: 优化后的 x 方向速度
 * @param[out] vy_optimized: 优化后的 y 方向速度
 */
float last_vx,last_vy;
static void Speed_optimize(float vx_desired, float vy_desired, float *vx_optimized, float *vy_optimized)
{
    // 速度优化
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
        // 停止速度优化
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

static float dt = 0, t = 0;
uint32_t CHASSIS_DWT_Count = 0;
float z_angel;
//正运动学公式
//通过轮胎的实际转速计算底盘几何中心的三轴速度
void Get_Base_Velocities(void  * argument)
{
	float x_length_coefficient=0.72977f;//x方向距离测量系数
	float y_length_coefficient=0.72977f;//y方向距离测量系数
//	chassis.Start_yaw = INS.YawTotalAngle;
	chassis.Start_yaw=4.2f;
//	chassis.Start_pitch=INS.Pitch;
	chassis.Start_pitch=1.8f;
	while(1)
	{
		dt = DWT_GetDeltaT(&CHASSIS_DWT_Count);//得到单位时间
		if(chassis.mode==CHASSIS_MODE_POWERLESS)
		{
			chassis.Start_yaw=INS.YawTotalAngle;
			chassis.Start_pitch=INS.Pitch;
			chassis.Roll_start=INS.Roll;
			z_angel=0.0f;
		}
		chassis.Yaw_angle = INS.YawTotalAngle-chassis.Start_yaw;//yaw角度
		while(chassis.Yaw_angle>180.0f)
		{
			chassis.Yaw_angle-=360.0f;
		}
		while(chassis.Yaw_angle<-180.0f)
		{
			chassis.Yaw_angle+=360.0f;
		}
		chassis.Actual_angle = - chassis.Yaw_angle;//Yaw轴角度
		chassis.Pitch_angle  =   INS.Pitch - chassis.Start_pitch;//pitch角度
		chassis.Roll_angle   =   INS.Roll-chassis.Roll_start;
		//根据电机转速测算轮子转速
		Kinematics.wheel1.actual_speed.rpm = (float)chassis.motor[0].Actual.Actual_speed / M2006_REDUCTION_RATIO;
		Kinematics.wheel2.actual_speed.rpm = (float)chassis.motor[1].Actual.Actual_speed / M2006_REDUCTION_RATIO;
		Kinematics.wheel3.actual_speed.rpm = (float)chassis.motor[2].Actual.Actual_speed / M2006_REDUCTION_RATIO;
		
		//轮子转速转换为轮子线速度
		Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
		Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
		Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
		
		//轮子线速度转换为底盘中心三轴的速度
		Kinematics.actual_velocities.angular_z = ( Kinematics.wheel1.actual_speed.linear_vel\
												 + Kinematics.wheel2.actual_speed.linear_vel\
												 + Kinematics.wheel3.actual_speed.linear_vel)/(3.0f*15.0f);
		Kinematics.actual_velocities.linear_x  = ( Kinematics.wheel2.actual_speed.linear_vel/sin(3.14/3) \
												 - Kinematics.wheel3.actual_speed.linear_vel/sin(3.14/3))/2.0f;
		Kinematics.actual_velocities.linear_y  = - Kinematics.wheel1.actual_speed.linear_vel*2.0f/3.0f \
												 + Kinematics.wheel2.actual_speed.linear_vel/3.0f \
												 + Kinematics.wheel3.actual_speed.linear_vel/3.0f;

		chassis.x_y[0] = chassis.x_y[0] 
					   + Kinematics.actual_velocities.linear_x*cos(chassis.Actual_angle/360.0f*2*PI) * dt * x_length_coefficient
					   - sin(chassis.Actual_angle/360.0f*2*PI)*Kinematics.actual_velocities.linear_y * dt * y_length_coefficient;
		chassis.x_y[1] = chassis.x_y[1] 
					   + Kinematics.actual_velocities.linear_y*cos(chassis.Actual_angle/360.0f*2*PI) * dt * y_length_coefficient
					   + sin(chassis.Actual_angle/360.0f*2*PI)*Kinematics.actual_velocities.linear_x * dt * x_length_coefficient;
		z_angel = z_angel + Kinematics.actual_velocities.angular_z*dt/3.14f*180.0f*0.78324f*0.981826;
		if(z_angel>180)
		{
			z_angel-=360;
		}
		else if(z_angel<-180)
		{
			z_angel+=360;
		}
//		chassis.Yaw_angle=0.5*(chassis.Yaw_angle+z_angel);
		vTaskDelay(13);
	}
}