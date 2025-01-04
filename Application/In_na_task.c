#include "In_na_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "USB_Commucation.h"

#define PITCH_LIMIT_angle 1.1f
uint32_t IN_DWT_Count = 0;


//ï¿½ï¿½È«ï¿½ï¿½yï¿½ï¿½ï¿½ï¿½Îª115ï¿½Õºï¿½ï¿½ï¿½ï¿½ï¿½Ë¹Åµï¿½ï¿½ï¿½ï¿½
float posion_map[5][3] ={
						{0.0f  , 0.0f ,   0.0f},//³õÊ¼Î»ÖÃ
						{90.0f, 0.0f ,   0.0f},//ÖÐÐÄµãÎ»ÖÃ
						{240.0f, 0.0f ,   180.0f  },
						{120.0f, 88.0f,   180/2},
						{120.0f,-84.0f,  -180/2},
							};
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½Ñ²ï¿½ï¿½ï¿½Ä²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
POSITION_t position;	

/**
 * @brief          
 * @param[in]        POSITION
 * @param[in]      
 * @retval               null					
*/							
void Posion_init(POSITION_t *posion)
{
	posion->target_posion = Central_position_task;
	
	posion->Task_status = UNDETECTED;
	
	posion->task_switch = TASK_OFF;
	//
	posion->cruise=OFF;
	//
	posion->Target_coordinate[0]=0.0f;
	posion->Target_coordinate[1]=0.0f;
	posion->Actual_coordinate[0]=0.0f;
	posion->Actual_coordinate[1]=0.0f;
	
	//
	PID_Init(&posion->Position[0],
	110.0f,0.0f,1.1f,1.9f,0.0f,0.00f,0.0f,0.0f,0.0f,0.0f,0,0);
	PID_Init(&posion->Position[1],
	110.0f,0.0f,1.1f,1.9f,0.0f,0.00f,0.0f,0.0f,0.0f,0.0f,0,0);
	PID_Init(&posion->Position[2],
	2.0f,  0.0f,0.1f,0.45f,0.0f,0.00f,0.0f,0.0f,0.0f,0.0f,0,0);
	
	//
	posion->Target_yaw = 0.0f;
	posion->Actual_yaw = 0.0f;
	posion->tran_ok=0;
}
/**
 * @brief          
 * @param[in]               POSITION_t
 * @param[in]        		Pack_rx_t
 * @retval                  null					
*/	
void Task_status_update(POSITION_t *posion,Pack_rx_t *pack)
{
//	posion->Task_status=TRANSPORT;
	if((posion->task_switch==TASK_OFF && posion->tran_ok==0)||(posion->tran_ok<150 && posion->tran_ok>70))
	{
		switch(pack->state)
		{
			case 0://Î´¼ì²âµ½
				posion->Task_status=UNDETECTED;
				break;
			case 1://¼ì²âµ½
				posion->Task_status=DETECTED;
				posion->tran_ok=0;
				break;
			case 2://ÔÝÊ±¶ªÊ§
				posion->Task_status=TEMOIRARY_LOSS;
				break;
			case 3://²¶»ñ
				posion->Task_status=PICK_UP;
				break;
			case 4://ÔËÊä
				posion->Task_status=TRANSPORT;
				break;
			case 5://
				
				break;
		}	
		
	}
	else
	{
		posion->Task_status=TRAN_OK;
	}
}
// 
void updateTargetPosition(POSITION_t *position, float x, float y, float yaw) {
    position->Target_coordinate[0] = x;
    position->Target_coordinate[1] = y;
    position->Target_yaw = yaw;
}
/**
 * @brief          
 * @param[in]               POSITION_t
 * @param[in]      
 * @retval                       null					
*/	
void Point_task(POSITION_t *position)
{
	if(position->task_switch==TASK_ON)//
	{
		if(position->Task_status==UNDETECTED ) 
		{
			//
			switch(position->target_posion)
			{
				case Departure_position_task://
				{
					updateTargetPosition(position, posion_map[0][0], posion_map[0][1], posion_map[0][2]);
					break;			
				}
				case Central_position_task://
				{
					updateTargetPosition(position, posion_map[1][0], posion_map[1][1], posion_map[1][2]);
					position->task_switch=TASK_OFF;
					pack.tran_flag=0;
					break;
				}
				case Safe_area_locaton://
				{
					updateTargetPosition(position, posion_map[3][0], posion_map[3][1], posion_map[3][2]);
					position->task_switch=TASK_OFF;
					break;
				}
				case Opposite_departure://
				{
					
					break;
				}
				case Opposite_Safe://
				{
					updateTargetPosition(position, posion_map[Opposite_Safe][0], posion_map[Opposite_Safe][1], posion_map[Opposite_Safe][2]);
					break;
				}
				case actual_posion:
				{
					updateTargetPosition(position, chassis.x_y[0], chassis.x_y[1], chassis.Actual_angle);
					break;
				}
				default:
				{
					break;
				}
			}
			if(position->Task_status==TRANSPORT)
			{
				position->Task_status=TRAN_OK;
			}
		}
	}
	if(position->Task_status==TRAN_OK || position->task_switch==TASK_OFF)//	{
	{
		//
		switch(position->target_posion)
		{
			case Departure_position_task://
			{
				updateTargetPosition(position, posion_map[0][0], posion_map[0][1], posion_map[0][2]);
				break;			
			}
			case Central_position_task://
			{
				updateTargetPosition(position, posion_map[1][0], posion_map[1][1], posion_map[1][2]);
				break;
			}
			case Safe_area_locaton://
			{
				updateTargetPosition(position, posion_map[3][0], posion_map[3][1], posion_map[3][2]);
				break;
			}
			case Opposite_departure://
			{
				
				break;
			}
			case Opposite_Safe://
			{
				updateTargetPosition(position, posion_map[Opposite_Safe][0], posion_map[Opposite_Safe][1], posion_map[Opposite_Safe][2]);
				break;
			}
			case actual_posion:
			{
				updateTargetPosition(position, chassis.x_y[0], chassis.x_y[1], chassis.Actual_angle);
				break;
			}
			default:
			{
				break;
			}
		
		}
		
	}
}
/*
	
*/
void Excute_task(POSITION_t *posion)
{
	if(posion->task_switch==TASK_ON && posion->Task_status==TRANSPORT)
	{
		//
		posion->z_out=0;
		posion->x_out=0;
		posion->y_out=0;
		return ;
	}
	else
	{
		//×´Ì¬ÅÐ¶Ï
		if(posion->Task_status==UNDETECTED
			||posion->Task_status==TEMOIRARY_LOSS
			||posion->Task_status==TRANSPORT
			||posion->Task_status==TRAN_OK)
		{
			//z_outµÄ¼ÆËã
			if((fabs(chassis.Actual_angle-posion->Target_yaw)<fabs(posion->Target_yaw/20.0f))
			 || fabs(posion->Target_yaw)<1.0f)
			{
				posion->Position[2].Kp=0.3f;
				posion->Position[2].MaxOut=1.3f;
				if(posion->cruise==OFF)
				{
					if(posion->Task_status==TRAN_OK && posion->tran_ok<150)
					{
						if(posion->tran_ok<60)
						{
							posion->z_out=0.0f;
						}
						else if(posion->tran_ok<150)
						{
							posion->z_out=-2.0;
							posion->x_out = 0.0f;
						}	
					}
					else
					{
						static float yaw_angle=0;
						if(posion->Target_yaw-chassis.Actual_angle>180.0f)
						{
							yaw_angle=chassis.Actual_angle+360.0f;
						}
						else if(posion->Target_yaw-chassis.Actual_angle<-180.0f)
						{
							yaw_angle=chassis.Actual_angle-360.0f;
						}
						else
						{
							yaw_angle=chassis.Actual_angle;
						}
						//½ÇËÙ¶È¼ÆËã
						posion->z_out=PID_Calculate
								(&posion->Position[2],
								  yaw_angle,
								  posion->Target_yaw);
					}
				}
				else if(posion->cruise==ON)
				{
					posion->z_out=-1.5;	
				}
			}
			else
			{
				posion->Position[2].Kp=0.7;
				posion->Position[2].MaxOut=2.1f;
				if(posion->cruise==OFF)
				{
					if(posion->Task_status==TRAN_OK && posion->tran_ok<150)
					{
						if(posion->tran_ok<60)
						{
							posion->z_out=0.0f;
						}
						else if(posion->tran_ok<150)
						{
							posion->z_out=-2.0;
							posion->x_out = 0.0f;
						}	
					}
					else
					{
						static float yaw_angle=0;
						if(posion->Target_yaw-chassis.Actual_angle>180.0f)
						{
							yaw_angle=chassis.Actual_angle+360.0f;
						}
						else if(posion->Target_yaw-chassis.Actual_angle<-180.0f)
						{
							yaw_angle=chassis.Actual_angle-360.0f;
						}
						else
						{
							yaw_angle=chassis.Actual_angle;
						}
						//½ÇËÙ¶È¼ÆËã
						posion->z_out=PID_Calculate
								(&posion->Position[2],
								  yaw_angle,
								  posion->Target_yaw);
						posion->Position[3].Output=0;
					}
				}
				else if(posion->cruise==ON)
				{
//					if(INS.YawTotalAngle<360.0f)
//					{
//						posion->z_out=+1.5;	
//					}
//					else if(INS.YawTotalAngle<-360.0f)
//					{
						posion->z_out=-1.5;	
//					}	
				}
			}

				
			if((fabs(chassis.x_y[0]-posion->Target_coordinate[0])<fabs((posion->Target_coordinate[0])/40.0f)) 
			 || fabs(posion->Target_coordinate[0])<3.0f)
			{
				posion->Position[0].Kp=0.9f;
			}
			else
			{
				posion->Position[0].Kp=1.9f;
			}
			//
			if(posion->Task_status!=TRAN_OK)
			{
				if((posion->Task_status==TRANSPORT) && (chassis.x_y[1]>(posion_map[3][1]+10)))
				{
					posion->vy=-100;
				}
				else
				{
					posion->vx=PID_Calculate
						 (&posion->Position[0],
						   chassis.x_y[0],
						   posion->Target_coordinate[0]);
					posion->Position[0].Output=0;
				}
			}
			
			if((fabs(chassis.x_y[1]-posion->Target_coordinate[1])<fabs((posion->Target_coordinate[1])/40.0f)) 
			 || fabs(posion->Target_coordinate[1])<3.0f)
			{
				posion->Position[1].Kp=0.9f;
			}
			else
			{
				posion->Position[1].Kp=2.0f;
			}
			//
			posion->vy=PID_Calculate
			 (&posion->Position[1],
			   chassis.x_y[1],
			   posion->Target_coordinate[1]);
			
			if(posion->vy>0 &&posion->vy<30 && posion->Task_status==TRANSPORT &&fabs(chassis.x_y[1]-posion->Target_coordinate[1])<5)
			{
				posion->vy+=30;
				if(posion->vy>40)
				{
					posion->vy=40;
				}
			}

			if(posion->tran_ok<60 && posion->Task_status==TRAN_OK)
			{
				posion->x_out=-100.0f;
				posion->y_out= 0.0f;
			}
			else if(posion->tran_ok<150 && posion->Task_status==TRAN_OK)
			{
				posion->x_out= 0.0f;
				posion->y_out= 0.0f;
			}
			else
			{
				posion->x_out=cos(posion->Actual_yaw) * posion->vx 
							+ sin(posion->Actual_yaw) * posion->vy;
			
				posion->y_out=cos(posion->Actual_yaw) * posion->vy 
							- sin(posion->Actual_yaw) * posion->vx;
			}
		}
		else 
		{
			posion->y_out=0;
		}
	}
}
/**
 * @brief          ²ÎÊý¸üÐÂ£¬ÔÝÊ±ÐèÇó²»´ó
 * @param[in]      POSITION_t
 * @param[in]        
 * @retval		   null					
*/	
void Update_param(POSITION_t *position,Chassis_t *chassis)
{
	position->Actual_yaw=-chassis->Yaw_angle/180.0f*PI;
	position->Actual_coordinate[0]=chassis->x_y[0];
	position->Actual_coordinate[1]=chassis->x_y[1];
}
/**
 * @brief          
 * @param[in]        POSITION_t
 * @param[in]        Pack_rx_t
 * @param[in]        Clamp_Task_t
 * @retval           null					
*/	
void Update_posion_task(
				POSITION_t *posion,
				Pack_rx_t *pack,
				Clamp_Task_t *Clamp_Task)
{
	static float dt=0;
	dt = DWT_GetDeltaT(&IN_DWT_Count);

	//×´Ì¬¸üÐÂ£¬¸üÐÂÈÎÎñ×´Ì¬
	Task_status_update(posion,&pack_rx);

	switch(posion->Task_status)
	{
		case UNDETECTED://Î´Ê¶±ð		
		{
			handleUndetected(posion);
			break;			
		}
		case DETECTED://Ê¶±ð
		{
            handleDetected(posion, pack, dt);
			break;
		}
		case TEMOIRARY_LOSS://ÔÝÊ±¶ªÊ§
		{
			handleTemporaryLoss(posion);
			break;
		}
		case PICK_UP://×¥È¡
		{
            handlePickUp(posion, Clamp_Task);
			break;
		}
		case TRANSPORT:
		{ 
            handleTransport(posion);
			break;
		}
		case TRAN_OK:
		{
            handleTransportOk(posion, Clamp_Task);
			break;
		}
		default:
		{
			break;
		}
	}
//	if(posion->Task_status!=TRAN_OK)
//	{
//		posion->tran_ok=0;
//	}
	Point_task(posion);
	if(posion->Task_status != PICK_UP && posion->Task_status != DETECTED)
	{
		Excute_task(posion);
	}
}
//ÈÎÎñµ÷¶ÈµÄÖ´ÐÐº¯Êý
void IN_Task(void  * argument)
{
	Posion_init(&position);
	while(1)
	{
		static int time_flag=0;
		//time_flag++;
		if(fabs(chassis.Pitch_angle)>1.3f)
		{
			pack.error_judge=1;
		}
		else
		{
			pack.error_judge=0;
		}
		if(chassis.mode==CHASSIS_MODE_POWERLESS)
		{
			pack.chassis_status=0;
		}
		else
		{
			pack.chassis_status=1;
		}
		pack.yaw_angle=chassis.Actual_angle;
		pack.x=chassis.x_y[0];
		pack.y=chassis.x_y[1]-(posion_map[3][1]-88.0f);
		Update_param(&position,&chassis);
		if(chassis.mode==CHASSIS_MODE_VISION)
		{
			Update_posion_task(&position,&pack_rx,&Clamp_Task_1);
			if((chassis.x_y[0]>(posion_map[3][0]-40))
			 &&(chassis.x_y[0]<(posion_map[3][0]+40))
			 &&(chassis.x_y[1]<(posion_map[3][1]-150))
			 &&(chassis.x_y[1]>(posion_map[3][1]-190)))
			{
				position.x_out=0;
				position.y_out=0;
				position.z_out=1.0;
			}
		}
		USB_TX();
		
		vTaskDelay(20);
	}
}

//Î´¼ì²âµ½Ä¿±ê
void handleUndetected(POSITION_t *posion) 
{
    posion->target_posion = Central_position_task;
    posion->cruise = ((fabs(posion->Target_coordinate[0] - posion->Actual_coordinate[0]) < 15)
				    &&(fabs(posion->Target_coordinate[1] - posion->Actual_coordinate[1]) < 15)) ? ON : OFF;
    Clamp_Task_1.flag_task = 0;
}

//¼ì²âµ½Ä¿±ê
void handleDetected(POSITION_t *posion, Pack_rx_t *pack_rx, float dt) 
{
    posion->target_posion = actual_posion;
    posion->x_out = pack_rx->speed;
    posion->y_out = 0.0f;
    posion->z_out = - pack_rx->yaw_add * dt * 1.2f;
    Clamp_Task_1.flag_task = 0;
    posion->cruise = OFF;
	posion_map[1][2]=chassis.Yaw_angle;
	if (fabs(chassis.Pitch_angle) > PITCH_LIMIT_angle && fabs(posion_map[3][1] - posion->Actual_coordinate[1]) < 3) 
	{
		posion->task_switch = TASK_ON;
		posion_map[3][1] = chassis.x_y[1];
		posion_map[3][0] = chassis.x_y[0];
	} 
	else 
	{
		posion->task_switch = TASK_OFF;
	}
}

// ÔÝÊ±¶ªÊ§Ä¿±ê
void handleTemporaryLoss(POSITION_t *posion) 
{
    posion->target_posion = Central_position_task;
    posion->cruise = ((fabs(posion->Target_coordinate[0] - posion->Actual_coordinate[0]) < 5)
				    &&(fabs(posion->Target_coordinate[1] - posion->Actual_coordinate[1]) < 5)) ? ON : OFF;
    Clamp_Task_1.flag_task = 0;
}

//²¶»ñÄ¿±ê
void handlePickUp(POSITION_t *posion, Clamp_Task_t *Clamp_Task) 
{
    posion->cruise = OFF;
    posion->x_out = 0.0f;
    posion->y_out = 0.0f;
    posion->z_out = 0.0f;
    Clamp_Task->flag_task = 1;
    posion->target_posion = actual_posion;
	posion_map[1][2]=chassis.Yaw_angle;
}

//ÔËÊäÄ¿±ê
void handleTransport(POSITION_t *posion) 
{
    static uint32_t time_out = 0;
    time_out++;

	posion->target_posion = Safe_area_locaton;
	time_out = 0;

	
	if(posion->target_posion == Safe_area_locaton)
	{
		posion->cruise = OFF;
		if (fabs(chassis.Pitch_angle) > PITCH_LIMIT_angle && fabs(posion->Target_coordinate[1] - posion->Actual_coordinate[1]) < 8) 
		{
			posion->task_switch = TASK_ON;
			posion_map[3][1] = chassis.x_y[1];
			posion_map[3][0] = chassis.x_y[0];
			
		} 
		else 
		{
			posion->task_switch = TASK_OFF;
		}

		if (posion->task_switch == TASK_OFF && fabs(posion->Target_coordinate[1] - posion->Actual_coordinate[1]) < 5) {
			posion_map[3][1] += 0.20f	;
			posion_map[1][1] = posion_map[3][1] - 80.0f;
		}
	}
	posion_map[1][2]=chassis.Yaw_angle;
}

//ÔËÊä³É¹¦Ä¿±ê
void handleTransportOk(POSITION_t *posion, Clamp_Task_t *Clamp_Task) 
{
    posion->tran_ok++;

    posion->cruise = OFF;

    if (posion->tran_ok > 150) 
	{
        posion->target_posion = Central_position_task;
		posion->task_switch=TASK_OFF;
        posion->tran_ok = 0;
		pack.tran_flag=0;
    }
	else if(posion->tran_ok >60)
	{
		posion->z_out = -1.5f;
	}
	else if (posion->tran_ok > 1) 
	{
        posion_map[1][0] = 120;
		posion_map[1][1] = posion_map[3][1] - 80.0f;
		posion_map[1][2]=chassis.Yaw_angle;
        Clamp_Task->flag_task = 0;
        posion->x_out = -90.0f;
		posion->z_out = -0.0f;
		posion->y_out = 0.0f;
		pack.tran_flag=1;
    }
}