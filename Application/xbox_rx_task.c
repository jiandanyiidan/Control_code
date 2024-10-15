#include "xbox_rx_task.h"
#include "chassis_task.h"
#include "Clamp_task.h"
#include <string.h>


xbox_data_t xbox_data,last_xbox_data;

uint8_t rx_data[60];
char receive_data_xbox[30];
uint8_t *rx_p;

uint8_t start_receive_flag_xbox = 0;

static void Rc_task(xbox_data_t *data,Chassis_t *chassis);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if(huart==&huart1)
	{
		for(uint16_t j=0;j<30;j++)
		{
			if(rx_data[j] == 0xA5 &&rx_data[j+29]==0xA2)//�����һλ����֡ͷ
			{
				start_receive_flag_xbox = 1;
			}
			if(start_receive_flag_xbox == 1)   //���ݴ���
			{
				memcpy(receive_data_xbox,rx_data+j,30);
				memcpy(&xbox_data,receive_data_xbox,30);
				Rc_task(&xbox_data,&chassis);
				memcpy(&last_xbox_data,&xbox_data,sizeof(xbox_data_t));
				start_receive_flag_xbox = 0;
			}
			if(start_receive_flag_xbox == 1)
			{   
				start_receive_flag_xbox=0;
			}
		}
	}
	HAL_UART_Receive_IT(&huart1,rx_data,sizeof(rx_data));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->ErrorCode&HAL_UART_ERROR_ORE)
	{
		uint32_t temp=huart->Instance->SR;
		temp =huart->Instance->DR;
		huart->Instance->CR1 |=UART_IT_RXNE;
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_IT(&huart1,rx_data,sizeof(rx_data));
	}
}

static void Rc_task(xbox_data_t *data,Chassis_t *chassis)
{
	chassis->x=(32767.0f-(float)data->joyLVert)/32767.0f*70.0f;
	clamp_to_zero(&chassis->x,5.0f);
	chassis->y=((float)data->joyLHori-32767.0f)/32767.0f*70.0f;
	clamp_to_zero(&chassis->y,5.0f);
	chassis->z=((float)data->joyRHori-32767.0f)/32767.0f*1.0f;
	clamp_to_zero(&chassis->z,0.1f);
	
	if(data->btnA&0x01&&(data->btnA!=last_xbox_data.btnA))
	{
	    Clamp_Task_1.flag_task = !Clamp_Task_1.flag_task;
	}
}

/**
 * @brief limit the value to zero if it is below the threshold
 * @param  value: the value to be limited
 * @param  threshold: the threshold
 * @retval None
*/
void clamp_to_zero(float *value, float threshold)
{
    if (fabs(*value) < threshold) {
        *value = 0.0;
    }
}