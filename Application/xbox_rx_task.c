#include "xbox_rx_task.h"
#include "chassis_task.h"
#include "Clamp_task.h"
#include <string.h>
#include "USB_Commucation.h"
#include "Control_data.h"

xbox_data_t xbox_data={0};
xbox_data_t last_xbox_data={0};
uint8_t rx_data[32];
char receive_data_xbox[16];

/*
 * @brief 串口接收完成回调函数
 * @param  huart:串口句柄
 * @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	static uint8_t start_receive_flag_xbox = 0;	
	if(huart==&huart1)
	{
		for(uint16_t j=0;j<30;j++)
		{
			if(rx_data[j] == 0xA5 &&rx_data[j+15]==0xA2)//判断是否正确接收
			{
				start_receive_flag_xbox = 1;
				chassis.xbox_status=XBOX_LINK;
			}
			else if (rx_data[j] == 0xAA &&rx_data[j+1]==0xBB&&rx_data[j+2]==0xCC)
			{
				//此时遥控器未连接
				start_receive_flag_xbox = 2;
				chassis.xbox_status=XBOX_UNLINK;
			}
			if(start_receive_flag_xbox == 1)   //当正确接收时进行数据处理
			{
				last_xbox_data=xbox_data;
				memcpy(receive_data_xbox,rx_data+j,sizeof(receive_data_xbox));
				memcpy(&xbox_data,receive_data_xbox,sizeof(receive_data_xbox));
				Rc_task(&xbox_data,&chassis,&pack_rx,&Clamp_Task_1);
				start_receive_flag_xbox = 0;
			}
			else if(start_receive_flag_xbox == 2)
			{
				start_receive_flag_xbox=0;
			}
		}
	}
	HAL_UART_Receive_IT(&huart1,rx_data,sizeof(rx_data));
}

/*
 * @brief 串口接收错误回调函数
 * @param  huart:串口句柄
 * @retval None
*/
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
