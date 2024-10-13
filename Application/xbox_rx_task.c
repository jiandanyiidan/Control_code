#include "xbox_rx_task.h"

xbox_data_t xbox_data;

uint8_t rx_data[60];
char receive_data[30];
uint8_t *rx_p;

uint8_t start_receive_flag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if(huart==&huart2)
	{
		for(uint16_t j=0;j<30;j++)
		{
			if(rx_data[j] == 0xA5 &&rx_data[j+29]==0xA2)//如果这一位等于帧头
			{
				start_receive_flag = 1;
			}
			if(start_receive_flag == 1)   //数据处理
			{
				memcpy(receive_data,rx_data+j,30);
				memcpy(&xbox_data,receive_data,30);
			}
			if(start_receive_flag == 1)
			{   
				start_receive_flag=0;
			}
		}
	}
	HAL_UART_Receive_IT(&huart2,rx_data,sizeof(rx_data));
}

