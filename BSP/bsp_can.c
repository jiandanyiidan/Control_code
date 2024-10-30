#include "bsp_can.h"
#include "Motor_bsp.h"

uint8_t bsp_can_init(void)
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	
	canFilter.FilterBank=1;    																
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							
	canFilter.FilterActivation=CAN_FILTER_ENABLE;						
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 						
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					
	canFilter.SlaveStartFilterBank=14;													
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);				
	
	canFilter.FilterBank=15;    														
//	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);				
	
	/*开启CAN通信*/
	HAL_CAN_Start(&hcan1);				
//	HAL_CAN_Start(&hcan2);
	
	
	/*使能中断*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);   
//	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);     
	return status;
}

/**
 * @brief 供用户自定义的发送函数
 * @param peripheral_handle 电机发送时外设的句柄
 * @param CANID 发送消息的拓展帧
 * @param data[] 发送消息的数据帧
 * @return 发送是否成功
 */
uint8_t platform_trans(void *peripheral_handle, uint8_t data[])
{
	CAN_TxHeaderTypeDef Tx_data;
	uint8_t txdata[8];
	uint32_t txMailbox;
	Tx_data.StdId=0x200;
	Tx_data.ExtId=0x00;
	Tx_data.IDE=CAN_ID_STD;
	Tx_data.RTR=CAN_RTR_DATA;
	Tx_data.DLC=8;
	Tx_data.TransmitGlobalTime=DISABLE;
	memcpy(txdata,data,8);
	if(peripheral_handle==&hcan1)
	{
		HAL_CAN_AddTxMessage(peripheral_handle,&Tx_data,txdata,&txMailbox);
	}
	return 0;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef Rx_data;
	uint8_t rxdata[8];
	
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rx_data,rxdata);
		Motor_Rx_data(rxdata,&Rx_data);
	}
//	else if(hcan->Instance==CAN2)
//	{
//		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&Rx_data,rxdata);
//	}
}
