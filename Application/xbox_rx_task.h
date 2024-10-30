#ifndef _XBOX_RX_TASK_H
#define _XBOX_RX_TASK_H
#include "usart.h"
#include "stdbool.h"
#include "string.h"



typedef enum
{
	XBOX_LINK = 0,//连接
	XBOX_UNLINK = 1,		//断开
}xbox_link_status_t;

typedef union 
{
	uint16_t allbit;
	struct 
	{
		uint16_t bit0 :1;
		uint16_t bit1 :1;
		uint16_t bit2 :1;
		uint16_t bit3 :1;
		uint16_t bit4 :1;
		uint16_t bit5 :1;
		uint16_t bit6 :1;
		uint16_t bit7 :1;
		uint16_t bit8 :1;
		uint16_t bit9 :1;
		uint16_t bit10 :1;
		uint16_t bit11 :1;
		uint16_t bit12 :1;
		uint16_t bit13 :1;
		uint16_t bit14 :1;
		uint16_t bit15 :1;
	}bit;
}BIT_FLAG;

typedef __packed struct
{
	uint8_t header;
//	BIT_FLAG Flag;
	uint8_t btnA :1;
	uint8_t btnB :1;
	uint8_t btnX :1;
	uint8_t btnY :1;  
	uint8_t btnShare :1;
	uint8_t btnStart :1;
	uint8_t btnSelect :1;
	uint8_t btnXbox :1;
	uint8_t btnLB :1;
	uint8_t btnRB :1;
	uint8_t btnLS :1;
	uint8_t btnRS :1;
	uint8_t btnDirUp :1;
	uint8_t btnDirLeft :1;
	uint8_t btnDirRight :1;
	uint8_t btnDirDown :1;
	uint16_t joyLHori;
	uint16_t joyLVert;
	uint16_t joyRHori;
	uint16_t joyRVert;
	uint16_t trigLT; 
	uint16_t trigRT;
	uint8_t frame;
}xbox_data_t;


extern uint8_t rx_data[32];
extern xbox_data_t xbox_data,last_xbox_data;
#endif

//	uint8_t btnA :1;
//	uint8_t btnB :1;
//	uint8_t btnX :1;
//	uint8_t btnY :1;  
//	uint8_t btnShare :1;
//	uint8_t btnStart :1;
//	uint8_t btnSelect :1;
//	uint8_t btnXbox :1;
//	uint8_t btnLB :1;
//	uint8_t btnRB :1;
//	uint8_t btnLS :1;
//	uint8_t btnRS :1;
//	uint8_t btnDirUp :1;
//	uint8_t btnDirLeft :1;
//	uint8_t btnDirRight :1;
//	uint8_t btnDirDown :1;