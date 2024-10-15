#ifndef _XBOX_RX_TASK_H
#define _XBOX_RX_TASK_H
#include "usart.h"
#include "stdbool.h"
#include "string.h"

typedef __packed struct
{
  uint8_t header;
  uint8_t btnA ;
  uint8_t btnB ;
  uint8_t btnX ;
  uint8_t btnY ;  
  uint8_t btnShare ;
  uint8_t btnStart ;
  uint8_t btnSelect ;
  uint8_t btnXbox ;
  uint8_t btnLB ;
  uint8_t btnRB ;
  uint8_t btnLS ;
  uint8_t btnRS ;
  uint8_t btnDirUp ;
  uint8_t btnDirLeft ;
  uint8_t btnDirRight ;
  uint8_t btnDirDown ;
  uint16_t joyLHori;
  uint16_t joyLVert;
  uint16_t joyRHori;
  uint16_t joyRVert;
  uint16_t trigLT; 
  uint16_t trigRT;
  uint8_t frame;
}xbox_data_t;

void clamp_to_zero(float *value, float threshold);
extern uint8_t rx_data[60];
extern xbox_data_t xbox_data;
#endif