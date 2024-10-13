#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "string.h"
#define CAN_2006Motor1_ID    0x201
#define CAN_2006Motor2_ID    0x202
#define CAN_2006Motor3_ID    0x203
#define CAN_2006Motor4_ID    0x204

uint8_t bsp_can_init(void);
uint8_t platform_trans(void *peripheral_handle, uint8_t data[]);
#endif /* __BSP_CAN_H */
