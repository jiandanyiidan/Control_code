#include "USB_Commucation.h"
#include "Clamp_task.h"
Pack_tx_t pack;
Pack_rx_t pack_rx;


/**
  * @brief CRC16 Caculation function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
  * @return : CRC16 checksum
  */
uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL) return 0xFFFF;
  while (dwLength--) {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}

/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */

bool Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return;

  w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

//接收处理函数
void cdc_vcp_data_rx (uint8_t *buf, uint32_t Len)
{
    for(uint16_t j=0;j<Len;j++)
    {
        if(buf[j] == 0xA5 )//如果这一位等于帧头
        {
            start_receive_flag = 1;
        }
        if(start_receive_flag == 1)   //数据处理
        {
            memcpy(receive_data,buf+j,Len);
            memcpy(&pack_rx,receive_data,Len);
        }
        if(start_receive_flag == 1)
        {  
            //改为下位机解算后注释掉
            start_receive_flag=0;
            return;
        }
    }
}

void USB_TX(void)
{ 
	pack.hander       =0x5A;

	//标记一下，后面测通信
	pack.saved     = Clamp_Task_1.flag_task;
	pack.crc16     =0xffff;
	memcpy(Buffer,&pack,sizeof(pack));
	Append_CRC16_Check_Sum(Buffer,sizeof(pack));
	CDC_Transmit_FS(Buffer,sizeof(pack));
}
