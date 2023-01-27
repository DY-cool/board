#ifndef __REMOTE_H_
#define __REMOTE_H_

#include "stm32f1xx.h"
#include "uTask.h"


typedef struct
{
	struct
	{
		uint16_t sw;
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s1_last;
		uint8_t s2;
		uint8_t s2_last;
		uint8_t ChangeFlag;
		
		int16_t left_LR;//拨杆变换值
		int16_t left_LR_last;
		int16_t left_UD;
		int16_t right_LR;
		int16_t right_UD;
	}rc;
	struct
	{
		int16_t x;
		int16_t x_last;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_l_last;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint16_t v;
	}key;
}RC_Ctl_t;


void RC_CountorAdd(void);////遥控在线计数器
enum CheckOnlion getRC_onlineFlag(void);	//获得遥控器在线标志
void Remote_Init(UART_HandleTypeDef* huart);
void RemoteData_Write(UART_HandleTypeDef *huart,DMA_HandleTypeDef* dma);


#endif
