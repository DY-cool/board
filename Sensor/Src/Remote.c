#include "Remote.h"
#include "usart.h"
#include "dma.h"
#include "uTask.h"
#include <string.h>


#define RcBuffLen 30u
uint8_t RcBuff[RcBuffLen]={0};
RC_Ctl_t RC_Ctl;
uint8_t Updata_RCflag=0;		//遥控器数据解算更新标志位
uint8_t RC_UpdataCountot=0;	//遥控器更新标志位


#define RC_ONLINE_LIM_MIN_NUM	5	//接收消息5次后开始发送消息
uint8_t RC_OnLineCount=0;		//遥控器连接后接收次数计数


void RC_CountorAdd(void)
{
		RC_UpdataCountot++;
}


enum CheckOnlion getRC_onlineFlag(void)//获得遥控器在线标志
{

	if(RC_UpdataCountot>=35){
	
	RC_UpdataCountot=50;
	RC_OnLineCount=0;
	return Lost;		//遥控器断线
	
	}
	else
	if(RC_UpdataCountot<35){
	
	return OnLion;	//遥控器在线
		
	}
	

	
}
/*
*数据传输函数
*/
uint8_t RCdata_Updata(Data_Struct* In_Struct){
	if(RC_OnLineCount>=RC_ONLINE_LIM_MIN_NUM){
		if(Updata_RCflag==0)
		{
		
			if(RC_Ctl.rc.s1==1){
			
				In_Struct->Staion=STATE_UC;	//上位机控制
				
			}else
				if(RC_Ctl.rc.s1==3){
					In_Struct->Staion=STATE_RC;	//遥控控制
				
				
					In_Struct->Linear_X=RC_Ctl.rc.right_UD;
					In_Struct->Angular_Z=RC_Ctl.rc.left_LR;
				}else
				if(RC_Ctl.rc.s1==2){
				In_Struct->Staion=STATE_REST;
					In_Struct->Linear_X=0.0f;
					In_Struct->Angular_Z=0.0f;
					RC_Ctl.rc.right_UD=0;
					RC_Ctl.rc.left_LR=0;

				}
			return 0;
		}
		else
		if(Updata_RCflag==1){
		
			In_Struct->Staion=STATE_WAIT;	//数据解算未完成
			
			return 1;
			
		}
	}
	else{
	
			In_Struct->Linear_X=0.0f;
			In_Struct->Angular_Z=0.0f;

	}
	
}






void Remote_Init(UART_HandleTypeDef* huart){

	HAL_UART_Receive_DMA(huart, RcBuff, RcBuffLen);//串口DMA接收
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//开启空闲接收中断

}
void RemoteTrate(uint8_t*sbus_rx_buffer ){
					if(RC_OnLineCount<RC_ONLINE_LIM_MIN_NUM){
					RC_OnLineCount++;
					}
					Updata_RCflag=1;//开始更新遥控数据
					RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0   高8位与低3位
					RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1     高5位与低6位
					RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) |(sbus_rx_buffer[4] << 10)) & 0x07ff; //!< Channel 2
					RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
					RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
					RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
					RC_Ctl.mouse.x = (sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8)); //!< Mouse X axis
					RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
					RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
					RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press 
					RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press 
					RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); //!< KeyBoard value
					
					RC_Ctl.rc.right_LR=(RC_Ctl.rc.ch0-1024);//将拨杆值变换                  //4个通道（1024为中间值）
					RC_Ctl.rc.right_UD=(RC_Ctl.rc.ch1-1024);
					RC_Ctl.rc.left_LR=(RC_Ctl.rc.ch2-1024);
					RC_Ctl.rc.left_UD=(RC_Ctl.rc.ch3-1024);
					//拨轮
					RC_Ctl.rc.sw =(uint16_t)(sbus_rx_buffer[16]|(sbus_rx_buffer[17]<<8))&0x7FF;//11 bit
					Updata_RCflag=0;
}



void RemoteData_Write(UART_HandleTypeDef *huart,DMA_HandleTypeDef* dma){
		uint32_t temp = 0;
		int32_t len =0;

			__HAL_UART_CLEAR_IDLEFLAG(huart);
			temp = huart->Instance->SR;
			temp = huart->Instance->DR;
			temp = dma->Instance->CNDTR; 
			HAL_UART_DMAStop(huart);
			len=RcBuffLen-temp;
				if((RcBuffLen-temp)==18){
				RC_UpdataCountot=0;
					RemoteTrate(RcBuff);
					memset(RcBuff,0,RcBuffLen);
				}
			HAL_UART_Receive_DMA(huart, (uint8_t*)RcBuff, RcBuffLen);//串口DMA接收

}


