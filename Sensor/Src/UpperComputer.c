#include "UpperComputer.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "IMU.h"
#include	"Kinematics_4WD.h"
#include "BatteryInfor.h"
#include "usart.h"
#include <string.h>
#include "ROS_ShareWare.h"
/*-------------------------------环形缓冲区---------------------------------------------------*/

void UpdataTargetSpeed4PC(uint8_t *Buff);
void pc_nvic(uint8_t *data );
void PCerrState();
void PCData_Write(UART_HandleTypeDef *huart,DMA_HandleTypeDef* dma){
		uint32_t temp = 0;
		int32_t len =0;

			__HAL_UART_CLEAR_IDLEFLAG(huart);
			temp = huart->Instance->SR;
			temp = huart->Instance->DR;
			temp = dma->Instance->CNDTR; 
			HAL_UART_DMAStop(huart);

			pc_nvic(Usart2_RX_Buf);
//		memset(Usart2_RX_Buf,0,Usart2_RX_LEN);
		HAL_UART_Receive_DMA(huart, (uint8_t*)Usart2_RX_Buf, Usart2_RX_LEN);//串口DMA接收

}
uint8_t PC_dataBuff[50]={0};
void pc_nvic(uint8_t *data )
{
	uint8_t i=0,t=0,len=0;
	for(i=0;i<Usart2_RX_LEN;i++){
		if(data[i]==0xFE)
		{
			if(data[i+1]==0xEF)
			{
			len=data[i+2]+3;
				for(t=2;t<len;t++)
				{
				PC_dataBuff[t]=data[i+t];
					
				}
				UpdataTargetSpeed4PC(PC_dataBuff);
			break;
			}
		
		}
	
	
	}
	
	

}



void PC_msgUpdate(){

Sendf2PcDataCom();

}
union floatHex_union floatHex;
union int16Hex_union int16Hex;
union uint32Hex_union uint32Hex;

struct PCdataUp IMUdata2pc={0,0,0,0,0,0,0,0,0};
void update_IMUdata(struct PCdataUp *data){
	
	IMUdata2pc.Pitch=data->Pitch;
	IMUdata2pc.Roll=data->Roll;
	IMUdata2pc.Yaw=data->Yaw;
	IMUdata2pc.Acc_x=data->Acc_x;
	IMUdata2pc.Acc_y=data->Acc_y;
	IMUdata2pc.Acc_z=data->Acc_z;
	IMUdata2pc.Gyro_x=data->Gyro_x;
	IMUdata2pc.Gyro_y=data->Gyro_y;
	IMUdata2pc.Gyro_z=data->Gyro_z;
}

Kinematics_Struct Kinematics_Forward2pc={0,0,0,0,0,0,0};//创建车体运动学正解结构体参数
void update_MOVEdata(Kinematics_Struct *data){

	Kinematics_Forward2pc.Linear_X=data->Linear_X;
	Kinematics_Forward2pc.Linear_Y=data->Linear_Y;
	Kinematics_Forward2pc.Angular_Z=data->Angular_Z;
}

Battery_Struct Battery2pc={0,0,0,0};//电池状态信息结构体参数
void update_Batterydata(Battery_Struct *data){
	Battery2pc.Voltage=data->Voltage;
	
	Battery2pc.Capacity=data->Capacity;
}

uint8_t getSpeetFlag=0;
PcTargetSpeed  TargetSpeed={0,0,0};
uint8_t PcTargetData(PcTargetSpeed *data ){

	data->Angular_Z=TargetSpeed.Angular_Z;
	data->Linear_X=TargetSpeed.Linear_X;
	data->Linear_Y=TargetSpeed.Linear_Y;
	
	if(getSpeetFlag==1){
		getSpeetFlag=0;
		return 1;
	}
	else
	{
	
		return 0;
	}
	
}
void PCerrState(){
TargetSpeed.Angular_Z=0;
TargetSpeed.Linear_X=0;
TargetSpeed.Linear_Y=0;
}
__IO uint8_t Usart2_TX_Buf[66];
void UpdataTargetSpeed4PC(uint8_t *Buff)
{
	if(Buff[3]==1)
	{
		
		getSpeetFlag=1;//接收到上位机目标速度将标志位置1
			
		for(uint8_t i=0; i<4; i++) floatHex.Hex[3-i] =  Buff[4+i];
		TargetSpeed.Linear_X = floatHex.floatValue;//车体X轴目标线速度值 m/s
		
		for(uint8_t i=0; i<4; i++) floatHex.Hex[3-i] =  Buff[8+i];
		TargetSpeed.Linear_Y = floatHex.floatValue;//车体Y轴目标线速度值 m/s
		
		for(uint8_t i=0; i<4; i++) floatHex.Hex[3-i] =  Buff[12+i];
		TargetSpeed.Angular_Z = floatHex.floatValue;//车体Z轴目标角速度值 rad/s

	}

}

void Sendf2PcDataCom(void)
{

			Usart2_TX_Buf[0] = 0xFE;
			Usart2_TX_Buf[1] = 0xEF;
			Usart2_TX_Buf[2] = 60;
			Usart2_TX_Buf[3] = getSpeetFlag;//Data acquisition status flag bit of upper computer
			floatHex.floatValue = Kinematics_Forward2pc.Linear_X;//车体当前X轴线速度 m/s
//			floatHex.floatValue = 120.0f;//车体当前X轴线速度 m/s
			Usart2_TX_Buf[4] = floatHex.Hex[3];
			Usart2_TX_Buf[5] = floatHex.Hex[2];
			Usart2_TX_Buf[6] = floatHex.Hex[1];
			Usart2_TX_Buf[7] = floatHex.Hex[0];
			
			floatHex.floatValue = Kinematics_Forward2pc.Linear_Y;//车体当前Y轴线速度 m/s
			Usart2_TX_Buf[8] = floatHex.Hex[3];
			Usart2_TX_Buf[9] = floatHex.Hex[2];
			Usart2_TX_Buf[10] = floatHex.Hex[1];
			Usart2_TX_Buf[11] = floatHex.Hex[0];
			
			floatHex.floatValue = Kinematics_Forward2pc.Angular_Z;//车体当前Z轴角速度 rad/s
			Usart2_TX_Buf[12] = floatHex.Hex[3];
			Usart2_TX_Buf[13] = floatHex.Hex[2];
			Usart2_TX_Buf[14] = floatHex.Hex[1];
			Usart2_TX_Buf[15] = floatHex.Hex[0];
			uint32Hex.uint32Value=getSysClock();								//Get current system time//获得当前系统时间
			Usart2_TX_Buf[16] = uint32Hex.Hex[3];
			Usart2_TX_Buf[17] = uint32Hex.Hex[2];
			Usart2_TX_Buf[18] = uint32Hex.Hex[1];
			Usart2_TX_Buf[19] = uint32Hex.Hex[0];
			int16Hex.int16Value = (int16_t)(Battery2pc.Voltage * 100);//电池电压值 10mv
			Usart2_TX_Buf[20]= int16Hex.Hex[1];//电池电压值 10mv
			Usart2_TX_Buf[21]= int16Hex.Hex[0];			
			Usart2_TX_Buf[22]= Battery2pc.Capacity;//电池电量值 %
			floatHex.floatValue = IMUdata2pc.Pitch;//俯仰角 rad
			Usart2_TX_Buf[23] = floatHex.Hex[3];
			Usart2_TX_Buf[24] = floatHex.Hex[2];
			Usart2_TX_Buf[25] = floatHex.Hex[1];
			Usart2_TX_Buf[26] = floatHex.Hex[0];
			
			floatHex.floatValue = IMUdata2pc.Yaw;//偏航角 rad
			Usart2_TX_Buf[27] = floatHex.Hex[3];
			Usart2_TX_Buf[28] = floatHex.Hex[2];
			Usart2_TX_Buf[29] = floatHex.Hex[1];
			Usart2_TX_Buf[30] = floatHex.Hex[0];
			
			floatHex.floatValue = IMUdata2pc.Roll;//翻滚角 rad
			Usart2_TX_Buf[31]= floatHex.Hex[3];
			Usart2_TX_Buf[32]= floatHex.Hex[2];
			Usart2_TX_Buf[33]= floatHex.Hex[1];
			Usart2_TX_Buf[34]= floatHex.Hex[0];
			
			floatHex.floatValue = IMUdata2pc.Acc_x;//加速度计X轴
			Usart2_TX_Buf[35]= floatHex.Hex[3];
			Usart2_TX_Buf[36]= floatHex.Hex[2];
			Usart2_TX_Buf[37]= floatHex.Hex[1];
			Usart2_TX_Buf[38]= floatHex.Hex[0];
			
			floatHex.floatValue = IMUdata2pc.Acc_y;//加速度计Y轴
			Usart2_TX_Buf[39]= floatHex.Hex[3];
			Usart2_TX_Buf[40]= floatHex.Hex[2];
			Usart2_TX_Buf[41]= floatHex.Hex[1];
			Usart2_TX_Buf[42]= floatHex.Hex[0];
			
			floatHex.floatValue = IMUdata2pc.Acc_z;//加速度计Z轴
			Usart2_TX_Buf[43]= floatHex.Hex[3];
			Usart2_TX_Buf[44]= floatHex.Hex[2];
			Usart2_TX_Buf[45]= floatHex.Hex[1];
			Usart2_TX_Buf[46]= floatHex.Hex[0];
			
			floatHex.floatValue = IMUdata2pc.Gyro_x;//陀螺仪X轴
			Usart2_TX_Buf[47]= floatHex.Hex[3];
			Usart2_TX_Buf[48]= floatHex.Hex[2];
			Usart2_TX_Buf[49]= floatHex.Hex[1];
			Usart2_TX_Buf[50]= floatHex.Hex[0];
			
			floatHex.floatValue = IMUdata2pc.Gyro_y;//陀螺仪Y轴
			Usart2_TX_Buf[51]= floatHex.Hex[3];
			Usart2_TX_Buf[52]= floatHex.Hex[2];
			Usart2_TX_Buf[53]= floatHex.Hex[1];
			Usart2_TX_Buf[54]= floatHex.Hex[0];
			
			floatHex.floatValue = IMUdata2pc.Gyro_z;//陀螺仪Z轴
			Usart2_TX_Buf[55]= floatHex.Hex[3];
			Usart2_TX_Buf[56]= floatHex.Hex[2];
			Usart2_TX_Buf[57]= floatHex.Hex[1];
			Usart2_TX_Buf[58]= floatHex.Hex[0];
			Usart2_TX_Buf[59] = 0;
			Usart2_TX_Buf[60] = 0;
			Usart2_TX_Buf[61] = 0;
			Usart2_TX_Buf[62] = 0;
			uint16_t SumCheck=0;//校验和
			uint8_t i = Usart2_TX_Buf[2] + 2;
			for(;(i--)>0;) SumCheck += Usart2_TX_Buf[i];//计算校验和
			Usart2_TX_Buf[63] = SumCheck;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)Usart2_TX_Buf, Usart2_TX_Buf[2] + 3);//串口DMA发送指令

}



