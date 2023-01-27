#ifndef _UPPERCOMPUTER_H_
#define _UPPERCOMPUTER_H_
#include "stm32f1xx_hal.h"
#include "main.h"
#include "IMU.h"
#include	"Kinematics_4WD.h"
#include "BatteryInfor.h"
#ifdef __cplusplus
 extern "C" {
#endif

union floatHex_union
{
	uint8_t Hex[4];
	float floatValue;
};

union int16Hex_union
{
	uint8_t Hex[2];
	int16_t int16Value;

};

union uint32Hex_union
{
	uint8_t Hex[4];
	uint32_t uint32Value;
};


typedef struct {
	
	__IO float Linear_X;	//X轴线速度 m/s
	__IO float Linear_Y;	//Y轴线速度 m/s
	__IO float Angular_Z;	//Z轴角速度 rad/s	



}PcTargetSpeed;


typedef struct
{
	__IO uint16_t BufLenght;//缓存长度
	__IO uint8_t* Buff;//缓存首地址
	__IO uint8_t Flag;//数据解析标志位，1=解析出一帧数据，0=未解析出有效数据帧
	
	__IO uint8_t FE_Flag;//收到0xFE数据 置1，紧接着下一位数据是0xEF则为新的帧头，否则置0
	__IO uint8_t FH;//帧头标志，0=帧头不完整、1=帧头完整
	__IO uint8_t DataLenght;//当前数据长度
}ParseData_Struct;

void ParseData_Init(ParseData_Struct *ParseData, uint16_t BufLeng);//解析数据参数初始化
void ParseDataFunction(ParseData_Struct *ParseData, uint8_t Data);//解析数据函数

void update_Batterydata(Battery_Struct *data);
void update_MOVEdata(Kinematics_Struct *data);
void update_IMUdata(struct PCdataUp *data);
uint8_t PcTargetData(PcTargetSpeed *data );
void Sendf2PcDataCom(void);
void PCData_Write(UART_HandleTypeDef *huart,DMA_HandleTypeDef* dma);
void PC_msgUpdate();
#ifdef __cplusplus
}
#endif

#endif

