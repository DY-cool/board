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
	
	__IO float Linear_X;	//X�����ٶ� m/s
	__IO float Linear_Y;	//Y�����ٶ� m/s
	__IO float Angular_Z;	//Z����ٶ� rad/s	



}PcTargetSpeed;


typedef struct
{
	__IO uint16_t BufLenght;//���泤��
	__IO uint8_t* Buff;//�����׵�ַ
	__IO uint8_t Flag;//���ݽ�����־λ��1=������һ֡���ݣ�0=δ��������Ч����֡
	
	__IO uint8_t FE_Flag;//�յ�0xFE���� ��1����������һλ������0xEF��Ϊ�µ�֡ͷ��������0
	__IO uint8_t FH;//֡ͷ��־��0=֡ͷ��������1=֡ͷ����
	__IO uint8_t DataLenght;//��ǰ���ݳ���
}ParseData_Struct;

void ParseData_Init(ParseData_Struct *ParseData, uint16_t BufLeng);//�������ݲ�����ʼ��
void ParseDataFunction(ParseData_Struct *ParseData, uint8_t Data);//�������ݺ���

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

