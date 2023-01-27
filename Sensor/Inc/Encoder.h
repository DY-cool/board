#ifndef __ENCODER_H
#define __ENCODER_H
#include "main.h"

#ifdef __cplusplus
 extern "C" {
#endif
	 
typedef struct
{	
	 uint32_t CaptureCount;	//�������������ֵ
	 uint16_t OverflowCount ;//�������������
	 int16_t  Capture_D_Value;//������ǰ��2�β�������Ĳ�ֵ
	 uint16_t CNT_Last;//������һ�ε�TIMx->CNT������ֵ
	 uint8_t TIMx;//��Ӧ�Ķ�ʱ�����
	 uint8_t Updata_Flag;
}Encoder_Struct;//����������ṹ�����

void __HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void Encoder_UpdataValue(Encoder_Struct *Encoder, int8_t Signed);//���±�������ֵ

void Encoder_StructM1_Init(uint8_t TIMx);
void Encoder_StructM2_Init(uint8_t TIMx);
void Encoder_StructM3_Init(uint8_t TIMx);
void Encoder_StructM4_Init(uint8_t TIMx);


void getEncoder1_value(Encoder_Struct * data);
void getEncoder2_value(Encoder_Struct * data);
void getEncoder3_value(Encoder_Struct * data);
void getEncoder4_value(Encoder_Struct * data);

#ifdef __cplusplus
}
#endif

#endif
