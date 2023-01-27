#ifndef __ENCODER_H
#define __ENCODER_H
#include "main.h"

#ifdef __cplusplus
 extern "C" {
#endif
	 
typedef struct
{	
	 uint32_t CaptureCount;	//编码器捕获计数值
	 uint16_t OverflowCount ;//编码器溢出次数
	 int16_t  Capture_D_Value;//编码器前后2次捕获计数的差值
	 uint16_t CNT_Last;//缓存上一次的TIMx->CNT计数器值
	 uint8_t TIMx;//对应的定时器序号
	 uint8_t Updata_Flag;
}Encoder_Struct;//电机编码器结构体参数

void __HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void Encoder_UpdataValue(Encoder_Struct *Encoder, int8_t Signed);//更新编码器数值

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
