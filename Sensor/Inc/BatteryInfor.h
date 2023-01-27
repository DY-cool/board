#ifndef __BATTERYINFOR_H
#define __BATTERYINFOR_H
#include "main.h"
#include "LowPassFilter.h"
#ifdef __cplusplus
 extern "C" {
#endif
#define ADC1_DMA_BuffLEN 2//ADC1�ɼ�����DMA���泤��

typedef struct
{
	__IO uint16_t ADchx;//��ص�ѹADCֵ
	__IO uint16_t ADre; //STM32�ڲ����յ�ѹADCֵ
	__IO float Voltage; //��ص�ѹֵ
	__IO uint8_t Capacity;//��ص����ٷֱ� 0-100
} Battery_Struct;//���״̬��Ϣ�ṹ�����


extern __IO uint32_t ADC1_DMA_Buff[ADC1_DMA_BuffLEN];//ADC1�ɼ�����DMA����
void UpdateBatteryInfor(LPF_Struct*LPF_SupplyVoltage,Battery_Struct *Battery ,uint32_t *ADCx_DMA_Buff);//���µ��״̬��Ϣ
uint8_t GetdBatteryCapacity(float BatteryVoltag, uint8_t BatterySeriesNumber);//��������ʣ������ٷֱ�

void Batter_Init(void);
#ifdef __cplusplus
}
#endif

#endif
