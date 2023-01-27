#ifndef __BATTERYINFOR_H
#define __BATTERYINFOR_H
#include "main.h"
#include "LowPassFilter.h"
#ifdef __cplusplus
 extern "C" {
#endif
#define ADC1_DMA_BuffLEN 2//ADC1采集数据DMA缓存长度

typedef struct
{
	__IO uint16_t ADchx;//电池电压ADC值
	__IO uint16_t ADre; //STM32内部参照电压ADC值
	__IO float Voltage; //电池电压值
	__IO uint8_t Capacity;//电池电量百分比 0-100
} Battery_Struct;//电池状态信息结构体参数


extern __IO uint32_t ADC1_DMA_Buff[ADC1_DMA_BuffLEN];//ADC1采集数据DMA缓存
void UpdateBatteryInfor(LPF_Struct*LPF_SupplyVoltage,Battery_Struct *Battery ,uint32_t *ADCx_DMA_Buff);//更新电池状态信息
uint8_t GetdBatteryCapacity(float BatteryVoltag, uint8_t BatterySeriesNumber);//计算电池组剩余电量百分比

void Batter_Init(void);
#ifdef __cplusplus
}
#endif

#endif
