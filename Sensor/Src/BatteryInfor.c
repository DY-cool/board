#include "BatteryInfor.h"
#include "LowPassFilter.h"
#include "adc.h"



#define R1_R2 ((float)(68+10)/10)//通过分压电阻计算分压系数
/*-----------------------------------------------------------
函数功能: 更新电池状态信息
输入参数: 电池参数结构体指针，ADC的DMA缓存数组指针
返 回 值: 
说    明: 调用该函数更新电池的电压、电量信息
 -----------------------------------------------------------*/
float VREFINT=1.627;//内部参照电压

void UpdateBatteryInfor(LPF_Struct*LPF_SupplyVoltage,Battery_Struct *Battery, uint32_t *ADCx_DMA_Buff)
{

	Battery->ADchx =  ADCx_DMA_Buff[0];//获取电池电压ADC值
	Battery->ADre  =  ADCx_DMA_Buff[1];//STM32内部参照电压ADC值
	
	LPF_SupplyVoltage->SampleValue = VREFINT * (float)Battery->ADchx / (float)Battery->ADre * R1_R2;//通过采样电阻分压换算出电池电压值赋值给低通滤波器结构体中的采集值
	Battery->Voltage = LowPassFilter(LPF_SupplyVoltage);//电池电压值经过低通滤波器后数值会更加稳定
	
	Battery->Capacity = GetdBatteryCapacity(Battery->Voltage, 4);//获取电池剩余电量百分比
}

/*-----------------------------------------------------------
函数功能: 计算电池组剩余电量百分比
输入参数: 当前电池组输出电压值，锂电池串联数
返 回 值: 当前电池组剩余电量百分比值
说    明: 电量百分比数值范围0-100，该函数仅对锂电池电量计算有效。
 -----------------------------------------------------------*/
 uint8_t BatteryCapacity;//锂电池组电量百分比值
 float SingleBatteryVoltag=0;
uint8_t GetdBatteryCapacity(float BatteryVoltag, uint8_t BatterySeriesNumber)
{
	 SingleBatteryVoltag = (float)BatteryVoltag / BatterySeriesNumber;//计算锂电池组单个锂电池电压值
	if(SingleBatteryVoltag >= 4.06)      BatteryCapacity=(((float)(SingleBatteryVoltag - 4.06)/0.14)*10+90);
	else if(SingleBatteryVoltag >= 3.98) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.98)/0.08)*10+80);
	else if(SingleBatteryVoltag >= 3.92) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.92)/0.06)*10+70);
	else if(SingleBatteryVoltag >= 3.87) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.87)/0.05)*10+60);
	else if(SingleBatteryVoltag >= 3.82) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.82)/0.05)*10+50);
	else if(SingleBatteryVoltag >= 3.79) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.79)/0.03)*10+40);
	else if(SingleBatteryVoltag >= 3.77) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.77)/0.02)*10+30);
	else if(SingleBatteryVoltag >= 3.74) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.74)/0.03)*10+20);
	else if(SingleBatteryVoltag >= 3.68) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.68)/0.06)*10+10);
	else if(SingleBatteryVoltag >= 3.45) BatteryCapacity=(((float)(SingleBatteryVoltag - 3.45)/0.23)*10+5);
	else if(SingleBatteryVoltag >= 3.0)  BatteryCapacity=(((float)(SingleBatteryVoltag - 3.0)/0.45)*10+0);
	else BatteryCapacity = 0;
	if(BatteryCapacity > 100) BatteryCapacity = 100;
	return BatteryCapacity;//电池电量
}

void Batter_Init(void){

	HAL_ADCEx_Calibration_Start(&hadc1),
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buff, ADC1_DMA_BuffLEN);  
	
	HAL_ADC_Start(&hadc1);//ADC开始采集ADC值


}
