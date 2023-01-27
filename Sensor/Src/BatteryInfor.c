#include "BatteryInfor.h"
#include "LowPassFilter.h"
#include "adc.h"



#define R1_R2 ((float)(68+10)/10)//ͨ����ѹ��������ѹϵ��
/*-----------------------------------------------------------
��������: ���µ��״̬��Ϣ
�������: ��ز����ṹ��ָ�룬ADC��DMA��������ָ��
�� �� ֵ: 
˵    ��: ���øú������µ�صĵ�ѹ��������Ϣ
 -----------------------------------------------------------*/
float VREFINT=1.627;//�ڲ����յ�ѹ

void UpdateBatteryInfor(LPF_Struct*LPF_SupplyVoltage,Battery_Struct *Battery, uint32_t *ADCx_DMA_Buff)
{

	Battery->ADchx =  ADCx_DMA_Buff[0];//��ȡ��ص�ѹADCֵ
	Battery->ADre  =  ADCx_DMA_Buff[1];//STM32�ڲ����յ�ѹADCֵ
	
	LPF_SupplyVoltage->SampleValue = VREFINT * (float)Battery->ADchx / (float)Battery->ADre * R1_R2;//ͨ�����������ѹ�������ص�ѹֵ��ֵ����ͨ�˲����ṹ���еĲɼ�ֵ
	Battery->Voltage = LowPassFilter(LPF_SupplyVoltage);//��ص�ѹֵ������ͨ�˲�������ֵ������ȶ�
	
	Battery->Capacity = GetdBatteryCapacity(Battery->Voltage, 4);//��ȡ���ʣ������ٷֱ�
}

/*-----------------------------------------------------------
��������: ��������ʣ������ٷֱ�
�������: ��ǰ����������ѹֵ��﮵�ش�����
�� �� ֵ: ��ǰ�����ʣ������ٷֱ�ֵ
˵    ��: �����ٷֱ���ֵ��Χ0-100���ú�������﮵�ص���������Ч��
 -----------------------------------------------------------*/
 uint8_t BatteryCapacity;//﮵��������ٷֱ�ֵ
 float SingleBatteryVoltag=0;
uint8_t GetdBatteryCapacity(float BatteryVoltag, uint8_t BatterySeriesNumber)
{
	 SingleBatteryVoltag = (float)BatteryVoltag / BatterySeriesNumber;//����﮵���鵥��﮵�ص�ѹֵ
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
	return BatteryCapacity;//��ص���
}

void Batter_Init(void){

	HAL_ADCEx_Calibration_Start(&hadc1),
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buff, ADC1_DMA_BuffLEN);  
	
	HAL_ADC_Start(&hadc1);//ADC��ʼ�ɼ�ADCֵ


}
