#ifndef __LOWPASSFILTER_H
#define __LOWPASSFILTER_H
#include "main.h"

typedef struct
{
	__IO float a;// �˲�ϵ�� ȡֵ��Χ0~1
	__IO float OutValue;//�˲�������ֵ
	__IO float SampleValue;//����ֵ 
}LPF_Struct;

void LPF_Struct_Init(LPF_Struct *LPF, float a);//��ͨ�˲�������ʼ��
float LowPassFilter(LPF_Struct *LPF);//��ͨ�˲����㺯��

#endif
