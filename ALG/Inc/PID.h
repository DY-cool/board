#ifndef _PID_H_
#define _PID_H_
#include "main.h"
#include "VelocityControl.h"
#define PID_H (float)(COUNTS_PER_REV/(float)(60000/(float)VelocityControl_T))//PID����ת��

typedef struct//����ʽPID�ṹ�����
{
	 float Proportion; //�������� Proportional Const
	 float Integral; //���ֳ��� Integral Const
	 float Derivative; //΢�ֳ��� Derivative Const
	
	 float Error;//E(k)	
	 float PrevError;//E(k-1)
	 float LastError;//E(k-2)
} IncPID_Struct;

typedef struct//λ��ʽPID�ṹ�����
{
	 float Proportion; //�������� Proportional Const
	 float Integral; //���ֳ��� Integral Const
	 float Derivative; //΢�ֳ��� Derivative Const
	
	 float Error;//E(k)	
	 float PrevError;//E(k-1)
	
	 float IntegralError;//�ۻ����ֵ
	 float IntegralError_Min;//�ۻ����ֵ�޷���Сֵ
	 float IntegralError_Max;//�ۻ����ֵ�޷����ֵ
} PosPID_Struct;

void IncPID_Init(IncPID_Struct *PID, float P, float I, float D);//����ʽPID������ʼ��
void PosPID_Init(PosPID_Struct *PID, float P, float I, float D, float IntegralError_Min, float IntegralError_Max);//λ��ʽPID������ʼ��
float Incremental_PID(IncPID_Struct *PID, float NextPoint, float SetPoint);//����ʽPID���㺯��
float Position_PID(PosPID_Struct *PID, float NextPoint, float SetPoint);//λ��ʽPID���㺯��
void PIDvalue_Updata(IncPID_Struct *PID,float SetRPM,uint8_t number);
#endif
