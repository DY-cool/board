#ifndef _PID_H_
#define _PID_H_
#include "main.h"
#include "VelocityControl.h"
#define PID_H (float)(COUNTS_PER_REV/(float)(60000/(float)VelocityControl_T))//PID参数转化

typedef struct//增量式PID结构体参数
{
	 float Proportion; //比例常数 Proportional Const
	 float Integral; //积分常数 Integral Const
	 float Derivative; //微分常数 Derivative Const
	
	 float Error;//E(k)	
	 float PrevError;//E(k-1)
	 float LastError;//E(k-2)
} IncPID_Struct;

typedef struct//位置式PID结构体参数
{
	 float Proportion; //比例常数 Proportional Const
	 float Integral; //积分常数 Integral Const
	 float Derivative; //微分常数 Derivative Const
	
	 float Error;//E(k)	
	 float PrevError;//E(k-1)
	
	 float IntegralError;//累积误差值
	 float IntegralError_Min;//累积误差值限幅最小值
	 float IntegralError_Max;//累积误差值限幅最大值
} PosPID_Struct;

void IncPID_Init(IncPID_Struct *PID, float P, float I, float D);//增量式PID参数初始化
void PosPID_Init(PosPID_Struct *PID, float P, float I, float D, float IntegralError_Min, float IntegralError_Max);//位置式PID参数初始化
float Incremental_PID(IncPID_Struct *PID, float NextPoint, float SetPoint);//增量式PID计算函数
float Position_PID(PosPID_Struct *PID, float NextPoint, float SetPoint);//位置式PID计算函数
void PIDvalue_Updata(IncPID_Struct *PID,float SetRPM,uint8_t number);
#endif
