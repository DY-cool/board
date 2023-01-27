#ifndef _VELOCITYCONTROL_H_
#define _VELOCITYCONTROL_H_

#include "main.h"


#define VelocityControl_T 5//速度闭环控制周期 ms
#define RPM_MAX         170.0f //电机最大转速 rpm
#define COUNTS_PER_REV  144000 //电机输出轴转一圈编码器计数值


typedef struct
{
	 float M1_RPM;	 //M1电机当前转速 rpm
	 float M2_RPM;	 //M2电机当前转速 rpm
	 float M3_RPM;	 //M3电机当前转速 rpm
	 float M4_RPM;	 //M4电机当前转速 rpm
	 float M1_SetRPM;//M1电机目标转速 rpm
	 float M2_SetRPM;//M2电机目标转速 rpm
	 float M3_SetRPM;//M3电机目标转速 rpm
	 float M4_SetRPM;//M4电机目标转速 rpm
}VelControl_Struct; //电机转速控制结构体参数

void VelocityControl_4WD(int16_t*Capture_D_Value,VelControl_Struct*VelControl);//4WD速度闭环控制函数



#endif
