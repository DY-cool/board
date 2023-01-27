#include "VelocityControl.h"
#include "PID.h"
#include "ROS_ShareWare.h"
#include "math.h"



/**************************************************************************
函数功能：4WD速度闭环控制函数
入口参数：4WD目标速度，编码器脉冲两次差值
返回  值：None
注    意：
**************************************************************************/

void VelocityControl_4WD(int16_t*Capture_D_Value,VelControl_Struct*VelControl){//4WD速度闭环控制函数
	
	/*-----------------------------电机当前转速值更新-----------------------------*/
	VelControl->M1_RPM = Capture_D_Value[0] * 60000 / (float)VelocityControl_T / (float)COUNTS_PER_REV;//更新M1电机当前转速
	VelControl->M2_RPM = Capture_D_Value[1] * 60000 / (float)VelocityControl_T / (float)COUNTS_PER_REV;//更新M2电机当前转速
	VelControl->M3_RPM = Capture_D_Value[2] * 60000 / (float)VelocityControl_T / (float)COUNTS_PER_REV;//更新M3电机当前转速
	VelControl->M4_RPM = Capture_D_Value[3] * 60000 / (float)VelocityControl_T / (float)COUNTS_PER_REV;//更新M4电机当前转速

	/*-------对“目标转速“进行限值-------*/	
	VelControl->M1_SetRPM = Constrain(VelControl->M1_SetRPM, -RPM_MAX, RPM_MAX);
	VelControl->M2_SetRPM = Constrain(VelControl->M2_SetRPM, -RPM_MAX, RPM_MAX);
	VelControl->M3_SetRPM = Constrain(VelControl->M3_SetRPM, -RPM_MAX, RPM_MAX);
	VelControl->M4_SetRPM = Constrain(VelControl->M4_SetRPM, -RPM_MAX, RPM_MAX);
}