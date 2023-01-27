#include "VelocityControl.h"
#include "PID.h"
#include "ROS_ShareWare.h"
#include "math.h"



/**************************************************************************
�������ܣ�4WD�ٶȱջ����ƺ���
��ڲ�����4WDĿ���ٶȣ��������������β�ֵ
����  ֵ��None
ע    �⣺
**************************************************************************/

void VelocityControl_4WD(int16_t*Capture_D_Value,VelControl_Struct*VelControl){//4WD�ٶȱջ����ƺ���
	
	/*-----------------------------�����ǰת��ֵ����-----------------------------*/
	VelControl->M1_RPM = Capture_D_Value[0] * 60000 / (float)VelocityControl_T / (float)COUNTS_PER_REV;//����M1�����ǰת��
	VelControl->M2_RPM = Capture_D_Value[1] * 60000 / (float)VelocityControl_T / (float)COUNTS_PER_REV;//����M2�����ǰת��
	VelControl->M3_RPM = Capture_D_Value[2] * 60000 / (float)VelocityControl_T / (float)COUNTS_PER_REV;//����M3�����ǰת��
	VelControl->M4_RPM = Capture_D_Value[3] * 60000 / (float)VelocityControl_T / (float)COUNTS_PER_REV;//����M4�����ǰת��

	/*-------�ԡ�Ŀ��ת�١�������ֵ-------*/	
	VelControl->M1_SetRPM = Constrain(VelControl->M1_SetRPM, -RPM_MAX, RPM_MAX);
	VelControl->M2_SetRPM = Constrain(VelControl->M2_SetRPM, -RPM_MAX, RPM_MAX);
	VelControl->M3_SetRPM = Constrain(VelControl->M3_SetRPM, -RPM_MAX, RPM_MAX);
	VelControl->M4_SetRPM = Constrain(VelControl->M4_SetRPM, -RPM_MAX, RPM_MAX);
}