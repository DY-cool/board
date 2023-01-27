#ifndef _VELOCITYCONTROL_H_
#define _VELOCITYCONTROL_H_

#include "main.h"


#define VelocityControl_T 5//�ٶȱջ��������� ms
#define RPM_MAX         170.0f //������ת�� rpm
#define COUNTS_PER_REV  144000 //��������תһȦ����������ֵ


typedef struct
{
	 float M1_RPM;	 //M1�����ǰת�� rpm
	 float M2_RPM;	 //M2�����ǰת�� rpm
	 float M3_RPM;	 //M3�����ǰת�� rpm
	 float M4_RPM;	 //M4�����ǰת�� rpm
	 float M1_SetRPM;//M1���Ŀ��ת�� rpm
	 float M2_SetRPM;//M2���Ŀ��ת�� rpm
	 float M3_SetRPM;//M3���Ŀ��ת�� rpm
	 float M4_SetRPM;//M4���Ŀ��ת�� rpm
}VelControl_Struct; //���ת�ٿ��ƽṹ�����

void VelocityControl_4WD(int16_t*Capture_D_Value,VelControl_Struct*VelControl);//4WD�ٶȱջ����ƺ���



#endif
