#ifndef __MOTOR_H_
#define __MOTOR_H_



#define MotorPWM_Max 2000 //PWM���ֵ
#define MotorPWM_Min 0 		//PWM��Сֵ

enum MotorNum{

	M1=0,
	M2=1,
	M3=2,
	M4=3,


};

void Motor_SpeedSET(enum MotorNum num,float pwm);




#endif