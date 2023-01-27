#include "Motor.h"
#include "stm32f1xx.h"
#include "tim.h"

//���ת��������
#define M1_ENB(a)	\
						if (a)	\
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_SET);	\
						else		\
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_RESET)	
#define M2_ENB(a)	\
						if (a)	\
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);	\
						else		\
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET)			
#define M3_ENB(a)	\
						if (a)	\
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET);	\
						else		\
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_RESET)						
#define M4_ENB(a)	\
						if (a)	\
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_SET);\
						else		\
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_RESET)
//���ת��������
#define Motor1_PWM 	TIM8->CCR4 //���Ƶ��1 PWM���
#define	Motor2_PWM 	TIM8->CCR3 //���Ƶ��2 PWM���
#define	Motor3_PWM 	TIM8->CCR2 //���Ƶ��3 PWM���
#define	Motor4_PWM 	TIM8->CCR1 //���Ƶ��4 PWM���

void Moter1_SetForward(float Pwm){//���1����
	M1_ENB(1);
	Motor1_PWM=Pwm;
}
void Moter1_SetReverse(float Pwm){//���1��ת
	M1_ENB(0);
	Motor1_PWM=-Pwm;

}
void Moter2_SetForward(float Pwm){//���2����
	M2_ENB(1);
	Motor2_PWM=Pwm;
}
void Moter2_SetReverse(float Pwm){//���2��ת
	M2_ENB(0);
	Motor2_PWM=-Pwm;

}
void Moter3_SetForward(float Pwm){//���3����
	M3_ENB(1);
	Motor3_PWM=Pwm;
}
void Moter3_SetReverse(float Pwm){//���3��ת
	M3_ENB(0);
	Motor3_PWM=-Pwm;

}
void Moter4_SetForward(float Pwm){//���4����
	M4_ENB(1);
	Motor4_PWM=Pwm;
}
void Moter4_SetReverse(float Pwm){//���4��ת
	M4_ENB(0);
	Motor4_PWM=-Pwm;

}


void Motor_SpeedSET(enum MotorNum num,float pwm){
	if(num==M1){
	
	if(pwm>=0){
	Moter1_SetForward(pwm);
	}
	else{
	Moter1_SetReverse(pwm);
	}
	
	
	}else	if(num==M2){
	
	if(pwm>=0){
	Moter2_SetForward(pwm);
	}
	else{
	Moter2_SetReverse(pwm);
	}
	
	
	}else	if(num==M3){
	
	if(pwm>=0){
	Moter3_SetForward(pwm);
	}
	else{
	Moter3_SetReverse(pwm);
	}
	
	
	}else	if(num==M4){
	
	if(pwm>=0){
	Moter4_SetForward(pwm);
	}
	else{
	Moter4_SetReverse(pwm);
	}
	
	
	}

}
