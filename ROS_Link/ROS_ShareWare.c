//
// Created by 10171 on 2022/1/2.
//
#include "ROS_ShareWare.h"
#include "Remote.h"
#include "usart.h"
#include "Encoder.h"
#include "maincpp.h"
#include "UpperComputer.h"
__IO uint8_t I2C1_Tx_Flag=0;
__IO uint8_t I2C1_Rx_Flag=0;

MPU6050_Struct MPU6050;
volatile uint32_t RTOS_Counter=0;

/*-----------------------------------------------------------
��������: ���ڽ��տ����жϺ���
�������: ���ھ������ָ��
�� �� ֵ: None
˵    ��: ���ڽ��յ����ݺ�һ��byte�ĸߵ�ƽ(����)״̬�ͻᴥ�������жϽ���ú�����
					�û��ں����ж�ȡ���ڽ��յ�������
 -----------------------------------------------------------*/
void UsartReceive_IDLE(UART_HandleTypeDef *huart)//���ڽ��տ����жϺ���
{

		if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET))  
		{			
			/*------------Usart4-------------*/
			if(huart->Instance == UART4)
			{
				RemoteData_Write(huart,&hdma_uart4_rx);
				
			}
			else
			if(huart->Instance == USART2){
			PCData_Write(huart,&hdma_usart2_rx);
			}
		}

}



void ClockInit(void ){
	RTOS_Counter=0;

}
void Systick_accumulate(void){

	RTOS_Counter++;

}

uint32_t getSystick(void){

	return RTOS_Counter;

}


uint32_t getSysClock(void)//Get system time
{
	return RTOS_Counter;
}

void Queue_errPrint(char *errData){



}

