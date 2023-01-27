//
// Created by 10171 on 2022/1/2.
//

#ifndef BODER_ROS_SHAREWARE_H
#define BODER_ROS_SHAREWARE_H
#include "MPU6050.h"
#include "i2c.h"
#include "Encoder.h"
#define Constrain(AMT, MIN, MAX) ((AMT) < (MIN)? (MIN):( (AMT) > (MAX)?(MAX):(AMT) ))//约束函数




extern  __IO uint8_t I2C1_Tx_Flag;
extern  __IO uint8_t I2C1_Rx_Flag;
extern MPU6050_Struct MPU6050;//创建MPU6050结构体参数
extern DMA_HandleTypeDef hdma_uart4_rx;



void ClockInit(void );
uint32_t getSystick(void);
void Systick_accumulate(void);
void Queue_errPrint(char *errData);
void UsartReceive_IDLE(UART_HandleTypeDef *huart);//serial port idle interrupt function
uint32_t getSysClock(void);//Get system time

#endif //BODER_ROS_SHAREWARE_H
