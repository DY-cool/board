#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "usart.h"
void setup(void);
void loop(void);
void ROS_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void ROS_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
