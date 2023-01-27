#include "uTask.h"
#include "Remote.h"
#include "gpio.h"

Data_Struct uRCDatabuf;	//遥控器数据转存区
/**
	* 在线标志位递加计数检测在线函数
	*计数周期是1ms
*/
void OnLineCount(){
	//遥控在线计数器
	RC_CountorAdd();
	//上位机在线计数器
	
}



/**
	*数据接收解算函数
*/
void DataTreat_Receive(RosRemote* uBuff){

uint8_t RC_UpdataFlag=1;
/*-----------------判断遥控在线---------------------*/

		if(getRC_onlineFlag()==Lost)	//未接收到消息进入
		{
		uBuff->Linear_X=0.0f;	
		uBuff->Angular_Z=0.0f;
		uBuff->ROS_Schema=STATE_LOST;
		}
		else
		if(getRC_onlineFlag()==OnLion)
		{
			RC_UpdataFlag=RCdata_Updata(&uRCDatabuf);//遥控器数据读取
			if(RC_UpdataFlag==1)//遥控器数据解算未完成
			{
			uBuff->Linear_X=uBuff->last_Linear_X;
			uBuff->Angular_Z=uBuff->Angular_Z;
			uBuff->ROS_Schema=STATE_WAIT;
			}else
			if(RC_UpdataFlag==0)//遥控器解算完成
			{
				if(uRCDatabuf.Staion==STATE_RC)//遥控器控制
				{
				uBuff->Linear_X=uRCDatabuf.Linear_X;
				uBuff->Angular_Z=uRCDatabuf.Angular_Z;
				uBuff->last_Linear_X=uBuff->Linear_X;
				uBuff->last_Angular_Z=uBuff->Angular_Z;
				uBuff->ROS_Schema=STATE_RC;
				
				}else
				if(uRCDatabuf.Staion==STATE_UC)//上位机控制
				{
				uBuff->Linear_X=0;
				uBuff->Angular_Z=0;

				uBuff->ROS_Schema=STATE_UC;
				
				
				}else
				if(uRCDatabuf.Staion==STATE_REST){
					uBuff->Linear_X=0;
					uBuff->Angular_Z=0;
					uBuff->ROS_Schema=STATE_REST;
				}
			}		
		}
}


void RC_Data_decod(RosRemote*RC_data,RC_Mid*Mid){
	Mid->X	=(float)(RC_data->Linear_X)/660;
	Mid->Z	=(float)(RC_data->Angular_Z)/660;
}

uint16_t	UC_promptNum=0;
uint16_t	RC_promptNum=0;
uint16_t	REST_promptNum=0;



void	RC_prompt(uint8_t ID){

	if(ID==1){		//电脑控制
	RC_promptNum=0;
	REST_promptNum=0;
	UC_promptNum++;
	
	if(UC_promptNum<=100){
		HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);

	}else
	{
	HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	UC_promptNum=101;
	}
	
	
	}else
	if(ID==2){		//遥控器
	UC_promptNum=0;
	REST_promptNum=0;
	RC_promptNum++;
	
	if(RC_promptNum<=20){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
	}else
	if(RC_promptNum<=40){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	}else
	if(RC_promptNum<=60){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
	}else
	if(RC_promptNum<=80){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	}else
	if(RC_promptNum<=100){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
	}else{
	HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	RC_promptNum=120;
	
	}
	
	
	}else
	if(ID==3){		//掉线
		UC_promptNum=0;
		RC_promptNum=0;
		REST_promptNum=0;

	}else
	if(ID==4){	//遥控初始化
		UC_promptNum=0;
		RC_promptNum=0;
		REST_promptNum++;

	if(REST_promptNum<=10){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
	}else
	if(REST_promptNum<=20){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	}else
	if(REST_promptNum<=30){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
	}else
	if(REST_promptNum<=40){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	}else
	if(REST_promptNum<=50){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
	}else
	if(REST_promptNum<=60){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	}else
	if(REST_promptNum<=70){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
	}else
	if(REST_promptNum<=80){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	}else
	if(REST_promptNum<=90){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
	}else{
	HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
	REST_promptNum=100;
	
	}
	
	}
	

}


uint16_t RC_Line_prompt_counter=0;

uint8_t RC_Line_prompt(void){
	
		if(RC_Line_prompt_counter<=150){
					RC_Line_prompt_counter ++;
		if(RC_Line_prompt_counter<80){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
		}else
		if(RC_Line_prompt_counter<=90){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
		}else		if(RC_Line_prompt_counter<=100){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
		}else		if(RC_Line_prompt_counter<=110){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
		}else		if(RC_Line_prompt_counter<=120){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
		}else		if(RC_Line_prompt_counter<=130){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_SET);
		}else		if(RC_Line_prompt_counter<=140){
			HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);
			RC_Line_prompt_counter=160;
			return	1;
		}
		}else
		{
			return	1;
		}
		
		return 0;
		


}


