/**************************************************************************
文件版本：神炽焰 V1.0
文件功能：1、初始化编码器结构体参数
					2、更新编码器数值
					3、定时器溢出中断回调函数
**************************************************************************/
#include "Encoder.h"
#include "ROS_ShareWare.h"



void Encoder_Struct_Init(Encoder_Struct *Encoder, uint8_t TIMx);//初始化编码器结构体参数






uint16_t TIM2_OverflowCount;//定时器2溢出次数计数值
uint16_t TIM3_OverflowCount;//定时器3溢出次数计数值
uint16_t TIM4_OverflowCount;//定时器4溢出次数计数值
uint16_t TIM5_OverflowCount;//定时器5溢出次数计数值


Encoder_Struct Encoder_M1;//创建M1编码器结构体参数
Encoder_Struct Encoder_M2;//创建M2编码器结构体参数
Encoder_Struct Encoder_M3;//创建M3编码器结构体参数
Encoder_Struct Encoder_M4;//创建M4编码器结构体参数

/*-----------------------------------------------------------
函数功能：初始化编码器与定时器对应
入口参数：	编码器对应的定时器编号
返 回 值：none
说    明：参数“TIMx”用于指定结构体对应的定时器序号，数值范围为2~5，对应定时器TIM2~TIM5
-------------------------------------------------------------*/
void Encoder_StructM1_Init(uint8_t TIMx){
	Encoder_Struct_Init(&Encoder_M1,TIMx);
}
void Encoder_StructM2_Init(uint8_t TIMx){
	Encoder_Struct_Init(&Encoder_M2,TIMx);
}
void Encoder_StructM3_Init(uint8_t TIMx){
	Encoder_Struct_Init(&Encoder_M3,TIMx);
}
void Encoder_StructM4_Init(uint8_t TIMx){
	Encoder_Struct_Init(&Encoder_M4,TIMx);
}



void getEncoder1_value(Encoder_Struct * data){

	Encoder_UpdataValue(&Encoder_M1,-1);
	data->Capture_D_Value=Encoder_M1.Capture_D_Value;
	data->CaptureCount=Encoder_M1.CaptureCount;
	data->CNT_Last=Encoder_M1.CNT_Last;
	data->OverflowCount=Encoder_M1.OverflowCount;
	data->TIMx=Encoder_M1.TIMx;
	data->Updata_Flag=Encoder_M1.Updata_Flag;
}
void getEncoder2_value(Encoder_Struct * data){
	Encoder_UpdataValue(&Encoder_M2,-1);
	data->Capture_D_Value=Encoder_M2.Capture_D_Value;
	data->CaptureCount=Encoder_M2.CaptureCount;
	data->CNT_Last=Encoder_M2.CNT_Last;
	data->OverflowCount=Encoder_M2.OverflowCount;
	data->TIMx=Encoder_M2.TIMx;
	data->Updata_Flag=Encoder_M2.Updata_Flag;
}
void getEncoder3_value(Encoder_Struct * data){
	Encoder_UpdataValue(&Encoder_M3,-1);
	data->Capture_D_Value=Encoder_M3.Capture_D_Value;
	data->CaptureCount=Encoder_M3.CaptureCount;
	data->CNT_Last=Encoder_M3.CNT_Last;
	data->OverflowCount=Encoder_M3.OverflowCount;
	data->TIMx=Encoder_M3.TIMx;
	data->Updata_Flag=Encoder_M3.Updata_Flag;
}
void getEncoder4_value(Encoder_Struct * data){
	Encoder_UpdataValue(&Encoder_M4,1);
	data->Capture_D_Value=Encoder_M4.Capture_D_Value;
	data->CaptureCount=Encoder_M4.CaptureCount;
	data->CNT_Last=Encoder_M4.CNT_Last;
	data->OverflowCount=Encoder_M4.OverflowCount;
	data->TIMx=Encoder_M4.TIMx;
	data->Updata_Flag=Encoder_M4.Updata_Flag;
}
/*-----------------------------------------------------------
函数功能：初始化编码器结构体参数
入口参数：结构体参数，对应的定时器序号
返 回 值：None
说    明: 
参数“TIMx”用于指定结构体对应的定时器序号，数值范围为1~5，对应定时器TIM1~TIM5
 -----------------------------------------------------------*/
void Encoder_Struct_Init(Encoder_Struct *Encoder, uint8_t TIMx)
{
	Encoder->CaptureCount = 0;	//编码器捕获计数值
	Encoder->OverflowCount = 0;//编码器溢出次数
	Encoder->Capture_D_Value = 0;//编码器前后2次捕获计数的差值
	Encoder->CNT_Last = 0;//缓存上一次的TIMx->CNT计数器值
	Encoder->TIMx = TIMx;//对应的定时器序号
	Encoder->Updata_Flag=0;//更新标志位
	
	
	switch(TIMx)//清0对应定时器溢出次数计数值
	{
		case 2: TIM2_OverflowCount=0;break;
		case 3: TIM3_OverflowCount=0;break;
		case 4: TIM4_OverflowCount=0;break;
		case 5: TIM5_OverflowCount=0;break;
		default:break;
	}
}

/*-----------------------------------------------------------
函数功能：更新编码器数值
入口参数：结构体参数，符号参数
返 回 值：None
说    明: 
调用该函数后“Encoder->CaptureCount”与“Encoder->Capture_D_Value”会获得更新

函数传入的参数中“Signed”的数值为“1”或“-1”
该数值为1时，“Encoder->CaptureCount”与“Encoder->Capture_D_Value”的更新值重新乘以1
该数值为-1时，“Encoder->CaptureCount”与“Encoder->Capture_D_Value”的更新值重新乘以-1
 -----------------------------------------------------------*/
void Encoder_UpdataValue(Encoder_Struct *Encoder, int8_t Signed)
{

	Encoder->Updata_Flag=1;	//编码器更新标志位置1
  uint16_t Encoder_TIM;
	
	switch(Encoder->TIMx)//获取对应定时器的计数值与溢出次数计数值
	{		
		case 2:
		{
			Encoder_TIM = TIM2->CNT;//读取TIMx->CNT计数器值
			Encoder->OverflowCount = TIM2_OverflowCount;//定时器1溢出次数计数值
		}break;
		
		case 3:
		{
			Encoder_TIM = TIM3->CNT;//读取TIMx->CNT计数器值
			Encoder->OverflowCount = TIM3_OverflowCount;//定时器1溢出次数计数值
		}break;
		
		case 4:
		{
			Encoder_TIM = TIM4->CNT;//读取TIMx->CNT计数器值
			Encoder->OverflowCount = TIM4_OverflowCount;//定时器1溢出次数计数值
		}break;
		
		case 5:
		{
			Encoder_TIM = TIM5->CNT;//读取TIMx->CNT计数器值
			Encoder->OverflowCount = TIM5_OverflowCount;//定时器1溢出次数计数值
		}break;
		
		default: return;//退出函数
	}
	
	if(Encoder_TIM > Encoder->CNT_Last)
	{
		Encoder->Capture_D_Value = Encoder_TIM - Encoder->CNT_Last;//获取编码器前后2次捕获计数的差值
		if((uint16_t)Encoder->Capture_D_Value > 10000) Encoder->Capture_D_Value -= 65536;//判断是否溢出跳变
	}
	else
	{
		Encoder->Capture_D_Value = 0-(Encoder->CNT_Last - Encoder_TIM);//获取编码器前后2次捕获计数的差值
		if((uint16_t)Encoder->Capture_D_Value > 10000) Encoder->Capture_D_Value += 65536;//判断是否溢出跳变
	}
	Encoder->CNT_Last = Encoder_TIM;
	
	Encoder->Capture_D_Value = (Encoder->Capture_D_Value) * Signed;//获取编码器前后2次捕获计数的差值
	Encoder->CaptureCount = (Encoder->OverflowCount*65536 + Encoder_TIM) * Signed;//获取编码器捕获计数值
	Encoder->Updata_Flag=0;//编码器更新标志位置0
}

/*-----------------------------------------------------------
函数功能: 编码器计数溢出中断函数
输入参数: 定时器句柄
返 回 值: None
说    明: 编码器计数溢出后会进入该中断函数
 0xFFFF跳变到0x0000，溢出
 0x0000跳变到0xFFFF，溢出
 -----------------------------------------------------------*/
void __HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM2)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{
			TIM2_OverflowCount--;       //向下计数溢出
		}
		else
		{
			TIM2_OverflowCount++;  		 //向上计数溢出
		}
	}
	
	else if(htim->Instance == TIM3)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{
			TIM3_OverflowCount--;       //向下计数溢出
		}
		else
		{
			TIM3_OverflowCount++;  		 //向上计数溢出
		}
	}
	
	else if(htim->Instance == TIM4)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{
			TIM4_OverflowCount--;       //向下计数溢出
		}
		else
		{
			TIM4_OverflowCount++;  		 //向上计数溢出
		}
	}

	else if(htim->Instance == TIM5)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{
			TIM5_OverflowCount--;       //向下计数溢出
		}
		else
		{
			TIM5_OverflowCount++;  		 //向上计数溢出
		} 
	}
}
