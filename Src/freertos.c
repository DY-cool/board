/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "MPU6050.h"
#include "IMU.h"
#include "Motor.h"
#include "Encoder.h"
#include "ROS_Shareware.h"
#include "uTask.h"
#include "LowPassFilter.h"
#include	"Kinematics_4WD.h"
#include	"VelocityControl.h"
#include "PID.h"
#include "math.h"
#include "BatteryInfor.h"
#include "usart.h"
#include "UpperComputer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/********************************移动任务******************************************/
#define USER_MOVE_VARIABLE_DBUG 0		//移动任务		参数    调试宏定义
#define USER_DBUG_WHEEL_RUN			0		//移动任务		PWM输出 调试宏定义
/********************************电池任务******************************************/
#define USER_BATTERY_VARIABLE_DBUG 0		//移动任务		参数    调试宏定义

/********************************数据传输任务******************************************/
#define USER_DATA_VARIABLE_DBUG 0		//移动任务		参数    调试宏定义
/********************************上位机传输任务******************************************/
#define USER_COMPUTER_VARIABLE_DBUG 0		//移动任务		参数    调试宏定义
/********************************底盘跟随模式******************************************/
#define USER_CLASS_FLOW_MODE 0		//移动任务		参数    调试宏定义
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Batter_Task_tHandle;
osThreadId IMUTask_tHandle;
osThreadId Move_base_Task_Handle;
osThreadId DataTreatTask04Handle;
osThreadId Data2PCTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   //按键消息队列的数量
#define RC_Q_NUM    1  		//遥控消息队列的数量  
QueueHandle_t RC_Queue;   		//遥控值消息队列句柄
#define MOVE_Q_NUM    1  		//底盘运动消息队列的数量  
QueueHandle_t MOVE_Queue;   		//底盘运动值消息队列句柄
#define IMU_Q_NUM    1  		//IMU消息队列的数量  
QueueHandle_t IMU_Queue;   		//IMU值消息队列句柄

#if USER_CLASS_FLOW_MODE==1
#define IMU_POSE_Q_NUM 1			//IMU解算姿态数据消息队列的数量
QueueHandle_t IMU_POSE_Queue;   		//IMU值消息队列句柄
#endif

#define BATTER_Q_NUM 1			//IMU解算姿态数据消息队列的数量
QueueHandle_t BATTER_Queue;   		//IMU值消息队列句柄

/* USER CODE END FunctionPrototypes */

void Batter_Task(void const * argument);
void IMUTask(void const * argument);
void Move_base_Task(void const * argument);
void DataTreat_Task(void const * argument);
void Data2PC_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  RC_Queue=xQueueCreate(RC_Q_NUM,sizeof(RosRemote));					//移动数据传输
  MOVE_Queue=xQueueCreate(MOVE_Q_NUM,sizeof(Kinematics_Struct));					//移动数据传输
  BATTER_Queue=xQueueCreate(BATTER_Q_NUM,sizeof(Battery_Struct));					//移动数据传输
  IMU_Queue=xQueueCreate(IMU_Q_NUM,sizeof(struct PCdataUp));	//IMU数据传输
	#if USER_CLASS_FLOW_MODE==1
  IMU_POSE_Queue=xQueueCreate(IMU_POSE_Q_NUM,sizeof(struct Attitude));	//IMU数据传输
	#endif
	/* add queues, ... */
	
	//sizeof(RosRemote*)
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Batter_Task_t */
  osThreadDef(Batter_Task_t, Batter_Task, osPriorityRealtime, 0, 128);
  Batter_Task_tHandle = osThreadCreate(osThread(Batter_Task_t), NULL);

  /* definition and creation of IMUTask_t */
  osThreadDef(IMUTask_t, IMUTask, osPriorityAboveNormal, 0, 256);
  IMUTask_tHandle = osThreadCreate(osThread(IMUTask_t), NULL);

  /* definition and creation of Move_base_Task_ */
  osThreadDef(Move_base_Task_, Move_base_Task, osPriorityIdle, 0, 1024);
  Move_base_Task_Handle = osThreadCreate(osThread(Move_base_Task_), NULL);

  /* definition and creation of DataTreatTask04 */
  osThreadDef(DataTreatTask04, DataTreat_Task, osPriorityHigh, 0, 256);
  DataTreatTask04Handle = osThreadCreate(osThread(DataTreatTask04), NULL);

  /* definition and creation of Data2PCTask */
  osThreadDef(Data2PCTask, Data2PC_Task, osPriorityHigh, 0, 256);
  Data2PCTaskHandle = osThreadCreate(osThread(Data2PCTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Batter_Task */
/**
  * @brief  Function implementing the Batter_Task_t thread.
  * @param  argument: Not used 
  * @retval None
  */
	#if USER_BATTERY_VARIABLE_DBUG==1
	Battery_Struct Battery={0,0,0,0};//电池状态信息结构体参数
#endif
/* USER CODE END Header_Batter_Task */
void Batter_Task(void const * argument)
{
  /* USER CODE BEGIN Batter_Task */
  /* Infinite loop */
	taskENTER_CRITICAL();//进入临界区	
	#if USER_BATTERY_VARIABLE_DBUG==0
	Battery_Struct Battery={0,0,0,0};//电池状态信息结构体参数
	#endif
	BaseType_t BATTER_err;
  LPF_Struct LPF_SupplyVoltage;//创建低通滤波电源电压值结构体参数
  LPF_Struct_Init(&LPF_SupplyVoltage,0.3);//低通滤波结构体参数初始化
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500; //运行周期
	taskEXIT_CRITICAL();//退出临界区

  for(;;)
  {
	
		xLastWakeTime=xTaskGetTickCount();  //获得当前系统时钟

		UpdateBatteryInfor(&LPF_SupplyVoltage,&Battery,(uint32_t *)ADC1_DMA_Buff);
		if(BATTER_Queue!=NULL)
		{
			BATTER_err=xQueueSend(BATTER_Queue,&Battery,1);
			if(BATTER_err==errQUEUE_FULL)
			{
			
			
			}
		
		}
		
		
		osDelayUntil(&xLastWakeTime,xFrequency);//500ms运行周期绝对周期 

  }
  /* USER CODE END Batter_Task */
}

/* USER CODE BEGIN Header_IMUTask */
/**
* @brief Function implementing the IMUTask_t thread.
* @param argument: Not used
* @retval None
*/	struct PCdataUp IMUdata_Mid;

/* USER CODE END Header_IMUTask */
void IMUTask(void const * argument)
{
  /* USER CODE BEGIN IMUTask */
  /* Infinite loop */
	taskENTER_CRITICAL();//进入临界区	
	BaseType_t IMU_err=errQUEUE_FULL;
	BaseType_t IMU_POSE_err=errQUEUE_FULL;
	#if USER_CLASS_FLOW_MODE==1
	struct Attitude Attitude={0};
	#endif
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 3; //运行周期
	taskEXIT_CRITICAL();//退出临界区

  for(;;)
  {
		xLastWakeTime=xTaskGetTickCount();  //获得当前系统时间
		PrepareForIMU(1);
		#if USER_CLASS_FLOW_MODE==1
		Attitude_Update(&Attitude);
		#endif
		IMU2PC_Updata(&IMUdata_Mid);//更新IMU数据上传到电脑
		
		if(IMU_Queue!=NULL){
				IMU_err=xQueueSend(IMU_Queue,&IMUdata_Mid,IMU_Q_NUM);
				if(IMU_err==errQUEUE_FULL){
					
				
				}
				
		
		}
	#if USER_CLASS_FLOW_MODE==1
		if(IMU_POSE_Queue!=NULL){
			IMU_POSE_err=xQueueSend(IMU_POSE_Queue,&Attitude,IMU_POSE_Q_NUM);
		}
	#endif
		
		osDelayUntil(&xLastWakeTime,xFrequency);//10ms运行周期绝对周期 
  }
  /* USER CODE END IMUTask */
}

/* USER CODE BEGIN Header_Move_base_Task */
/**
* @brief Function implementing the Move_base_Task_ thread.
* @param argument: Not used
* @retval None
*/
#define CONTROL_MASSAGE_HZ	200	//数据读取频率

/*Dbug*/
#if USER_MOVE_VARIABLE_DBUG ==1
	uint32_t previous_Massage_time=0;
	uint8_t	RC_UpdataFlag=0;
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 3; //运行周期
  RosRemote p_RC_data={.ROS_Schema=STATE_LOST,
                       .Linear_X=0,
                       .Linear_Y=0,
                       .Angular_Z=0};

  RC_Mid  RC={0,0};
  uint8_t QueueGetUpdataNum=0;  //队列出错计数位

  LPF_Struct LPF_Linear_X;//创建低通滤波车体X轴线速度结构体参数
  LPF_Struct LPF_Angular_Z;//创建低通滤波车体Z轴角速度结构体参数
  Kinematics_Struct Kinematics_Inverse;//创建车体运动学逆解结构体参数
  Kinematics_Struct Kinematics_Forward;//创建车体运动学正解结构体参数
  VelControl_Struct VelControl;//创建电机转速控制结构体参数

	float LPF_X=0,LPF_Y=0;
  Encoder_Struct Encoder[4]={{.Updata_Flag=0,.TIMx=0,.OverflowCount=0,
                              .CNT_Last=0,.CaptureCount=0,.Capture_D_Value=0}
                              ,{.Updata_Flag=0,.TIMx=0,.OverflowCount=0,
                              .CNT_Last=0,.CaptureCount=0,.Capture_D_Value=0}
                              ,{.Updata_Flag=0,.TIMx=0,.OverflowCount=0,
                              .CNT_Last=0,.CaptureCount=0,.Capture_D_Value=0}
                              ,{.Updata_Flag=0,.TIMx=0,.OverflowCount=0,
                              .CNT_Last=0,.CaptureCount=0,.Capture_D_Value=0}};

  int16_t D_Value_mid[4]={0,0,0,0};

   float PWM_M1=0;
	 float PWM_M2=0;
	 float PWM_M3=0;
	 float PWM_M4=0;

  IncPID_Struct PID_M1={0,0,0,0,0,0};//创建M1电机PID结构体参数
  IncPID_Struct PID_M2={0,0,0,0,0,0};//创建M2电机PID结构体参数
  IncPID_Struct PID_M3={0,0,0,0,0,0};//创建M3电机PID结构体参数
  IncPID_Struct PID_M4={0,0,0,0,0,0};//创建M4电机PID结构体参数
	uint8_t i = 0;
	
#endif

#if USER_DBUG_WHEEL_RUN==1
	float dbug_PWM1=0;
	float dbug_PWM2=0;
	float dbug_PWM3=0;
	float dbug_PWM4=0;


#endif
/* USER CODE END Header_Move_base_Task */
void Move_base_Task(void const * argument)
{
  /* USER CODE BEGIN Move_base_Task */
  /* Infinite loop */
	taskENTER_CRITICAL();//进入临界区	
	#if USER_MOVE_VARIABLE_DBUG==0
	uint32_t previous_Massage_time=0;
	uint8_t	RC_UpdataFlag=0;
	uint8_t IMU_POSE_UpdataFlag=0;
  BaseType_t MOVE_err;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 3; //运行周期
  RosRemote p_RC_data={.ROS_Schema=STATE_LOST,
                       .Linear_X=0,
                       .Linear_Y=0,
                       .Angular_Z=0};
	#if USER_CLASS_FLOW_MODE==1
	struct Attitude	Attitude={0};
	uint8_t IMUQueueGetUpdataNum=0;  //队列出错计数位
	#endif
  RC_Mid  RC={0,0};
  uint8_t QueueGetUpdataNum=0;  //队列出错计数位

  LPF_Struct LPF_Linear_X;//创建低通滤波车体X轴线速度结构体参数
  LPF_Struct LPF_Angular_Z;//创建低通滤波车体Z轴角速度结构体参数
	LPF_Struct_Init(&LPF_Linear_X, 0.05);//低通滤波结构体参数初始化
	LPF_Struct_Init(&LPF_Angular_Z, 0.05);//低通滤波结构体参数初始化
  Kinematics_Struct Kinematics_Inverse;//创建车体运动学逆解结构体参数
  Kinematics_Struct Kinematics_Forward;//创建车体运动学正解结构体参数
  VelControl_Struct VelControl;//创建电机转速控制结构体参数

  Encoder_Struct Encoder[4]={{.Updata_Flag=0,.TIMx=0,.OverflowCount=0,
                              .CNT_Last=0,.CaptureCount=0,.Capture_D_Value=0}
                              ,{.Updata_Flag=0,.TIMx=0,.OverflowCount=0,
                              .CNT_Last=0,.CaptureCount=0,.Capture_D_Value=0}
                              ,{.Updata_Flag=0,.TIMx=0,.OverflowCount=0,
                              .CNT_Last=0,.CaptureCount=0,.Capture_D_Value=0}
                              ,{.Updata_Flag=0,.TIMx=0,.OverflowCount=0,
                              .CNT_Last=0,.CaptureCount=0,.Capture_D_Value=0}};

  int16_t D_Value_mid[4]={0,0,0,0};

  static float PWM_M1=0;
	static float PWM_M2=0;
	static float PWM_M3=0;
	static float PWM_M4=0;

  IncPID_Struct PID_M1={0,0,0,0,0,0};//创建M1电机PID结构体参数
  IncPID_Struct PID_M2={0,0,0,0,0,0};//创建M2电机PID结构体参数
  IncPID_Struct PID_M3={0,0,0,0,0,0};//创建M3电机PID结构体参数
  IncPID_Struct PID_M4={0,0,0,0,0,0};//创建M4电机PID结构体参数
	static uint8_t i = 0;
	#endif
	#if	USER_MOVE_VARIABLE_DBUG==1
		LPF_Struct_Init(&LPF_Linear_X, 0.05);//低通滤波结构体参数初始化
	LPF_Struct_Init(&LPF_Angular_Z, 0.05);//低通滤波结构体参数初始化
	#endif
	taskEXIT_CRITICAL();//退出临界区

  for(;;)
  {
	

  xLastWakeTime=xTaskGetTickCount();  //获得当前系统时钟
	
	if((getSystick()-previous_Massage_time)>=(1000/CONTROL_MASSAGE_HZ)){
		//检查遥控器数据队列读取数据
    if(RC_Queue!=NULL)
		{
			if(xQueueReceive(RC_Queue,&p_RC_data,1)){
			//获得数据后执行
			RC_UpdataFlag=1;//遥控器刷星完成标志
			QueueGetUpdataNum=0;
			
			}
      else
      {
        QueueGetUpdataNum++;
      }

		}
    else{
      QueueGetUpdataNum=10;
    }
		#if USER_CLASS_FLOW_MODE==1
		if(IMU_POSE_Queue!=NULL)
		{
			if(xQueueReceive(IMU_POSE_Queue,&Attitude,1))
			{
			IMU_POSE_UpdataFlag=1;
			IMUQueueGetUpdataNum=0;
			}
			else
			{
			IMUQueueGetUpdataNum++;
			
			}
		}
		else
		{
		IMUQueueGetUpdataNum=10;
		}
		#endif
		
		
    /***************************电机控制速度解算**********************/
    if (RC_UpdataFlag==1)
    {
      //运行控制判断
      if (p_RC_data.ROS_Schema==STATE_RC)//遥控器控制
      {
        
        LPF_Linear_X.a = 0.05f;
				LPF_Angular_Z.a = 0.05f;

        RC_Data_decod(&p_RC_data,&RC);
				Kinematics_Inverse.Linear_X = RC.X * 0.8f;//左侧Y摇杆控制车体X轴线速度
				Kinematics_Inverse.Angular_Z = -RC.Z * 2.0f;//右侧X摇杆控制车体Z轴角速度
      }
      else
      if (p_RC_data.ROS_Schema==STATE_UC) //上位机控制
      {
        /***********上位机电脑控制**************/
				LPF_Linear_X.a = 0.05f;
				LPF_Angular_Z.a = 0.05f;

        RC_Data_decod(&p_RC_data,&RC);
				Kinematics_Inverse.Linear_X = RC.X * 0.8f;//左侧Y摇杆控制车体X轴线速度
				Kinematics_Inverse.Angular_Z = -RC.Z * 2.0f;//右侧X摇杆控制车体Z轴角速度

      }else
      if (p_RC_data.ROS_Schema==STATE_LOST)//遥控器掉线
      {//立即停车
  			LPF_Linear_X.a = 0.05f;
				LPF_Angular_Z.a = 0.05f;
        Kinematics_Inverse.Linear_X = 0;
				Kinematics_Inverse.Angular_Z = 0;
				RC.X=0.0f;
				RC.Z=0.0f;
				Kinematics_Forward.Angular_Z=0;
				Kinematics_Forward.Linear_X=0;
				Kinematics_Forward.Linear_Y=0;
				Kinematics_Forward.M1_RPM=0;
				Kinematics_Forward.M2_RPM=0;
				Kinematics_Forward.M3_RPM=0;
				Kinematics_Forward.M4_RPM=0;
				VelControl.M1_RPM=0;
				VelControl.M2_RPM=0;
				VelControl.M3_RPM=0;
				VelControl.M4_RPM=0;
				VelControl.M1_SetRPM=0;
				VelControl.M2_SetRPM=0;
				VelControl.M3_SetRPM=0;
				VelControl.M4_SetRPM=0;
				PWM_M1=0;
				PWM_M2=0;
				PWM_M3=0;
				PWM_M4=0;
				}
				else
				if(p_RC_data.ROS_Schema==STATE_REST){//遥控器初始化
				LPF_Linear_X.a = 0.05f;
				LPF_Angular_Z.a = 0.05f;
        Kinematics_Inverse.Linear_X = 0;
				Kinematics_Inverse.Angular_Z = 0;
				RC.X=0.0f;
				RC.Z=0.0f;
				Kinematics_Forward.Angular_Z=0;
				Kinematics_Forward.Linear_X=0;
				Kinematics_Forward.Linear_Y=0;
				Kinematics_Forward.M1_RPM=0;
				Kinematics_Forward.M2_RPM=0;
				Kinematics_Forward.M3_RPM=0;
				Kinematics_Forward.M4_RPM=0;
				VelControl.M1_RPM=0;
				VelControl.M2_RPM=0;
				VelControl.M3_RPM=0;
				VelControl.M4_RPM=0;
				VelControl.M1_SetRPM=0;
				VelControl.M2_SetRPM=0;
				VelControl.M3_SetRPM=0;
				VelControl.M4_SetRPM=0;
				PWM_M1=0;
				PWM_M2=0;
				PWM_M3=0;
				PWM_M4=0;
				
				}
		RC_UpdataFlag=0;	
    }
    else{
      if (QueueGetUpdataNum>=5){    //队列出错5次立即停车
        //立即停车
  			LPF_Linear_X.a = 0.05f;
				LPF_Angular_Z.a = 0.05f;
        Kinematics_Inverse.Linear_X = 0;
				Kinematics_Inverse.Angular_Z = 0;
      }
    }
		
		if(p_RC_data.ROS_Schema==STATE_LOST){
		  Motor_SpeedSET(M1,2000.0f);
			Motor_SpeedSET(M2,2000.0f);
			Motor_SpeedSET(M3,2000.0f);
			Motor_SpeedSET(M4,2000.0f);
		}else
		if(p_RC_data.ROS_Schema==STATE_REST){
		  Motor_SpeedSET(M1,2000.0f);
			Motor_SpeedSET(M2,2000.0f);
			Motor_SpeedSET(M3,2000.0f);
			Motor_SpeedSET(M4,2000.0f);

		}else
		{
				/***********************电机控制输出算法*******************/
				LPF_Linear_X.SampleValue = Kinematics_Inverse.Linear_X;//通过采样车体X轴目标线速度值给低通滤波器结构体中的采集值
				 Kinematics_Inverse.Linear_X = LowPassFilter(&LPF_Linear_X);//车体X轴目标线速度值经过低通滤波器后数值会更加平滑
					
				LPF_Angular_Z.SampleValue = Kinematics_Inverse.Angular_Z;//通过采样车体Z轴目标角速度值给低通滤波器结构体中的采集值
				Kinematics_Inverse.Angular_Z = LowPassFilter(&LPF_Angular_Z);//车体Z轴目标角速度值经过低通滤波器后数值会更加平滑

				Kinematics_4WD_CalculateRPM(&Kinematics_Inverse);//4WD运动学逆解函数
				VelControl.M1_SetRPM = -Kinematics_Inverse.M1_RPM;//转速值赋值
				VelControl.M2_SetRPM = Kinematics_Inverse.M2_RPM;//转速值赋值
				VelControl.M3_SetRPM = -Kinematics_Inverse.M3_RPM;//转速值赋值
				VelControl.M4_SetRPM = Kinematics_Inverse.M4_RPM;//转速值赋值
				//跟新编码器数值
				getEncoder1_value(&(Encoder[0]));
				getEncoder2_value(&(Encoder[1]));
				getEncoder3_value(&(Encoder[2]));
				getEncoder4_value(&(Encoder[3]));
				//获取编码器Capture_D_Value
				D_Value_mid[0]=-Encoder[0].Capture_D_Value;
				D_Value_mid[1]=Encoder[1].Capture_D_Value;
				D_Value_mid[2]=-Encoder[2].Capture_D_Value;
				D_Value_mid[3]=Encoder[3].Capture_D_Value;

				VelocityControl_4WD(D_Value_mid,&VelControl);

				//速度滤去低值抖动
				if( (fabs(VelControl.M1_SetRPM) < 0.005f) && (fabs(VelControl.M2_SetRPM) < 0.005f) && (fabs(VelControl.M3_SetRPM) < 0.005f) && (fabs(VelControl.M4_SetRPM) < 0.005f) )
				{
				PWM_M1 = PWM_M2 = PWM_M3 = PWM_M4 = 0;
				}

				//PID参数设置更新
				PIDvalue_Updata(&PID_M1,VelControl.M1_SetRPM,1);
				PIDvalue_Updata(&PID_M2,VelControl.M2_SetRPM,2);
				PIDvalue_Updata(&PID_M3,VelControl.M3_SetRPM,3);
				PIDvalue_Updata(&PID_M4,VelControl.M4_SetRPM,4);

				PWM_M1 += Incremental_PID(&PID_M1, -Encoder[0].Capture_D_Value, VelControl.M1_SetRPM * PID_H); //通过增量式PID计算获取M1电机PWM输出	
				PWM_M2 += Incremental_PID(&PID_M2, Encoder[1].Capture_D_Value, VelControl.M2_SetRPM * PID_H); //通过增量式PID计算获取M2电机PWM输出
				PWM_M3 += Incremental_PID(&PID_M3, -Encoder[2].Capture_D_Value, VelControl.M3_SetRPM * PID_H); //通过增量式PID计算获取M3电机PWM输出
				PWM_M4 += Incremental_PID(&PID_M4, Encoder[3].Capture_D_Value, VelControl.M4_SetRPM * PID_H); //通过增量式PID计算获取M4电机PWM输出

				/*------------------------PWM限值------------------------*/
				PWM_M1 = Constrain(PWM_M1, -MotorPWM_Max+1, MotorPWM_Max-1);
				PWM_M2 = Constrain(PWM_M2, -MotorPWM_Max+1, MotorPWM_Max-1);
				PWM_M3 = Constrain(PWM_M3, -MotorPWM_Max+1, MotorPWM_Max-1);
				PWM_M4 = Constrain(PWM_M4, -MotorPWM_Max+1, MotorPWM_Max-1);
				
				/****************设置电机PWM输出值*************************/
				#if USER_DBUG_WHEEL_RUN==0
				if(PWM_M1>=-0)
				{
					Motor_SpeedSET(M1,2000.0f-PWM_M1);
				}else
				if(PWM_M1<-0){
					Motor_SpeedSET(M1,(-2000.0f-PWM_M1));	
				}		
				
				if(PWM_M2>=-0)
				{
					Motor_SpeedSET(M2,2000.0f-PWM_M2);
				}else
				if(PWM_M2<-0){
					Motor_SpeedSET(M2,(-2000.0f-PWM_M2));	
				}	

				if(PWM_M3>=-0)
				{
					Motor_SpeedSET(M3,2000.0f-PWM_M3);
				}else
				if(PWM_M3<-0){
					Motor_SpeedSET(M3,(-2000.0f-PWM_M3));	
				}	

				if(PWM_M4>=-0)
				{
					Motor_SpeedSET(M4,2000.0f-PWM_M4);
				}else
				if(PWM_M4<-0){
					Motor_SpeedSET(M4,(-2000.0f-PWM_M4));	
				}
				#endif
				
				#if USER_DBUG_WHEEL_RUN==1
				if(dbug_PWM1>=-0)
				{
				Motor_SpeedSET(M1,2000.0f-dbug_PWM1);
				}else
				if(PWM_M1<-0){
				Motor_SpeedSET(M1,(-2000.0f-dbug_PWM1));	
				}		
				
				if(dbug_PWM2>=-0)
				{
				Motor_SpeedSET(M2,2000.0f-dbug_PWM2);
				}else
				if(dbug_PWM2<-0){
				Motor_SpeedSET(M2,(-2000.0f-dbug_PWM2));	
				}	

				if(dbug_PWM3>=-0)
				{
				Motor_SpeedSET(M3,2000.0f-dbug_PWM3);
				}else
				if(dbug_PWM3<-0){
				Motor_SpeedSET(M3,(-2000.0f-dbug_PWM3));	
				}	

				if(dbug_PWM4>=-0)
				{
				Motor_SpeedSET(M4,2000.0f-dbug_PWM4);
				}else
				if(dbug_PWM4<-0){
				Motor_SpeedSET(M4,(-2000.0f-dbug_PWM4));	
				}
				#endif	
				/************************逆解算将四轮速度转化位X和Z的速度*********************************/
				i++;
				Kinematics_Forward.M1_RPM += VelControl.M1_RPM;
				Kinematics_Forward.M2_RPM += VelControl.M2_RPM;
				Kinematics_Forward.M3_RPM += VelControl.M3_RPM;
				Kinematics_Forward.M4_RPM += VelControl.M4_RPM;
				if(i > 1) 
				{
					Kinematics_Forward.M1_RPM = (float)Kinematics_Forward.M1_RPM / i;
					Kinematics_Forward.M2_RPM = (float)Kinematics_Forward.M2_RPM / i;
					Kinematics_Forward.M3_RPM = (float)Kinematics_Forward.M3_RPM / i;
					Kinematics_Forward.M4_RPM = (float)Kinematics_Forward.M4_RPM / i;
					Kinematics_4WD_GetVelocities(&Kinematics_Forward);//4WD运动学正解函数
					if(MOVE_Queue!=NULL){
						MOVE_err=xQueueSend(MOVE_Queue,&Kinematics_Forward,1);
						if(MOVE_err==errQUEUE_FULL){
						
						}
					}
					i = Kinematics_Forward.M1_RPM = Kinematics_Forward.M2_RPM = Kinematics_Forward.M3_RPM = Kinematics_Forward.M4_RPM = 0;//清0
				}
				/****************************速度解算完成进行数据传输************************************/
				
				
			
			
			
		}
  }



		osDelayUntil(&xLastWakeTime,xFrequency);


  }
  /* USER CODE END Move_base_Task */
}

/* USER CODE BEGIN Header_DataTreat_Task */
/**
* @brief Function implementing the DataTreatTask04 thread.
* @param argument: Not used
* @retval None
*/

#if USER_DATA_VARIABLE_DBUG==1
	RosRemote p_RC_data={.ROS_Schema=STATE_LOST,0,0,0,0,0,0};
  struct PCdataUp IMUdata2PC={0,0,0,0,0,0,0,0,0};
	Kinematics_Struct Kinematics_Forward;//创建车体运动学正解结构体参数
	Battery_Struct PC_Battery={0,0,0,0};//电池状态信息结构体参数
	


#endif


#define DATABUFF_T_HZ	1000	//在线检测周期运行频率
#define DATABUFF_Send_HZ	200	//在线检测周期运行频率
#define IMUBuffReceive_HZ	100	//IMU数据发送频率

/* USER CODE END Header_DataTreat_Task */
void DataTreat_Task(void const * argument)
{
  /* USER CODE BEGIN DataTreat_Task */
	taskENTER_CRITICAL();//进入临界区	
	//遥控器
#if USER_DATA_VARIABLE_DBUG==0
	RosRemote p_RC_data={.ROS_Schema=STATE_LOST,0,0,0,0,0,0};
  struct PCdataUp IMUdata2PC={0,0,0,0,0,0,0,0,0};
	ParseData_Struct ParseData_Mast;//创建上位机解析协议结构体参数

#endif
	BaseType_t RC_err;
	
	uint32_t previous_DataBuff_T=0;	//在线检测周期运行插值
	uint32_t previous_DataBuffSend_T=0;	//在线检测周期运行插值
	Kinematics_Struct Kinematics_Forward;//创建车体运动学正解结构体参数
	Battery_Struct PC_Battery={0,0,0,0};//电池状态信息结构体参数
	PcTargetSpeed  TargetSpeed={0,0,0};
	uint8_t HeartbeatTime=20;
	taskEXIT_CRITICAL();//退出临界区

  /* Infinite loop */
  for(;;)
  {
	
	

		if((getSystick()-previous_DataBuff_T)>=(1000/DATABUFF_T_HZ)){
			OnLineCount();
			previous_DataBuff_T=getSystick();
		}
		
		/*************************提示可连接遥控器提示音*************/

		//数据发送
		if((getSystick()-previous_DataBuffSend_T)>=(1000/DATABUFF_Send_HZ))
		{
			if(RC_Line_prompt())//是否准许连接遥控提示完成后可以连接遥控或者上位机
			{
				if(RC_Queue!=NULL){
					DataTreat_Receive(&p_RC_data);
					/************启动遥控器时发出遥控器状态提示音*********/
					if(p_RC_data.ROS_Schema==STATE_UC){
					RC_prompt(1);
						if(PcTargetData(&TargetSpeed))
						{
						HeartbeatTime=0;
										
						}else
						{
						HeartbeatTime++;
						}
					
					if(HeartbeatTime<20){
						p_RC_data.Angular_Z=TargetSpeed.Angular_Z;
						p_RC_data.Linear_X=TargetSpeed.Linear_X;
						p_RC_data.Linear_Y=TargetSpeed.Linear_Y;
					}else
					{
					
					p_RC_data.Angular_Z=0.0f;
					p_RC_data.Linear_X=0.0f;
					p_RC_data.Linear_Y=0.0f;

					HeartbeatTime=30;
					}

					
					}else
					if(p_RC_data.ROS_Schema ==STATE_RC){
					RC_prompt(2);

					
					}else
					if(p_RC_data.ROS_Schema==STATE_LOST){
					RC_prompt(3);
					p_RC_data.Angular_Z=0.0f;
					p_RC_data.last_Angular_Z=0.0f;
					p_RC_data.last_Linear_X=0.0f;
					p_RC_data.last_Linear_Y=0.0f;
					p_RC_data.Linear_X=0.0f;
					p_RC_data.Linear_Y=0.0f;
					HAL_GPIO_WritePin(GPIOC,BEEP_Pin,GPIO_PIN_RESET);

					}else
					if(p_RC_data.ROS_Schema==STATE_REST){
					RC_prompt(4);
					p_RC_data.Angular_Z=0.0f;
					p_RC_data.last_Angular_Z=0.0f;
					p_RC_data.last_Linear_X=0.0f;
					p_RC_data.last_Linear_Y=0.0f;
					p_RC_data.Linear_X=0.0f;
					p_RC_data.Linear_Y=0.0f;
					}
	/******************************遥控器数据发送*************************/
					RC_err=xQueueSend(RC_Queue,&p_RC_data,1);
					if(RC_err==errQUEUE_FULL){
						
					}
				}
		
			}
			previous_DataBuffSend_T=getSystick();
		}
		
		if((p_RC_data.ROS_Schema==STATE_UC)||(p_RC_data.ROS_Schema ==STATE_RC)){
				//向电脑发送IMU数据更新
				if (IMU_Queue!=NULL)
				{
					if (xQueueReceive(IMU_Queue,&IMUdata2PC,10))
					{
						/*发送数据到上位机*/
						update_IMUdata(&IMUdata2PC);
						}
					}
				//向电脑发送移动数据更新
					if(MOVE_Queue!=NULL){
					if(xQueueReceive(MOVE_Queue,&Kinematics_Forward,1)){
					
							/*发送速度到上位机*/
							update_MOVEdata(&Kinematics_Forward);
						}
						else{
						
						}
					}
					//向电脑发送电量数据更新
					if(BATTER_Queue!=NULL){
						if(xQueueReceive(BATTER_Queue,&PC_Battery,1))
						{
							/*向电脑发送电量数据*/
							update_Batterydata(&PC_Battery);
						
						}
					
					}
		
			

			
		
		
		
		}
	
	
  }
  /* USER CODE END DataTreat_Task */
}

/* USER CODE BEGIN Header_Data2PC_Task */
/**
* @brief Function implementing the Data2PCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data2PC_Task */
void Data2PC_Task(void const * argument)
{
  /* USER CODE BEGIN Data2PC_Task */
		taskENTER_CRITICAL();//进入临界区	
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = 5; //运行周期

		taskEXIT_CRITICAL();//退出临界区

  /* Infinite loop */
  for(;;)
  {
	  xLastWakeTime=xTaskGetTickCount();  //获得当前系统时钟

		PC_msgUpdate();

		osDelayUntil(&xLastWakeTime,xFrequency);

  }
  /* USER CODE END Data2PC_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */




		 
		 
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
