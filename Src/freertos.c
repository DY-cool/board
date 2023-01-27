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
/********************************�ƶ�����******************************************/
#define USER_MOVE_VARIABLE_DBUG 0		//�ƶ�����		����    ���Ժ궨��
#define USER_DBUG_WHEEL_RUN			0		//�ƶ�����		PWM��� ���Ժ궨��
/********************************�������******************************************/
#define USER_BATTERY_VARIABLE_DBUG 0		//�ƶ�����		����    ���Ժ궨��

/********************************���ݴ�������******************************************/
#define USER_DATA_VARIABLE_DBUG 0		//�ƶ�����		����    ���Ժ궨��
/********************************��λ����������******************************************/
#define USER_COMPUTER_VARIABLE_DBUG 0		//�ƶ�����		����    ���Ժ궨��
/********************************���̸���ģʽ******************************************/
#define USER_CLASS_FLOW_MODE 0		//�ƶ�����		����    ���Ժ궨��
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
   //������Ϣ���е�����
#define RC_Q_NUM    1  		//ң����Ϣ���е�����  
QueueHandle_t RC_Queue;   		//ң��ֵ��Ϣ���о��
#define MOVE_Q_NUM    1  		//�����˶���Ϣ���е�����  
QueueHandle_t MOVE_Queue;   		//�����˶�ֵ��Ϣ���о��
#define IMU_Q_NUM    1  		//IMU��Ϣ���е�����  
QueueHandle_t IMU_Queue;   		//IMUֵ��Ϣ���о��

#if USER_CLASS_FLOW_MODE==1
#define IMU_POSE_Q_NUM 1			//IMU������̬������Ϣ���е�����
QueueHandle_t IMU_POSE_Queue;   		//IMUֵ��Ϣ���о��
#endif

#define BATTER_Q_NUM 1			//IMU������̬������Ϣ���е�����
QueueHandle_t BATTER_Queue;   		//IMUֵ��Ϣ���о��

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
  RC_Queue=xQueueCreate(RC_Q_NUM,sizeof(RosRemote));					//�ƶ����ݴ���
  MOVE_Queue=xQueueCreate(MOVE_Q_NUM,sizeof(Kinematics_Struct));					//�ƶ����ݴ���
  BATTER_Queue=xQueueCreate(BATTER_Q_NUM,sizeof(Battery_Struct));					//�ƶ����ݴ���
  IMU_Queue=xQueueCreate(IMU_Q_NUM,sizeof(struct PCdataUp));	//IMU���ݴ���
	#if USER_CLASS_FLOW_MODE==1
  IMU_POSE_Queue=xQueueCreate(IMU_POSE_Q_NUM,sizeof(struct Attitude));	//IMU���ݴ���
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
	Battery_Struct Battery={0,0,0,0};//���״̬��Ϣ�ṹ�����
#endif
/* USER CODE END Header_Batter_Task */
void Batter_Task(void const * argument)
{
  /* USER CODE BEGIN Batter_Task */
  /* Infinite loop */
	taskENTER_CRITICAL();//�����ٽ���	
	#if USER_BATTERY_VARIABLE_DBUG==0
	Battery_Struct Battery={0,0,0,0};//���״̬��Ϣ�ṹ�����
	#endif
	BaseType_t BATTER_err;
  LPF_Struct LPF_SupplyVoltage;//������ͨ�˲���Դ��ѹֵ�ṹ�����
  LPF_Struct_Init(&LPF_SupplyVoltage,0.3);//��ͨ�˲��ṹ�������ʼ��
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500; //��������
	taskEXIT_CRITICAL();//�˳��ٽ���

  for(;;)
  {
	
		xLastWakeTime=xTaskGetTickCount();  //��õ�ǰϵͳʱ��

		UpdateBatteryInfor(&LPF_SupplyVoltage,&Battery,(uint32_t *)ADC1_DMA_Buff);
		if(BATTER_Queue!=NULL)
		{
			BATTER_err=xQueueSend(BATTER_Queue,&Battery,1);
			if(BATTER_err==errQUEUE_FULL)
			{
			
			
			}
		
		}
		
		
		osDelayUntil(&xLastWakeTime,xFrequency);//500ms�������ھ������� 

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
	taskENTER_CRITICAL();//�����ٽ���	
	BaseType_t IMU_err=errQUEUE_FULL;
	BaseType_t IMU_POSE_err=errQUEUE_FULL;
	#if USER_CLASS_FLOW_MODE==1
	struct Attitude Attitude={0};
	#endif
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 3; //��������
	taskEXIT_CRITICAL();//�˳��ٽ���

  for(;;)
  {
		xLastWakeTime=xTaskGetTickCount();  //��õ�ǰϵͳʱ��
		PrepareForIMU(1);
		#if USER_CLASS_FLOW_MODE==1
		Attitude_Update(&Attitude);
		#endif
		IMU2PC_Updata(&IMUdata_Mid);//����IMU�����ϴ�������
		
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
		
		osDelayUntil(&xLastWakeTime,xFrequency);//10ms�������ھ������� 
  }
  /* USER CODE END IMUTask */
}

/* USER CODE BEGIN Header_Move_base_Task */
/**
* @brief Function implementing the Move_base_Task_ thread.
* @param argument: Not used
* @retval None
*/
#define CONTROL_MASSAGE_HZ	200	//���ݶ�ȡƵ��

/*Dbug*/
#if USER_MOVE_VARIABLE_DBUG ==1
	uint32_t previous_Massage_time=0;
	uint8_t	RC_UpdataFlag=0;
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 3; //��������
  RosRemote p_RC_data={.ROS_Schema=STATE_LOST,
                       .Linear_X=0,
                       .Linear_Y=0,
                       .Angular_Z=0};

  RC_Mid  RC={0,0};
  uint8_t QueueGetUpdataNum=0;  //���г������λ

  LPF_Struct LPF_Linear_X;//������ͨ�˲�����X�����ٶȽṹ�����
  LPF_Struct LPF_Angular_Z;//������ͨ�˲�����Z����ٶȽṹ�����
  Kinematics_Struct Kinematics_Inverse;//���������˶�ѧ���ṹ�����
  Kinematics_Struct Kinematics_Forward;//���������˶�ѧ����ṹ�����
  VelControl_Struct VelControl;//�������ת�ٿ��ƽṹ�����

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

  IncPID_Struct PID_M1={0,0,0,0,0,0};//����M1���PID�ṹ�����
  IncPID_Struct PID_M2={0,0,0,0,0,0};//����M2���PID�ṹ�����
  IncPID_Struct PID_M3={0,0,0,0,0,0};//����M3���PID�ṹ�����
  IncPID_Struct PID_M4={0,0,0,0,0,0};//����M4���PID�ṹ�����
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
	taskENTER_CRITICAL();//�����ٽ���	
	#if USER_MOVE_VARIABLE_DBUG==0
	uint32_t previous_Massage_time=0;
	uint8_t	RC_UpdataFlag=0;
	uint8_t IMU_POSE_UpdataFlag=0;
  BaseType_t MOVE_err;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 3; //��������
  RosRemote p_RC_data={.ROS_Schema=STATE_LOST,
                       .Linear_X=0,
                       .Linear_Y=0,
                       .Angular_Z=0};
	#if USER_CLASS_FLOW_MODE==1
	struct Attitude	Attitude={0};
	uint8_t IMUQueueGetUpdataNum=0;  //���г������λ
	#endif
  RC_Mid  RC={0,0};
  uint8_t QueueGetUpdataNum=0;  //���г������λ

  LPF_Struct LPF_Linear_X;//������ͨ�˲�����X�����ٶȽṹ�����
  LPF_Struct LPF_Angular_Z;//������ͨ�˲�����Z����ٶȽṹ�����
	LPF_Struct_Init(&LPF_Linear_X, 0.05);//��ͨ�˲��ṹ�������ʼ��
	LPF_Struct_Init(&LPF_Angular_Z, 0.05);//��ͨ�˲��ṹ�������ʼ��
  Kinematics_Struct Kinematics_Inverse;//���������˶�ѧ���ṹ�����
  Kinematics_Struct Kinematics_Forward;//���������˶�ѧ����ṹ�����
  VelControl_Struct VelControl;//�������ת�ٿ��ƽṹ�����

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

  IncPID_Struct PID_M1={0,0,0,0,0,0};//����M1���PID�ṹ�����
  IncPID_Struct PID_M2={0,0,0,0,0,0};//����M2���PID�ṹ�����
  IncPID_Struct PID_M3={0,0,0,0,0,0};//����M3���PID�ṹ�����
  IncPID_Struct PID_M4={0,0,0,0,0,0};//����M4���PID�ṹ�����
	static uint8_t i = 0;
	#endif
	#if	USER_MOVE_VARIABLE_DBUG==1
		LPF_Struct_Init(&LPF_Linear_X, 0.05);//��ͨ�˲��ṹ�������ʼ��
	LPF_Struct_Init(&LPF_Angular_Z, 0.05);//��ͨ�˲��ṹ�������ʼ��
	#endif
	taskEXIT_CRITICAL();//�˳��ٽ���

  for(;;)
  {
	

  xLastWakeTime=xTaskGetTickCount();  //��õ�ǰϵͳʱ��
	
	if((getSystick()-previous_Massage_time)>=(1000/CONTROL_MASSAGE_HZ)){
		//���ң�������ݶ��ж�ȡ����
    if(RC_Queue!=NULL)
		{
			if(xQueueReceive(RC_Queue,&p_RC_data,1)){
			//������ݺ�ִ��
			RC_UpdataFlag=1;//ң����ˢ����ɱ�־
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
		
		
    /***************************��������ٶȽ���**********************/
    if (RC_UpdataFlag==1)
    {
      //���п����ж�
      if (p_RC_data.ROS_Schema==STATE_RC)//ң��������
      {
        
        LPF_Linear_X.a = 0.05f;
				LPF_Angular_Z.a = 0.05f;

        RC_Data_decod(&p_RC_data,&RC);
				Kinematics_Inverse.Linear_X = RC.X * 0.8f;//���Yҡ�˿��Ƴ���X�����ٶ�
				Kinematics_Inverse.Angular_Z = -RC.Z * 2.0f;//�Ҳ�Xҡ�˿��Ƴ���Z����ٶ�
      }
      else
      if (p_RC_data.ROS_Schema==STATE_UC) //��λ������
      {
        /***********��λ�����Կ���**************/
				LPF_Linear_X.a = 0.05f;
				LPF_Angular_Z.a = 0.05f;

        RC_Data_decod(&p_RC_data,&RC);
				Kinematics_Inverse.Linear_X = RC.X * 0.8f;//���Yҡ�˿��Ƴ���X�����ٶ�
				Kinematics_Inverse.Angular_Z = -RC.Z * 2.0f;//�Ҳ�Xҡ�˿��Ƴ���Z����ٶ�

      }else
      if (p_RC_data.ROS_Schema==STATE_LOST)//ң��������
      {//����ͣ��
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
				if(p_RC_data.ROS_Schema==STATE_REST){//ң������ʼ��
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
      if (QueueGetUpdataNum>=5){    //���г���5������ͣ��
        //����ͣ��
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
				/***********************�����������㷨*******************/
				LPF_Linear_X.SampleValue = Kinematics_Inverse.Linear_X;//ͨ����������X��Ŀ�����ٶ�ֵ����ͨ�˲����ṹ���еĲɼ�ֵ
				 Kinematics_Inverse.Linear_X = LowPassFilter(&LPF_Linear_X);//����X��Ŀ�����ٶ�ֵ������ͨ�˲�������ֵ�����ƽ��
					
				LPF_Angular_Z.SampleValue = Kinematics_Inverse.Angular_Z;//ͨ����������Z��Ŀ����ٶ�ֵ����ͨ�˲����ṹ���еĲɼ�ֵ
				Kinematics_Inverse.Angular_Z = LowPassFilter(&LPF_Angular_Z);//����Z��Ŀ����ٶ�ֵ������ͨ�˲�������ֵ�����ƽ��

				Kinematics_4WD_CalculateRPM(&Kinematics_Inverse);//4WD�˶�ѧ��⺯��
				VelControl.M1_SetRPM = -Kinematics_Inverse.M1_RPM;//ת��ֵ��ֵ
				VelControl.M2_SetRPM = Kinematics_Inverse.M2_RPM;//ת��ֵ��ֵ
				VelControl.M3_SetRPM = -Kinematics_Inverse.M3_RPM;//ת��ֵ��ֵ
				VelControl.M4_SetRPM = Kinematics_Inverse.M4_RPM;//ת��ֵ��ֵ
				//���±�������ֵ
				getEncoder1_value(&(Encoder[0]));
				getEncoder2_value(&(Encoder[1]));
				getEncoder3_value(&(Encoder[2]));
				getEncoder4_value(&(Encoder[3]));
				//��ȡ������Capture_D_Value
				D_Value_mid[0]=-Encoder[0].Capture_D_Value;
				D_Value_mid[1]=Encoder[1].Capture_D_Value;
				D_Value_mid[2]=-Encoder[2].Capture_D_Value;
				D_Value_mid[3]=Encoder[3].Capture_D_Value;

				VelocityControl_4WD(D_Value_mid,&VelControl);

				//�ٶ���ȥ��ֵ����
				if( (fabs(VelControl.M1_SetRPM) < 0.005f) && (fabs(VelControl.M2_SetRPM) < 0.005f) && (fabs(VelControl.M3_SetRPM) < 0.005f) && (fabs(VelControl.M4_SetRPM) < 0.005f) )
				{
				PWM_M1 = PWM_M2 = PWM_M3 = PWM_M4 = 0;
				}

				//PID�������ø���
				PIDvalue_Updata(&PID_M1,VelControl.M1_SetRPM,1);
				PIDvalue_Updata(&PID_M2,VelControl.M2_SetRPM,2);
				PIDvalue_Updata(&PID_M3,VelControl.M3_SetRPM,3);
				PIDvalue_Updata(&PID_M4,VelControl.M4_SetRPM,4);

				PWM_M1 += Incremental_PID(&PID_M1, -Encoder[0].Capture_D_Value, VelControl.M1_SetRPM * PID_H); //ͨ������ʽPID�����ȡM1���PWM���	
				PWM_M2 += Incremental_PID(&PID_M2, Encoder[1].Capture_D_Value, VelControl.M2_SetRPM * PID_H); //ͨ������ʽPID�����ȡM2���PWM���
				PWM_M3 += Incremental_PID(&PID_M3, -Encoder[2].Capture_D_Value, VelControl.M3_SetRPM * PID_H); //ͨ������ʽPID�����ȡM3���PWM���
				PWM_M4 += Incremental_PID(&PID_M4, Encoder[3].Capture_D_Value, VelControl.M4_SetRPM * PID_H); //ͨ������ʽPID�����ȡM4���PWM���

				/*------------------------PWM��ֵ------------------------*/
				PWM_M1 = Constrain(PWM_M1, -MotorPWM_Max+1, MotorPWM_Max-1);
				PWM_M2 = Constrain(PWM_M2, -MotorPWM_Max+1, MotorPWM_Max-1);
				PWM_M3 = Constrain(PWM_M3, -MotorPWM_Max+1, MotorPWM_Max-1);
				PWM_M4 = Constrain(PWM_M4, -MotorPWM_Max+1, MotorPWM_Max-1);
				
				/****************���õ��PWM���ֵ*************************/
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
				/************************����㽫�����ٶ�ת��λX��Z���ٶ�*********************************/
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
					Kinematics_4WD_GetVelocities(&Kinematics_Forward);//4WD�˶�ѧ���⺯��
					if(MOVE_Queue!=NULL){
						MOVE_err=xQueueSend(MOVE_Queue,&Kinematics_Forward,1);
						if(MOVE_err==errQUEUE_FULL){
						
						}
					}
					i = Kinematics_Forward.M1_RPM = Kinematics_Forward.M2_RPM = Kinematics_Forward.M3_RPM = Kinematics_Forward.M4_RPM = 0;//��0
				}
				/****************************�ٶȽ�����ɽ������ݴ���************************************/
				
				
			
			
			
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
	Kinematics_Struct Kinematics_Forward;//���������˶�ѧ����ṹ�����
	Battery_Struct PC_Battery={0,0,0,0};//���״̬��Ϣ�ṹ�����
	


#endif


#define DATABUFF_T_HZ	1000	//���߼����������Ƶ��
#define DATABUFF_Send_HZ	200	//���߼����������Ƶ��
#define IMUBuffReceive_HZ	100	//IMU���ݷ���Ƶ��

/* USER CODE END Header_DataTreat_Task */
void DataTreat_Task(void const * argument)
{
  /* USER CODE BEGIN DataTreat_Task */
	taskENTER_CRITICAL();//�����ٽ���	
	//ң����
#if USER_DATA_VARIABLE_DBUG==0
	RosRemote p_RC_data={.ROS_Schema=STATE_LOST,0,0,0,0,0,0};
  struct PCdataUp IMUdata2PC={0,0,0,0,0,0,0,0,0};
	ParseData_Struct ParseData_Mast;//������λ������Э��ṹ�����

#endif
	BaseType_t RC_err;
	
	uint32_t previous_DataBuff_T=0;	//���߼���������в�ֵ
	uint32_t previous_DataBuffSend_T=0;	//���߼���������в�ֵ
	Kinematics_Struct Kinematics_Forward;//���������˶�ѧ����ṹ�����
	Battery_Struct PC_Battery={0,0,0,0};//���״̬��Ϣ�ṹ�����
	PcTargetSpeed  TargetSpeed={0,0,0};
	uint8_t HeartbeatTime=20;
	taskEXIT_CRITICAL();//�˳��ٽ���

  /* Infinite loop */
  for(;;)
  {
	
	

		if((getSystick()-previous_DataBuff_T)>=(1000/DATABUFF_T_HZ)){
			OnLineCount();
			previous_DataBuff_T=getSystick();
		}
		
		/*************************��ʾ������ң������ʾ��*************/

		//���ݷ���
		if((getSystick()-previous_DataBuffSend_T)>=(1000/DATABUFF_Send_HZ))
		{
			if(RC_Line_prompt())//�Ƿ�׼������ң����ʾ��ɺ��������ң�ػ�����λ��
			{
				if(RC_Queue!=NULL){
					DataTreat_Receive(&p_RC_data);
					/************����ң����ʱ����ң����״̬��ʾ��*********/
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
	/******************************ң�������ݷ���*************************/
					RC_err=xQueueSend(RC_Queue,&p_RC_data,1);
					if(RC_err==errQUEUE_FULL){
						
					}
				}
		
			}
			previous_DataBuffSend_T=getSystick();
		}
		
		if((p_RC_data.ROS_Schema==STATE_UC)||(p_RC_data.ROS_Schema ==STATE_RC)){
				//����Է���IMU���ݸ���
				if (IMU_Queue!=NULL)
				{
					if (xQueueReceive(IMU_Queue,&IMUdata2PC,10))
					{
						/*�������ݵ���λ��*/
						update_IMUdata(&IMUdata2PC);
						}
					}
				//����Է����ƶ����ݸ���
					if(MOVE_Queue!=NULL){
					if(xQueueReceive(MOVE_Queue,&Kinematics_Forward,1)){
					
							/*�����ٶȵ���λ��*/
							update_MOVEdata(&Kinematics_Forward);
						}
						else{
						
						}
					}
					//����Է��͵������ݸ���
					if(BATTER_Queue!=NULL){
						if(xQueueReceive(BATTER_Queue,&PC_Battery,1))
						{
							/*����Է��͵�������*/
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
		taskENTER_CRITICAL();//�����ٽ���	
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = 5; //��������

		taskEXIT_CRITICAL();//�˳��ٽ���

  /* Infinite loop */
  for(;;)
  {
	  xLastWakeTime=xTaskGetTickCount();  //��õ�ǰϵͳʱ��

		PC_msgUpdate();

		osDelayUntil(&xLastWakeTime,xFrequency);

  }
  /* USER CODE END Data2PC_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */




		 
		 
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
