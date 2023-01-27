#ifndef _UTASK_H_
#define _UTASK_H_

#include "Remote.h"
enum ROS_STATION{
	
	STATE_UC,
	STATE_RC,
	STATE_WAIT, //解算中
	STATE_LOST, //掉线
	STATE_REST

};

enum CheckOnlion{
	
	OnLion,
	
	Lost,
	
};

typedef struct RosRemote{
	
	enum ROS_STATION		ROS_Schema;//地盘模式控制
	
	 float Linear_X;	//X轴线速度 m/s
	 float Linear_Y;	//Y轴线速度 m/s
	 float Angular_Z;	//Z轴角速度 rad/s	
	 
	 float last_Linear_X;	//X轴线速度 m/s
	 float last_Linear_Y;	//Y轴线速度 m/s
	 float last_Angular_Z;	//Z轴角速度 rad/s	

	 

}RosRemote;

typedef struct
{
	 uint8_t RC_UpdataCountor;// 遥控器更新计数
	 enum ROS_STATION Staion;
	 float Linear_X;	//X轴线速度 m/s
	 float Linear_Y;	//Y轴线速度 m/s
	 float Angular_Z;	//Z轴角速度 rad/s	
}Data_Struct;//车体运动学结构体参数

typedef struct RC_Mid_
{
	float	X;
	float	Z;
}RC_Mid;



uint8_t RCdata_Updata(Data_Struct *In_Struct);

void OnLineCount(void);
void DataTreat_Receive(RosRemote* uBuff);
void RC_Data_decod(RosRemote*RC_data,RC_Mid*Mid);
void	RC_prompt(uint8_t ID);
uint8_t RC_Line_prompt(void);
#endif
