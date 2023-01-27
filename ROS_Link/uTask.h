#ifndef _UTASK_H_
#define _UTASK_H_

#include "Remote.h"
enum ROS_STATION{
	
	STATE_UC,
	STATE_RC,
	STATE_WAIT, //������
	STATE_LOST, //����
	STATE_REST

};

enum CheckOnlion{
	
	OnLion,
	
	Lost,
	
};

typedef struct RosRemote{
	
	enum ROS_STATION		ROS_Schema;//����ģʽ����
	
	 float Linear_X;	//X�����ٶ� m/s
	 float Linear_Y;	//Y�����ٶ� m/s
	 float Angular_Z;	//Z����ٶ� rad/s	
	 
	 float last_Linear_X;	//X�����ٶ� m/s
	 float last_Linear_Y;	//Y�����ٶ� m/s
	 float last_Angular_Z;	//Z����ٶ� rad/s	

	 

}RosRemote;

typedef struct
{
	 uint8_t RC_UpdataCountor;// ң�������¼���
	 enum ROS_STATION Staion;
	 float Linear_X;	//X�����ٶ� m/s
	 float Linear_Y;	//Y�����ٶ� m/s
	 float Angular_Z;	//Z����ٶ� rad/s	
}Data_Struct;//�����˶�ѧ�ṹ�����

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
