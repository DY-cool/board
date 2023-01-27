#include "canfestival.h"
#include "tim.h"
#include "can.h"

static TIMEVAL last_time_set = TIMEVAL_MAX;

unsigned int TimeCNT=0;             //ʱ�����
unsigned int NextTime=0;            //��һ�δ���ʱ�����
unsigned int TIMER_MAX_COUNT=70000; //����ʱ�����

void TimerForCan(void)
{
    TimeCNT++;
    
    if (TimeCNT >= TIMER_MAX_COUNT)
    {
        TimeCNT=0;
    }
    if (TimeCNT == NextTime)
    {
        TimeDispatch();     //��ʱʱ�䵽��ִ��ʱ����صķַ�����
    }
}

void __HAL_TIM7_PeriodElapsedCallback(void){
	TimerForCan();
}


void initTimer(void)
{
	CANopenTimeInit();

}
UNS8 canSend(CAN_PORT notused, Message *m){

 return CAN_Send((can_message_t*)m);
}
UNS8 canChangeBaudRate(CAN_PORT port, char* baud){

	return 0;
}

void setTimer(TIMEVAL value)
{ 
    NextTime = (TimeCNT+value)%TIMER_MAX_COUNT;
}

TIMEVAL getElapsedTime(void)
{
    int ret=0;
    
    ret = TimeCNT> last_time_set ? TimeCNT - last_time_set : TimeCNT + TIMER_MAX_COUNT - last_time_set;
    last_time_set = TimeCNT;
    return ret;
}

