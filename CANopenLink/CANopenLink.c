#include "canfestival.h"
#include "tim.h"
#include "can.h"

static TIMEVAL last_time_set = TIMEVAL_MAX;

unsigned int TimeCNT=0;             //时间计数
unsigned int NextTime=0;            //下一次触发时间计数
unsigned int TIMER_MAX_COUNT=70000; //最大的时间计数

void TimerForCan(void)
{
    TimeCNT++;
    
    if (TimeCNT >= TIMER_MAX_COUNT)
    {
        TimeCNT=0;
    }
    if (TimeCNT == NextTime)
    {
        TimeDispatch();     //定时时间到，执行时间相关的分发任务
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

