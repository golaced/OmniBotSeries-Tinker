#ifndef _SYS_TIME_H_
#define _SYS_TIME_H_

#define GET_TIME_NUM 50

enum
{
	NOW = 0,
	OLD,
	NEW,
	DT_LAST
};

float Get_Cycle_T(int item);
void Cycle_Time_Init();
#endif
