#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h> 
#include "sys_time.h"

volatile float Cycle_T[GET_TIME_NUM][4];
struct timeval tv;

uint32_t GetSysTime_us(void)
{
	register uint32_t ms;
	uint32_t value;
	gettimeofday(&tv, NULL);
	ms = tv.tv_sec * 1000000 + tv.tv_usec; 
	value = ms;
	return value;
}

float Get_Cycle_T(int item)
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];
	Cycle_T[item][NOW] = (float)GetSysTime_us() / 1000000.0f;
	if (Cycle_T[item][NOW] > Cycle_T[item][OLD])
	{
		Cycle_T[item][NEW] = ((Cycle_T[item][NOW] - Cycle_T[item][OLD]));
		Cycle_T[item][DT_LAST] = Cycle_T[item][NEW];
	}
	else
		Cycle_T[item][NEW] = Cycle_T[item][DT_LAST];

	return Cycle_T[item][NEW];
}

void Cycle_Time_Init()
{
	int i;
	for (i = 0; i < GET_TIME_NUM; i++)
	{
		Get_Cycle_T(i);
	}
}