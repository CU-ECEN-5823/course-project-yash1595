/*
 * smoke.c
 *
 *  Created on: Apr 21, 2019
 *      Author: yashm
 */

#include "smoke.h"

void SmokeInit(void)
{
	WarnFlag=false;
}

void SmokeRead(void)
{
	static uint32_t i=0;
	WarnFlag=false;
	for(i=0;i<100;i+=1)
	{
		ADCSample();
		WaitForFlag();
		if(millivolts>2000)
		{
			WarnFlag=true;
			return;
		}
		LOG_INFO("Millivolts:%d",millivolts);

	}

}
