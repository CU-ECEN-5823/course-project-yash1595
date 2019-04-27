/*
 * smoke.c
 *
 *  Created on: Apr 21, 2019
 *      Author: yashm
 */

#include "smoke.h"


/*********************************************
 * @brief: Sets flag to false to be used when
 * 		   Smoke alert is to be set.
 *********************************************/
void SmokeInit(void)
{
	WarnFlag=false;
}

/**********************************************
 * @brief: Reads ADC value. Triggers when > 2V.
 **********************************************/
void SmokeRead(void)
{
	static uint32_t i=0;
	WarnFlag=false;
	for(i=0;i<1;i+=1)
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
