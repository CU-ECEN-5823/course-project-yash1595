/*
 * buzzer.c
 *
 *  Created on: Apr 9, 2019
 *      Author: yashm
 */

#include "buzzer.h"

void BuzzerInit(void)
{
	GPIO_PinModeSet(gpioPortF,3,gpioModePushPull,true);
}

void Buzzer(uint8_t reps)
{
	static uint8_t i=0;
	for(i=0;i<reps;++i)
	{
		  GPIO_PinOutSet(gpioPortF,3);
		  BlockWait(1500);
		  GPIO_PinOutClear(gpioPortF,3);
		  BlockWait(1500);
	}

}


