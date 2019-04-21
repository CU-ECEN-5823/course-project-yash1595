/*
 * servo.c
 *
 *  Created on: Apr 9, 2019
 *      Author: yashm
 */

#include "servo.h"

void ServoInit(void)
{

	GPIO_PinModeSet(gpioPortD, 12, gpioModePushPull, true);

}

void UnlockDoor(void)
{
	static uint8_t i=0;
	for(i=0;i<10;++i)
	{
		GPIO_PinOutSet(SERVO_PORT,SERVO_PIN);
		BlockWait(DOOR_OPEN);
		GPIO_PinOutClear(SERVO_PORT,SERVO_PIN);
		BlockWait(TOTAL_CYCLE-DOOR_OPEN);
	}

	//GPIO_PinOutClear(SERVO_PORT,SERVO_PIN);
}

void LockDoor(void)
{
	static uint8_t i=0;
	for(i=0;i<10;++i)
	{
		GPIO_PinOutSet(SERVO_PORT,SERVO_PIN);
		BlockWait(DOOR_CLOSE);
		GPIO_PinOutClear(SERVO_PORT,SERVO_PIN);
		BlockWait(TOTAL_CYCLE-DOOR_CLOSE);
	}
	//GPIO_PinOutClear(SERVO_PORT,SERVO_PIN);
}
