/*
 * LED.c
 *
 *  Created on: Apr 21, 2019
 *      Author: yashm
 */
#include "LED.h"


void InitLED(void)
{
	GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, false);	//Green LED
	LEDOff();
}

void LEDOn(void)
{
	GPIO_PinOutSet(LED_PORT, LED_PIN);
}

void LEDOff(void)
{
	GPIO_PinOutClear(LED_PORT, LED_PIN);
}

