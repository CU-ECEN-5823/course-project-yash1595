/*
 * ultrasonic.c
 *
 *  Created on: Apr 6, 2019
 *      Author: yashm
 */
#include "ultrasonic.h"

void InitSensor(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(gpioPortD, 11, gpioModePushPull, true);		//Output
	GPIO_PinModeSet(gpioPortD, 10, gpioModeInput, false);		//Input
	GPIO_IntClear(GPIO_IntGet());
	GPIO_IntConfig(gpioPortD, 10, true, true, true);
//	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
//	GPIO_DriveStrengthSet(gpioPortF, gpioDriveStrengthStrongAlternateStrong);
	LOG_INFO("Set up\n");
}

void TriggerPulse()
{

	GPIO_PinOutClear(gpioPortD,11);
	BlockWait(2);

//	PulseStart=Logging()*1000000;//000000;
//	LOG_INFO("Pulse start");
	GPIO_PinOutSet(gpioPortD,11);
	BlockWait(10);
	GPIO_PinOutClear(gpioPortD,11);

}

float EchoPulse(void)
{
//	PulseStart=Logging();

		//PulseStart=Logging()*1000000;
	BlockWait(200);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	//while(!TriggerComplete){LOG_INFO("inside\n");}
	//TriggerComplete=0;

	LOG_INFO("Echo Done\n");

	if(((PulseStop-PulseStart)/58.0) <= 0.0)
		calculated+= 0;
	else
		calculated+=1;

	LOG_INFO("Pulse Start-Pulse Stop %f",(PulseStop-PulseStart)/58.0);
//	LOG_INFO("Into /58 :%f",(PulseStop-PulseStart)/58.0);
//	LOG_INFO("Into 29.1/2 :%f",(((PulseStart-PulseStop)*29.1)/2));
//	LOG_INFO("Into 29.1/2*0.000001 :%f",((((PulseStart-PulseStop)*29.1)/2)*0.000001));

	return(((PulseStart-PulseStop)/58.0));

}

//void GPIO_EVEN_IRQHandler(void)
//{
//	static int Interrupt_Read;
//	CORE_AtomicDisableIrq();
//	Interrupt_Read = GPIO_IntGet();
//	GPIO_IntClear(Interrupt_Read);
//	LowToHigh^=1;
//	if(LowToHigh)
//	{
//		PulseStart=Logging()*1000000;
//		LOG_INFO("Pulse Start %f",PulseStart);
//	}
//
//	else
//	{
//		PulseStop=Logging()*1000000;
//		LOG_INFO("Pulse Stop %f",PulseStop);
//		TriggerComplete=1;
//	}
//
//	CORE_AtomicEnableIrq();
//}
