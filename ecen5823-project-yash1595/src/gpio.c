/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "log.h"
#include "em_gpio.h"
#include <string.h>
#include "ultrasonic.h"
#include "finger_print.h"

/**
 * TODO: define these.  See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
 * and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
 */


void gpioInit()
{

	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	//GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);

	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	gpioLed1SetOff();

}

void Buttons_Init(void)
{
	GPIO_PinModeSet(gpioPortF, 6, gpioModeInput, false);
	GPIO_PinModeSet(gpioPortF, 7, gpioModeInput, false);
	GPIO_IntClear(GPIO_IntGet());
	GPIO_IntConfig(gpioPortF, 7, true, false, true);
	GPIO_IntConfig(gpioPortF, 6, true, false, true);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	ButtonFlag=0;
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}
void gpioEnableDisplay()
{
	GPIO_PinOutSet(gpioPortD, 15);//Check
}
void gpioSetDisplayExtcomin(bool high)
{
	if(high)
		GPIO_PinOutSet(gpioPortD,13);
	else
		GPIO_PinOutClear(gpioPortD,13);
}

void GPIO_ODD_IRQHandler(void)
{
	uint32_t iflags;
	 LOG_INFO("PB1\n");
	/* Get all odd interrupts. */
	iflags = GPIO_IntGetEnabled() & 0x0000AAAA;

	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	ButtonFlag=BUTTON1;
	CheckFingerPrint();
	CORE_EXIT_CRITICAL();
	GPIO_IntClear(iflags);
	LOG_INFO("Exit\n");

}

void GPIO_EVEN_IRQHandler(void)	//For Finger
{
  LOG_INFO("PB0\n");
	uint32_t iflags;

  /* Get all even interrupts. */
	iflags = GPIO_IntGetEnabled() & 0x00005555;

	/* Clean only even interrupts. */
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	ButtonFlag=BUTTON0;
	CheckFingerPrint();
	CORE_EXIT_CRITICAL();
	GPIO_IntClear(iflags);

}
//void GPIO_EVEN_IRQHandler(void)
//{
//
//	uint32_t iflags;
//
//  /* Get all even interrupts. */
//	iflags = GPIO_IntGetEnabled() & 0x00005555;
//
//	/* Clean only even interrupts. */
//	CORE_DECLARE_IRQ_STATE;
//	CORE_ENTER_CRITICAL();
//	if(LowToHigh^=1)
//		PulseStart=Logging()*1000000;
//	else
//		PulseStop=Logging()*1000000;
//
//	//CheckFingerPrint();
//	CORE_EXIT_CRITICAL();
//	GPIO_IntClear(iflags);
//
//}


