/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "LETIMER.h"
#include "finger_print.h"
#include "LED.h"

#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5

void gpioInit()
{
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

  	GPIO_PinModeSet(PortB0, PinB0, gpioModeInput, false);
  	GPIO_PinModeSet(PortB1, PinB1, gpioModeInput, false);
  	GPIO_PinModeSet(gpioPortD, 12, gpioModePushPull, false);	//Green LED
  	GPIO_DriveStrengthSet(gpioPortB, gpioDriveStrengthWeakAlternateStrong);

}

void Button_Init(void)
{
	uint32_t int_clear;
	int_clear = GPIO_IntGet();
	GPIO_IntClear(int_clear);
	GPIO_IntConfig(PortB0, PinB0, false, true, true);

	int_clear = GPIO_IntGet();
	GPIO_IntClear(int_clear);
	GPIO_IntConfig(PortB1,PinB1, false, true, true);
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
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	uint32_t iflags;

	/* Get all odd interrupts. */
	iflags = GPIO_IntGetEnabled() & 0x0000AAAA;

	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	LOG_INFO(">>>>>>>>>>>>>>>B1\n");
		mask |= button_event;
		ButtonToggle=GPIO_PinInGet(gpioPortF,7);
		LEDOff();
		ButtonFlag=BUTTON1;
	CORE_EXIT_CRITICAL();
	GPIO_IntClear(iflags);

	gecko_external_signal(mask);
	//timer 100ms start
}

void GPIO_EVEN_IRQHandler(void)
{
  uint32_t iflags;
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);

  /* Get all even interrupts. */
	iflags = GPIO_IntGetEnabled() & 0x00005555;

	/* Clean only even interrupts. */
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	LOG_INFO(">>>>>>>>>>>>>>>B2\n");
		mask |= button_event;
		ButtonToggle=GPIO_PinInGet(gpioPortF,6);
		LEDOff();
		ButtonFlag=BUTTON0;
	CORE_EXIT_CRITICAL();
	GPIO_IntClear(iflags);
	gecko_external_signal(mask);

}


/****************************************************************************************
* @brief:	ISR for enabling button interrupts.
*****************************************************************************************/
void EnableGPIOInterrupts(void)
{
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
}
