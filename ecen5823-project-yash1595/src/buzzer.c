/*
 * buzzer.c
 *
 *  Created on: Apr 9, 2019
 *      Author: yashm
 */

#include "buzzer.h"
//
//void BuzzerInit(void)
//{
//	GPIO_PinModeSet(gpioPortF,3,gpioModePushPull,true);
//	BuzzerOff();
//}
//
//void Buzzer(uint32_t ring)
//{
////	static uint8_t i=0;
////	for(i=0;i<ring;i+=1)
////	{
////		GPIO_PinOutSet(gpioPortF,3);
////		BlockWait(1500);
////		GPIO_PinOutClear(gpioPortF,3);
////		BlockWait(1500);
////	}
//	GPIO_PinOutSet(gpioPortF,3);
//}
//
//void BuzzerOff(void)
//{
//	GPIO_PinOutClear(gpioPortF,3);
//}

/*
 * buzzer.c
 *
 *  Created on: Apr 9, 2019
 *      Author: yashm
 */

#include "buzzer.h"

#define PWM_FREQ 1000

void BuzzerInit(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER0, true);
  /* set CC0 location 15 pin (PC10) as output, initially high */
  GPIO_PinModeSet (gpioPortC, 10, gpioModePushPull, 1);

  /* select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit =
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionToggle,
    .mode       = timerCCModePWM,
    .filter     = false,
    .prsInput   = false,
    .coist      = true,
    .outInvert  = false,
  };

  /* configure CC channel 0 */
  TIMER_InitCC (TIMER0, 0, &timerCCInit);

  /* route CC0 to location 15 (PC10) and enable pin */
  TIMER0->ROUTELOC0 |= TIMER_ROUTELOC0_CC0LOC_LOC15;
  TIMER0->ROUTEPEN |= TIMER_ROUTEPEN_CC0PEN;

  /* set PWM period */
  topValue = CMU_ClockFreqGet (cmuClock_HFPER) / (1000); // TIMER_ClkSel_TypeDef
  TIMER_TopSet (TIMER0, topValue);

  /* Set PWM duty cycle to 50% */
  TIMER_CompareSet (TIMER0, 0, (100000));	//100000	//1000000

  /* set timer parameters */
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = false,
    .debugRun   = true,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

  /* configure and start timer */
  TIMER_Init (TIMER0, &timerInit);
  BuzzerOff();
  //ServoPosition(lock);//(true,false);//LockDoor();
}


//void BuzzerInit(void)
//{
//	GPIO_PinModeSet(gpioPortF,3,gpioModePushPull,true);
//	BuzzerOff();
//}

void Buzzer(uint32_t ring)
{
//	static uint8_t i=0;
//	for(i=0;i<ring;i+=1)
//	{
//		GPIO_PinOutSet(gpioPortF,3);
//		BlockWait(1500);
//		GPIO_PinOutClear(gpioPortF,3);
//		BlockWait(1500);
//	}
	//GPIO_PinOutSet(gpioPortF,3);
	TIMER_Enable(TIMER0,true);
}

void BuzzerOff(void)
{
	//GPIO_PinOutClear(gpioPortF,3);
	TIMER_Enable(TIMER0,false);
}




