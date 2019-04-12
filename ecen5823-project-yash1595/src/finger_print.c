/*
 * fingerprint.c
 *
 *  Created on: Apr 5, 2019
 *      Author: yashm
 */

#include "finger_print.h"

USART_TypeDef           *finger = USART2;

void FingerPrintInit(void)
{
	CMU_ClockEnable(cmuClock_USART2, true);

	GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, true); //TX
	GPIO_PinModeSet(gpioPortF, 5, gpioModeInput, false); 	//RX

	GPIO_DriveStrengthSet(gpioPortF,gpioDriveStrengthStrongAlternateStrong);

	LOG_INFO("Set the pins\n");

	USART_InitAsync_TypeDef UartFinger   =
			{                                                                                                  \
					usartDisable,           /* Enable RX/TX when initialization is complete. */
					0,                     /* Use current configured reference clock for configuring baud rate. */
					9600,                /* 9600 bits/s. */
					usartOVS16,            /* 16x oversampling. */
					usartDatabits8,        /* 8 data bits. */
					usartNoParity,         /* No parity. */
					usartStopbits1,        /* 1 stop bit. */
					false,                 /* Do not disable majority vote. */
					false,                 /* Not USART PRS input mode. */
					usartPrsRxCh0,         /* PRS channel 0. */
					false,                 /* Auto CS functionality enable/disable switch */
					0,                     /* Auto CS Hold cycles */
					0,                     /* Auto CS Setup cycles */
					usartHwFlowControlNone /* No HW flow control */                                                       \
			  };


	  LOG_INFO("Set the config\n");
	  USART_InitAsync(USART2,&UartFinger);

//	  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

//	  CMU_ClockDivSet(cmuClock_USART2,cmuClkDiv_1);

//	  UartFinger.enable = usartDisable;	//check
	  LOG_INFO("Set the clock source\n");

//	  USART_Enable(USART2, usartDisable);//LEUART_Init(USART2, &UartFinger);	//??

	  USART2->ROUTELOC0 = ((17)|(17<<8));	//try
	  USART2->ROUTEPEN = ((1<<0) |(1<<1));  //try


	  LOG_INFO("Set the ROUTE pins\n");

	  USART_Enable(USART2, usartEnable);	//check

	  LOG_INFO("USART2 Enbaled\n");

	CommandSend(0,CMD_OPEN);
	CommandResponse(CMD_OPEN); //CMD_DEL_ALL //CMD_CMOS_LED


	CommandSend(0,CMD_CMOS_LED);
	CommandResponse(CMD_CMOS_LED);
}

void CommandSend(uint8_t param,uint8_t cmd_code)
{
	uint8_t CommandArray[COMMAND_SIZE]={
							  0x55,					//CMD START CODE-1
							  0xAA,					//CMD START CODE-2
							  0X01,0,			//Device-ID
							  param,0,0,0,	//Input Para
							  cmd_code,0,		//CMD CODE
							  0,0};			//CHECK-SUM

	uint16_t checksum=CheckSumCal(&CommandArray[0]);
	CommandArray[10]=checksum & 0X00FF;
	CommandArray[11]=(checksum & 0XFF00) >> 8;

//	LOG_INFO("Checksum:%4x",checksum);
//
//	LOG_INFO("Checksum-HIGH:%4x",(CommandArray[COMMAND_SIZE-1]));
//
//	LOG_INFO("Checksum-LOW:%4x",CommandArray[COMMAND_SIZE-2]);

	uint8_t i=0;

	CORE_AtomicDisableIrq();
	for(i=0;i<12;i+=1)
	{
		USART_Tx(finger,CommandArray[i]);
		//LOG_INFO("Index Sent:[%x]\n",CommandArray[i]);
	}
	CORE_AtomicEnableIrq();

//	LOG_INFO("Sent All Command bytes\n");
}

uint32_t CommandResponse(uint8_t param)
{
	LOG_INFO("Entered Command Response\n");
	uint8_t ResponseArray[COMMAND_SIZE],i,ReceivedByte;

	CORE_AtomicDisableIrq();
	for(i=0;i<COMMAND_SIZE;i+=1) //COMMAND_SIZE-5
	{
		ResponseArray[i]=USART_Rx(USART2);//LOG_INFO("%x",USART_Rx(USART2));

	}
	CORE_AtomicEnableIrq();

	for(i=0;i<COMMAND_SIZE;i+=1)
		LOG_INFO("received:%x",ResponseArray[i]);



	uint32_t Value;
	if(param==CMD_IS_PRESS)
	{
		Value = ResponseArray[4];
		LOG_INFO("Value received:%d",Value);
		return Value;
	}

	Value |= ResponseArray[4]<<0;
	Value |= ResponseArray[5]<<8;
	Value |= ResponseArray[6]<<16;
	Value |= ResponseArray[7]<<24;

	LOG_INFO("Status:%x",Value);
	LOG_INFO("Received all Response Bytes\n");

	return Value;

}

uint16_t CheckSumCal(uint8_t* Command)
{
	uint16_t checksum=0,i=0;
	for(i=0;i<(COMMAND_SIZE);++i)
		{
			checksum+=*Command;
			Command+=1;
		}
	return checksum;
}

void CheckFingerPrint(void)
{

	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();

	uint32_t i=0,Value=1,error=0;

	LOG_INFO("LED======\n");
	CommandSend(1,CMD_CMOS_LED);
	CommandResponse(CMD_CMOS_LED);
	while(1)
	{
		CommandSend(0,CMD_IS_PRESS);
		Value=CommandResponse(CMD_IS_PRESS);
		if(Value){LOG_INFO("Place Finger\n");error+=1;LOG_INFO("ERRROR:%d",error);}
		if(error>50)
			{
				CommandSend(0,CMD_CMOS_LED);
				CommandResponse(CMD_CMOS_LED);
				return;
			}
		else if(!Value)break;

	}

	LOG_INFO("CAPTURE FIN-1======\n");
	CommandSend(0x00,CMD_CAPTURE_FIN);
	uint32_t resp2=CommandResponse(CMD_CAPTURE_FIN);
	if(resp2!=0x1012)LOG_INFO("Success capturing\n");

	LOG_INFO("CHECK======\n");
	CommandSend(0x00,CMD_ID);
	if(CommandResponse(CMD_ID))
	{
		LOG_INFO("Invalid ID\n");
		LOG_INFO("LED======\n");
		CommandSend(0,CMD_CMOS_LED);
		CommandResponse(CMD_CMOS_LED);
		return;
	}

	LOG_INFO("Valid ID\n");

	if(ButtonFlag==BUTTON0)LOG_INFO("Smoke Override\n");
	else LOG_INFO("Allow Entry\n");

	CommandSend(0,CMD_CMOS_LED);
	CommandResponse(CMD_CMOS_LED);

	ButtonFlag=0;

	CORE_EXIT_CRITICAL();

	return;

}
//void CommandcmosLED(uint8_t state)
//{
//	uint8_t CommandArray[COMMAND_SIZE]={
//							  0x55,					//CMD START CODE-1
//							  0xAA,					//CMD START CODE-2
//							  0X01,0X00,			//Device-ID
//							  state,0X00,0X00,0X00,	//Input Para
//							  0x12,0x00,			//CMD CODE
//							  0X00,0X00};			//CHECK-SUM
//
//	uint16_t checksum=CheckSumCal(&CommandArray);
//	CommandArray[COMMAND_SIZE-2]=checksum&0x00FF;
//	CommandArray[COMMAND_SIZE-1]=((checksum >> 8) &0X00FF);
//
//	LOG_INFO("Checksum:%4x",checksum);
//
//	LOG_INFO("Checksum-HIGH:%4x",(CommandArray[COMMAND_SIZE-1]));
//
//	LOG_INFO("Checksum-LOW:%4x",CommandArray[COMMAND_SIZE-2]);
//
//	uint8_t i=0;
//	for(i=0;i<COMMAND_SIZE;++i)
//	{
//		USART_Tx(finger,CommandArray[i]);
//		LOG_INFO("Index Sent:[%x]\n",CommandArray[i]);
//	}
//	LOG_INFO("Sent All Command bytes\n");
//}
