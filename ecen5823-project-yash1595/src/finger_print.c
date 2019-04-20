/*
 * finger_print.c
 *
 *  Created on: Apr 17, 2019
 *      Author: yashm
 */

#include "finger_print.h"

#if (TIVA_C==NO)
USART_TypeDef           *finger = USART2;

void FingerPrintInit(void)
{
    CMU_ClockEnable(cmuClock_USART2, true);

    GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, true);  //TX
    GPIO_PinModeSet(gpioPortF, 5, gpioModeInput, false);    //RX

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

//    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

//    CMU_ClockDivSet(cmuClock_USART2,cmuClkDiv_1);

//    UartFinger.enable = usartDisable; //check
      LOG_INFO("Set the clock source\n");

//    USART_Enable(USART2, usartDisable);//LEUART_Init(USART2, &UartFinger);    //??

      USART2->ROUTELOC0 = ((17)|(17<<8));   //try
      USART2->ROUTEPEN = ((1<<0) |(1<<1));  //try


      LOG_INFO("Set the ROUTE pins\n");

      USART_Enable(USART2, usartEnable);    //check

      LOG_INFO("USART2 Enbaled\n");

    CommandSend(0,CMD_OPEN);
    CommandResponse(CMD_OPEN); //CMD_DEL_ALL //CMD_CMOS_LED


    CommandSend(0,CMD_CMOS_LED);
    CommandResponse(CMD_CMOS_LED);
}
#endif

#if (TIVA_C ==YES)
void FingerPrintInit(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    MAP_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    MAP_UARTConfigSetExpClk(UART7_BASE, SYSTEM_CLOCK, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}
#endif

void CommandSend(uint8_t param,uint8_t cmd_code)
{
#if (TIVA_C==YES)
    UARTprintf("Entered command send\n");
#else
    LOG_INFO("Entered command send\n");
#endif

    uint8_t CommandArray[COMMAND_SIZE]={
                              0x55,                 //CMD START CODE-1
                              0xAA,                 //CMD START CODE-2
                              0X01,0,               //Device-ID
                              param,0,0,0,          //Input Para
                              cmd_code,0,           //CMD CODE
                              0,0};                 //CHECK-SUM

    uint16_t checksum=CheckSumCal(&CommandArray[0]);
    CommandArray[10]=checksum & 0X00FF;
    CommandArray[11]=(checksum & 0XFF00) >> 8;

    uint8_t i=0;


    for(i=0;i<12;i+=1)
    {
#if (TIVA_C == YES)
        UARTCharPut(UART7_BASE,CommandArray[i]);
#else
        USART_Tx(finger,CommandArray[i]);
#endif
        //UARTprintf("%x\n",CommandArray[i]);
    }

#if (TIVA_C == YES)
    UARTprintf("Sent all bytes\n",CommandArray[i]);
#else
    LOG_INFO("Sent All Command bytes\n");
#endif



}

uint32_t CommandResponse(uint8_t param)
{
#if (TIVA_C ==YES)
    UARTprintf("Entered Command Response\n");
#else
    LOG_INFO("Entered Command Response\n");
#endif

    uint8_t ResponseArray[COMMAND_SIZE],i,ReceivedByte;


    for(i=0;i<12;i+=1) //COMMAND_SIZE-5
    {
#if (TIVA_C ==YES)
        ResponseArray[i]=UARTCharGet(UART7_BASE);//UARTprintf("%x",USART_Rx(USART2));
#else
        ResponseArray[i]=USART_Rx(USART2);
#endif
    }

//
//    for(i=0;i<12;i+=1) //COMMAND_SIZE-5
//        {
//            UARTprintf("%x\n",ResponseArray[i]);
//        }


    uint32_t Value;
    if(param==CMD_IS_PRESS)
    {
        Value = ResponseArray[4];
#if (TIVA_C ==YES)
        UARTprintf("Value received:%d",Value);
#else
        LOG_INFO("Value received:%d",Value);
#endif
        return Value;
    }

    if(param==CMD_ID)
    {
        Value = ResponseArray[8];
        return Value;
    }

    if(param==CMD_ENROLL_COUNT)
    {
        Value = ResponseArray[10]-30;
        return Value;
    }



    Value |= ResponseArray[4]<<0;
    Value |= ResponseArray[5]<<8;
    Value |= ResponseArray[6]<<16;
    Value |= ResponseArray[7]<<24;

#if (TIVA_C ==YES)
    UARTprintf("Status:%x",Value);
    UARTprintf("Received all Response Bytes\n");
#else
    LOG_INFO("Status:%x",Value);
    LOG_INFO("Received all Response Bytes\n");
#endif
    return Value;

}

uint16_t CheckSumCal(uint8_t* Command)
{
    uint16_t checksum=0,i=0;
    for(i=0;i<12;++i)
        {
            checksum+=*Command;
            Command+=1;
        }

    return checksum;
}


void AddFingerPrint(void)
{
    static uint32_t value=0,wait=0;


    CommandSend(1,CMD_CMOS_LED);
    CommandResponse(CMD_CMOS_LED);

    CommandSend(0,CMD_ENROLL_COUNT);
    uint32_t id_count=CommandResponse(CMD_ENROLL_COUNT)+1;

    CommandSend(id_count,CMD_ENROLL_START);    //Increment the first parameter every time.
    CommandResponse(CMD_ENROLL_START);

    value=0;
    CommandSend(0,CMD_IS_PRESS);
    value=CommandResponse(CMD_IS_PRESS);
    while(!value)
    {
#if(TIVA_C ==YES)
        UARTprintf("Remove Finger\n");
#else
        LOG_INFO("Remove Finger\n");
#endif
        CommandSend(0,CMD_IS_PRESS);
        value=CommandResponse(CMD_IS_PRESS);
    }

    value=1;
    CommandSend(0,CMD_IS_PRESS);
    value=CommandResponse(CMD_IS_PRESS);
    while(value)
    {
#if(TIVA_C ==YES)
        UARTprintf("Place Finger\n");
#else
        LOG_INFO("Place Finger\n");
#endif
        CommandSend(0,CMD_IS_PRESS);
        value=CommandResponse(CMD_IS_PRESS);
    }

//    UARTprintf("Capture Finger\n");
    CommandSend(0,CMD_CAPTURE_FIN);
    uint32_t resp2=CommandResponse(CMD_CAPTURE_FIN);
//    if(resp2!=0x1012)UARTprintf("Success capturing\n");

    //SysCtlDelay(2000000);

//    UARTprintf("Enroll-1\n");
    CommandSend(0,CMD_ENROLL_1);
    CommandResponse(CMD_ENROLL_1); // //SysCtlDelay(2000000);

    //SysCtlDelay(2000000);

 //   UARTprintf("IsPress?\n");
    value=0;
    CommandSend(0,CMD_IS_PRESS);
    value=CommandResponse(CMD_IS_PRESS);
    while(!value)
    {
#if(TIVA_C ==YES)
        UARTprintf("Remove Finger\n");
#else
        LOG_INFO("Remove Finger\n");
#endif
        CommandSend(0,CMD_IS_PRESS);
        value=CommandResponse(CMD_IS_PRESS);
    }

    //SysCtlDelay(2000000);

    value=1;
    CommandSend(0,CMD_IS_PRESS);
    value=CommandResponse(CMD_IS_PRESS);
    while(value)
    {
#if(TIVA_C ==YES)
        UARTprintf("Place Finger\n");
#else
        LOG_INFO("Place Finger\n");
#endif
        CommandSend(0,CMD_IS_PRESS);
        value=CommandResponse(CMD_IS_PRESS);
    }

    //SysCtlDelay(2000000);

//    UARTprintf("Capture Finger\n");
    CommandSend(0x00,CMD_CAPTURE_FIN);
    resp2=CommandResponse(CMD_CAPTURE_FIN);
 //   if(resp2!=0x1012)UARTprintf("Success capturing\n");

    //SysCtlDelay(2000000);

 //   UARTprintf("Enroll-2\n");
    CommandSend(0,CMD_ENROLL_2);
    CommandResponse(CMD_ENROLL_2);

    //SysCtlDelay(2000000);

 //   UARTprintf("IsPress-2?\n");
    value=0;
    CommandSend(0,CMD_IS_PRESS);
    value=CommandResponse(CMD_IS_PRESS);
    while(!value)
    {
#if(TIVA_C ==YES)
        UARTprintf("Remove Finger\n");
#else
        LOG_INFO("Remove Finger\n");
#endif
        CommandSend(0,CMD_IS_PRESS);
        value=CommandResponse(CMD_IS_PRESS);
    }

    //SysCtlDelay(2000000);

    value=1;
    CommandSend(0,CMD_IS_PRESS);
    value=CommandResponse(CMD_IS_PRESS);
    while(value)
    {
#if(TIVA_C ==YES)
        UARTprintf("Place Finger\n");
#else
        LOG_INFO("Place Finger\n");
#endif
        CommandSend(0,CMD_IS_PRESS);
        value=CommandResponse(CMD_IS_PRESS);
    }

    //SysCtlDelay(2000000);

 //   UARTprintf("Capture Finger\n");
    CommandSend(0,CMD_CAPTURE_FIN);
    resp2=CommandResponse(CMD_CAPTURE_FIN);
  //  if(resp2!=0x1012)UARTprintf("Success capturing\n");

    //SysCtlDelay(2000000);

 //   UARTprintf("Enroll-3\n");
    CommandSend(0,CMD_ENROLL_3);
    CommandResponse(CMD_ENROLL_3);

    CommandSend(0,CMD_CMOS_LED);
    CommandResponse(CMD_CMOS_LED);

}


void CheckFingerPrint(void)
{
   uint32_t i=0,Value=1,error=0;

 //   UARTprintf("LED======\n");
    CommandSend(1,CMD_CMOS_LED);
    CommandResponse(CMD_CMOS_LED);
    while(1)
    {
        CommandSend(0,CMD_IS_PRESS);
        Value=CommandResponse(CMD_IS_PRESS);
        if(Value){
#if (TIVA_C ==YES)
            UARTprintf("Place Finger\n");
            UARTprintf("ERRROR:%d",error);
#else
            LOG_INFO("Place Finger\n");
            LOG_INFO("ERRROR:%d",error);
#endif
            error+=1;
        }
        if(error>50)
            {
                CommandSend(0,CMD_CMOS_LED);
                CommandResponse(CMD_CMOS_LED);
#if (TIVA_C == NO)
                ButtonFlag=0;
#endif
                return;
            }
        else if(!Value)break;

    }

    CommandSend(0x00,CMD_CAPTURE_FIN);
    uint32_t resp2=CommandResponse(CMD_CAPTURE_FIN);

    CommandSend(0x00,CMD_ID);
    if(CommandResponse(CMD_ID)!=0x30)
    {
#if (TIVA_C ==YES)
        UARTprintf("Invalid ID\n");
        UARTprintf("LED======\n");
#else
        LOG_INFO("Invalid ID\n");
        LOG_INFO("LED======\n");
#endif
    }
    else
      {
   #if (TIVA_C == YES)
       UARTprintf("Valid ID\n");
   #else
       LOG_INFO("Valid ID\n");
   #endif
      }
        CommandSend(0,CMD_CMOS_LED);
        CommandResponse(CMD_CMOS_LED);

#if(TIVA_C == NO)
        if(ButtonFlag==BUTTON0)Button1();//LOG_INFO("Smoke Override\n");
        else if(ButtonFlag==BUTTON1)Button2();//LOG_INFO("Allow Entry\n");
#endif
        return;
    }

