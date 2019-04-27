/*
 * finger_print2.h
 *
 *  Created on: Apr 17, 2019
 *      Author: yashm
 */

/************************* Description **********************
 *  RX -> P32	- BLACK
 *  TX -> P30	- RED
 *  VCC-> 5V
 *  GNF-> GND
 ************************************************************/
#ifndef SRC_FINGER_PRINT_H_
#define SRC_FINGER_PRINT_H_
#define YES 0
#define NO  1

#define TIVA_C   NO

#if (TIVA_C==NO)
#include "em_usart.h"
#include "em_cmu.h"
#include "retargetserial.h"
#include "log.h"
#include "em_core.h"
#include "gpio.h"
#include "LETIMER.h"

extern flag;
uint8_t ButtonFlag;

#define BUTTON0 (1)
#define BUTTON1 (2)

#else
#include "main.h"

#include <stdint.h>
#include <stdbool.h>
#include "drivers/pinout.h"
#include "utils/uartstdio.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"

// TivaWare includes
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "i2cm_drv.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#endif

#define COMMAND_SIZE (12)
#define DEV_ID      (0x00000000)
#define ON      (1)
#define OFF     (0)
#define CMD_CMOS_LED        (0x12)
#define CMD_OPEN            (0x00)
#define CMD_ENROLL_CNT      (0X20)
#define CMD_CHECK_ENR       (0x21)
#define CMD_ENROLL_START    (0x22)
#define CMD_CAPTURE_FIN     (0X60)
#define CMD_ENROLL_1        (0X23)
#define CMD_ENROLL_2        (0X24)
#define CMD_ENROLL_3        (0X25)
#define CMD_IS_PRESS        (0X26)
#define CMD_GET_IMAGE       (0X62)
#define CMD_DEL_ALL         (0X41)
#define CMD_VERIFY          (0x50)
#define CMD_ID              (0x51)
#define CMD_ENROLL_COUNT    (0x20)

void FingerPrintInit(void);
void CommandSend(uint8_t param,uint8_t cmd_code);
uint32_t CommandResponse(uint8_t param);
void CheckFingerPrint(void);
uint16_t CheckSumCal(uint8_t* Command);
void AddFingerPrint(void);
// Variables
extern uint8_t CommandArray[COMMAND_SIZE];
extern uint8_t ResponseArray[COMMAND_SIZE];

#endif /* SRC_FINGER_PRINT_H_ */

