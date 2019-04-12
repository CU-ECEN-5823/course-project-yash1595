/*
 * fingerprint.h
 *
 *  Created on: Apr 5, 2019
 *      Author: yashm
 */

#ifndef SRC_FINGER_PRINT_H_
#define SRC_FINGER_PRINT_H_

#include "em_usart.h"
#include "em_cmu.h"
#include "retargetserial.h"
#include "log.h"
#include "em_core.h"
#include "gpio.h"
#include "LETIMER.h"
//#include "retargetserial.h"
//#include "retargetserialhalconfig.h"
#define COMMAND_SIZE (12)
#define DEV_ID		(0x00000000)
#define ON		(1)
#define OFF 	(0)
#define CMD_CMOS_LED		(0x12)
#define CMD_OPEN			(0x00)
#define CMD_ENROLL_CNT		(0X20)
#define CMD_CHECK_ENR		(0x21)
#define CMD_ENROLL_START 	(0x22)
#define CMD_CAPTURE_FIN		(0X60)
#define CMD_ENROLL_1		(0X23)
#define CMD_ENROLL_2		(0X24)
#define CMD_ENROLL_3		(0X25)
#define CMD_IS_PRESS		(0X26)
#define CMD_GET_IMAGE		(0X62)
#define CMD_DEL_ALL			(0X41)
#define CMD_VERIFY			(0x50)
#define CMD_ID				(0x51)
void FingerPrintInit(void);
void CommandSend(uint8_t param,uint8_t cmd_code);
uint32_t CommandResponse(uint8_t param);
void CheckFingerPrint(void);
uint16_t CheckSumCal(uint8_t* Command);
// Variables
extern uint8_t CommandArray[COMMAND_SIZE];
extern uint8_t ResponseArray[COMMAND_SIZE];
extern flag;


#endif /* SRC_FINGER_PRINT_H_ */
