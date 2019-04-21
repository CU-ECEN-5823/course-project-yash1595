/*
 * servo.h
 *
 *  Created on: Apr 9, 2019
 *      Author: yashm
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#include "LETIMER.h"

#define DOOR_OPEN	(1000)//1000
#define DOOR_CLOSE	(10000)//10000
#define TOTAL_CYCLE	(20000)
#define SERVO_PORT	(3)
#define SERVO_PIN	(12)

void ServoInit(void);
void UnlockDoor(void);
void LockDoor(void);

#endif /* SRC_SERVO_H_ */
