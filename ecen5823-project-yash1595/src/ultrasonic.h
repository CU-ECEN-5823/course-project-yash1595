/*
 * ultrasonic.h
 *
 *  Created on: Apr 6, 2019
 *      Author: yashm
 */

#ifndef SRC_ULTRASONIC_H_
#define SRC_ULTRASONIC_H_

#include "LETIMER.h"
#include "log.h"
float PulseStart,PulseStop;
uint32_t calculated;
uint8_t TriggerWaitFlag,TriggerComplete;
uint8_t LowToHigh;

void InitSensor(void);

void TriggerPulse(void);

float EchoPulse(void);

#endif /* SRC_ULTRASONIC_H_ */
