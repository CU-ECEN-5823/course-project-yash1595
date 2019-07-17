/*
 * LED.h
 *
 *  Created on: Apr 21, 2019
 *      Author: yashm
 */

#ifndef SRC_LED_H_
#define SRC_LED_H_
#include "LETIMER.h"

/********* PIN ************
 *
 *  P8  ==> LED
 *  GND ==> Ground
 *
 **************************/

#define LED_PORT	gpioPortD
#define LED_PIN		(12)

void InitLED(void);
void LEDOn(void);
void LEDOff(void);

#endif /* SRC_LED_H_ */
