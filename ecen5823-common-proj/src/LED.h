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
 *  P3  ==> LED
 *  GND ==> Ground
 *
 **************************/

#define LED_PORT	gpioPortC
#define LED_PIN		(7)

void InitLED(void);
void LEDOn(void);
void LEDOff(void);

#endif /* SRC_LED_H_ */
