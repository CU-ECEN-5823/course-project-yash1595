/*
 * buzzer.h
 *
 *  Created on: Apr 9, 2019
 *      Author: yashm
 */

#ifndef SRC_BUZZER_H_
#define SRC_BUZZER_H_
#include "LETIMER.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"
/******* PIN **********
 *
 * P10
 */
uint32_t topValue;
void BuzzerInit(void);
void Buzzer(uint32_t);
void BuzzerOff(void);

#endif /* SRC_BUZZER_H_ */
