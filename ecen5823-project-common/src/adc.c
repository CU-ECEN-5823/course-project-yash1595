/*
 * adc.c
 *
 *  Created on: Apr 19, 2019
 *      Author: yashm
 */


#include "adc.h"

/**************************************************************************//**
 * @brief  Initialize ADC function
 *****************************************************************************/
void initADC (void)
{
  adcFinished=false;
  // Enable ADC0 clock
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Modify init structs and initialize
  init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1

  initSingle.diff       = false;        // single ended
  initSingle.reference  = adcRef2V5;    // internal 2.5V reference
  initSingle.resolution = adcRes12Bit;  // 12-bit resolution
  initSingle.acqTime    = adcAcqTime4;  // set acquisition time to meet minimum requirement

  // Select ADC input.
  initSingle.posSel = adcPosSelAPORT2XCH9;

  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);
//  ADC_IntClear(ADC0,ADC_IF_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);
  ADC_IntEnable(ADC0,ADC_IF_SINGLE);
}

void ADCSample(void)
{

	//if(!ADC_Initialied_flag)return;
  // Start ADC conversion
  ADC_Start(ADC0, adcStartSingle);

//  // Wait for conversion to be complete
//  while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));
//
//  // Get ADC result
//  sample = ADC_DataSingleGet(ADC0);
//
//  // Calculate input voltage in mV
//  millivolts = (sample * 2500) / 4096;
//  LOG_INFO("ADC:Value:%d",millivolts);
}

void ADC0_IRQHandler(void)
{
	LOG_INFO("here\n");
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL();
	/* Clear ADC0 interrupt flag */
	uint32_t flags = ADC_IntGet(ADC0);
	ADC_IntClear(ADC0, flags);
	/* Read conversion result to clear Single Data Valid flag */
	sample = ADC_DataSingleGet(ADC0);
	millivolts = (sample * 2500) / 4096;
	adcFinished=true;
  CORE_EXIT_CRITICAL();
}
