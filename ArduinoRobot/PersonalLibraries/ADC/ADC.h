#ifndef ADC_H_
#define ADC_H_

////////////////////////////// INCLUDES ///////////////////////////////////////////
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ErrorManager.h"
////////////////////////////// END INCLUDES ///////////////////////////////////////

////////////////////////////// DEFINES ////////////////////////////////////////////
#define ADC0 0																			// Human-friendly name for ADC channels
#define ADC1 1																			//
#define ADC2 2																			//
#define ADC3 3																			//
#define ADC4 4																			//
#define ADC5 5																			//
#define ADCTEMPSENSOR 8																	// In-built temperature sensor on the ADC (convert with internal 1.1v reference)
////////////////////////////// END DEFINES ////////////////////////////////////////

////////////////////////////// VARIABLES DECLARATION //////////////////////////////
extern volatile uint8_t ISR_ADC;														// External variable indicating a finished conversion. TO BE POLLED IN MAIN PROGRAM
////////////////////////////// END VARIABLES DECLARATION //////////////////////////

////////////////////////////// FUNCTIONS DECLARATION //////////////////////////////
void initADC();																			// Initialize ADC
void startADC(uint8_t channel);															// Start ADC
uint16_t getADCValue();																	// Return ADC value
////////////////////////////// END FUNCTIONS DECLARATION //////////////////////////

#endif