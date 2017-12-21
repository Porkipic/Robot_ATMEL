#ifndef TIMER_H_
#define TIMER_H_

////////////////////////////// INCLUDES ///////////////////////////////////////////
#include <avr/interrupt.h>
////////////////////////////// END INCLUDES ///////////////////////////////////////

////////////////////////////// DEFINES ////////////////////////////////////////////
#define HIGH 0																			// Used to indicate the status read by pulsein function
#define LOW 1																			// Used to indicate the status read by pulsein function
////////////////////////////// END DEFINES ////////////////////////////////////////

////////////////////////////// VARIABLES DECLARATION //////////////////////////////
extern volatile uint8_t ISR_TMR1CAPT;													// External variable indicating Input Compare condition. TO BE POLLED IN MAIN PROGRAM
////////////////////////////// END VARIABLES DECLARATION //////////////////////////

////////////////////////////// FUNCTIONS DECLARATION //////////////////////////////
void initTimer();																		// Initialize Timer utilities
uint32_t micros();																		// Returns number of microseconds since initTimer();
uint32_t millis();																		// Returns number of milliseconds since initTimer();
void startPulseLength(uint8_t state, uint16_t timeout);									// Start us pulse length timer, detecting pin state and returns 0 if timeout
uint16_t getPulseLength();																// Return number of us since startPulseLength													
////////////////////////////// END FUNCTIONS DECLARATION //////////////////////////


#endif