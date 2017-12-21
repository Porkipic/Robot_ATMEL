#include "Timer.h"
#include <avr/interrupt.h>

volatile uint16_t captTimestamp = 0;													// Input capture timestamp
volatile uint16_t us1000Counter = 0;													// 1000's us counter
volatile uint32_t microsSecondes;														// Number of microsecondes since start (overflow after around 1h 10min)
volatile uint32_t milliSecondes;														// Number of milliSecondes since start (overflow after around 49 days)

//********** Initialize Timer utilities **********
void initTimer(){
	TCCR0A = (1<<WGM01);																// Timer0 CTC mode
	TCCR0B = (1<<CS00);																	// Timer0 no prescaling
	OCR0A = 7;																			// Timer0 OCR0A TOP value (see formula in datasheet for Normal Mode)
	TIMSK0 = (1<<TOIE0);																// Enable Timer 0 Timer Overflow Interrupt
	TCNT0 = 0;																			// Timer0 counter reset
	
	TCCR1B = (1<<WGM12) | (1<<CS10);													// Timer1 CTC mode OCRA1A TOP value, no prescaling
	TCNT1 = 0;																			// Timer1 counter reset
}
//********************

//********** Enlapsed microseconds **********
uint32_t micros(){
	return microsSecondes;
}
//********************

//********** Enlapsed milliseconds **********
uint32_t millis(){
	return milliSecondes;
}
//********************

//********** Pulse duration timer **********
void startPulseLength(uint8_t state, uint16_t timeout){
	TCNT1 = 0;																			// Timer 1 counter reset
	OCR1A = timeout*7;																	// Set max measurable pulse length duration. TO BE CHECK WITH DATASHEET FORMULA
	TCCR1B = (state<<ICES1);															// Set which edge is detected
	TIMSK1 = (1<<ICIE1) | (1<<OCIE1A);													// Enable Input Capture Interrupt, enable Output Compare A interrupt
}
//********************

//********** Handle pulse duration time(us) **********
uint16_t getPulseLength(){
	ISR_TMR1CAPT = 0;
	return captTimestamp/2;																// Return the timestamp (/2 due to prescaler set to 8)
}
//********************

//********** Timer0 overflow ISR **********
ISR(TIMER0_OVF_vect){
	microsSecondes ++;
	us1000Counter ++;
	if (us1000Counter==1000){
		us1000Counter = 0;																// Reset 1000's us counter
		milliSecondes ++;
	}
}
//********************

//********** Timer1 Input Capture ISR **********
ISR(TIMER1_CAPT_vect){
	captTimestamp = ICR1;																// Return TIMER1_CAPT timestamp
	ISR_TMR1CAPT = 1;																	// Set ISR_TMR1CAPT extern variable. TO BE POLLED IN MAIN PROGRAM
	TIMSK1 = (0<<ICIE1);																// Disable Input Capture Interrupt
}
//********************

//********** Timer1 overflow ISR **********
ISR(TIMER1_OVF_vect){
	captTimestamp = 0;																	// Return 0 if PulseLength>timeout
	ISR_TMR1CAPT = 1;																	// Set ISR_TMR1CAPT extern variable. TO BE POLLED IN MAIN PROGRAM
	TIMSK1 = (0<<ICIE1);																// Disable Input Capture Interrupt
}