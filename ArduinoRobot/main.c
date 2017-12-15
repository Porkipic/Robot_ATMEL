////////////////////////////// INCLUDES ///////////////////////////////////////////
#include <avr/io.h>
////////////////////////////// END INCLUDES ///////////////////////////////////////

////////////////////////////// CONSTANTS DECLARATION //////////////////////////////
const uint8_t minSpeed 			= 100;													// Minimum speed value (motors stall under the value). To be determined and adjusted for different motors
const uint8_t maxSpeed 			= 300;													// Maximum speed value (motors cannot turn faster). To be determined and adjusted for different motors
const uint8_t PIDPeriod			= 100;													// Minimum time between PID computing (max 255 ms)
const uint16_t kp				= 500;													// P factor used for PID (increase = faster response, worst stability)
const uint16_t ki				= 25;													// I factor used for PID (increase = faster response, worst stability, eliminate steady-state error)
const uint16_t kd				= 0;													// D factor used for PID (increase = slower response, better stability)
const uint16_t PIDAttenuation	= 1000;													// Reduce the overall influence of the PID value on the motor speed
////////////////////////////// END CONSTANTS DECLARATION //////////////////////////

////////////////////////////// VARIABLES DECLARATION //////////////////////////////
uint16_t loopCounter			= 0;													// Number of loops the uC has run
uint8_t speedPIDSetpointRight	= 128; 													// Speed setpoint for motor A (min 0, max 255)
uint8_t speedPIDSetpointLeft	= 128; 													// Speed setpoint for motor B (min 0, max 255)
volatile uint16_t tickEncRight	= 0;													// Tick counter for encoder on motor A (increased by ISR)
volatile uint16_t tickEncLeft	= 0;													// Tick counter for encoder on motor B (increased by ISR)
uint16_t previousTickEncRight	= 0;													// Previous tick count A
uint16_t previousTickEncLeft	= 0;													// Previous tick count B
uint16_t PIDIntegratorRight		= 0;													// Sum of PID error A for I computing
uint16_t PIDIntegratorLeft		= 0;													// Sum of PID error B for I computing
uint16_t previousPIDErrorRight	= 0;													// Previous error in tick count A for D computing
uint16_t previousPIDErrorLeft	= 0;													// Previous error in tick count B for D computing
uint16_t timestampEncRight		= 0;													// Timestamp A for duration computing
uint16_t timestampEncLeft		= 0;													// Timestamp B for duration computing
////////////////////////////// END VARIABLES DECLARATION //////////////////////////

int main (void){

	//DDRD = B11111110; // Set pin direction (1=output, 0=input).
	//PORTD = B10101000; // Set pin state (1=HIGH, 0=LOW).
	//DDRD = (1<<PD2); 

	//********** Pins configuration **********
	DDRB	= B00000110;																// Set pin direction (1=OUTPUT, 0=INPUT)
	PORTB	= B00000000;																// Set pin state :
	DDRC	= B00000000;																// - if OUTPUT:	1= HIGH, 		0= LOW
	PORTC	= B00000000;																// - if INPUT:	1= Pullup on,	0= Pullup off
	DDRD	= B11110000;																//
	PORTD	= B00001100;																//
	//********************
    
	//********** Interrupts **********
	attachInterrupt(digitalPinToInterrupt(encA), encAISR, CHANGE);	// Interrupt triggered on change on encoders pins
	attachInterrupt(digitalPinToInterrupt(encB), encBISR, CHANGE);	// Can be changed by RISING to reduce the number of interruptions
	//********************

	while(1) {

	}
	return 0;
}
