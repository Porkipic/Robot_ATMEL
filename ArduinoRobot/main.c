////////////////////////////// INCLUDES ///////////////////////////////////////////
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ADC.h"
////////////////////////////// END INCLUDES ///////////////////////////////////////

////////////////////////////// ISR FLAGS //////////////////////////////////////////
volatile uint8_t ISR_RESET		= 0;													// External Pin, Power-on Reset, Brown-out Reset and Watchdog System Reset
volatile uint8_t ISR_INT0		= 0;													// External Interrupt Request 0
volatile uint8_t ISR_INT1		= 0;													// External Interrupt Request 1
volatile uint8_t ISR_PCINT0		= 0;													// Pin Change Interrupt Request 0
volatile uint8_t ISR_PCINT1		= 0;													// Pin Change Interrupt Request 1
volatile uint8_t ISR_PCINT2		= 0;													// Pin Change Interrupt Request 2
volatile uint8_t ISR_WDT		= 0;													// Watchdog Time-out Interrupt
volatile uint8_t ISR_TMR2CA		= 0;													// Timer/Counter 2 Compare Match A
volatile uint8_t ISR_TMR2CB		= 0;													// Timer/Counter 2 Compare Match B
volatile uint8_t ISR_TMR2OVF	= 0;													// Timer/Counter 2 Overflow
volatile uint8_t ISR_TMR1CAPT	= 0;													// Timer/Counter 1 Capture Event
volatile uint8_t ISR_TMR1CA		= 0;													// Timer/Counter 1 Compare Match A
volatile uint8_t ISR_TMR1CB		= 0;													// Timer/Counter 1 Compare Match B
volatile uint8_t ISR_TMR1OVF	= 0;													// Timer/Counter 1 Overflow
volatile uint8_t ISR_TMR0CA		= 0;													// Timer/Counter 0 Compare Match A
volatile uint8_t ISR_TMR0CB		= 0;													// Timer/Counter 0 Compare Match B
volatile uint8_t ISR_TMR0OVF	= 0;													// Timer/Counter 0 Overflow
volatile uint8_t ISR_SPI		= 0;													// SPI Serial Transfer Complete
volatile uint8_t ISR_USARTRX	= 0;													// USART Rx Complete
volatile uint8_t ISR_USARTUDRE	= 0;													// USART Data Register Empty
volatile uint8_t ISR_USARTTX	= 0;													// USART Tx Complete
volatile uint8_t ISR_ADC		= 0;													// ADC Conversion Complete
volatile uint8_t ISR_EEREADY	= 0;													// EEPROM Ready
volatile uint8_t ISR_ANALOGCOMP	= 0;													// Analog Comparator
volatile uint8_t ISR_TWI		= 0;													// 2-wire Serial Interface
volatile uint8_t ISR_SPM		= 0;													// Store Program Memory Ready
////////////////////////////// END ISR FLAGS //////////////////////////////////////

////////////////////////////// CONSTANTS DECLARATION //////////////////////////////
const uint8_t minSpeed 			= 100;													// Minimum speed value (motors stall under the value). To be determined and adjusted for different motors
const uint16_t maxSpeed 		= 300;													// Maximum speed value (motors cannot turn faster). To be determined and adjusted for different motors
const uint8_t PIDPeriod			= 100;													// Minimum time between PID computing (max 255 ms)
const uint16_t kp				= 500;													// P factor used for PID (increase = faster response, worst stability)
const uint16_t ki				= 25;													// I factor used for PID (increase = faster response, worst stability, eliminate steady-state error)
const uint16_t kd				= 0;													// D factor used for PID (increase = slower response, better stability)
const uint16_t PIDAttenuation	= 1000;													// Reduce the overall influence of the PID value on the motor speed
////////////////////////////// END CONSTANTS DECLARATION //////////////////////////

////////////////////////////// VARIABLES DECLARATION //////////////////////////////
volatile uint16_t tickEncRight	= 0;													// Tick counter for encoder on motor A (increased by ISR)
volatile uint16_t tickEncLeft	= 0;													// Tick counter for encoder on motor B (increased by ISR)
uint16_t loopCounter			= 0;													// Number of loops the uC has run
uint8_t speedPIDSetpointRight	= 128; 													// Speed setpoint for motor A (min 0, max 255)
uint8_t speedPIDSetpointLeft	= 128; 													// Speed setpoint for motor B (min 0, max 255)
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
////////////////////////////// SETUP /////////////////////////////////////////////
	//********** Pins configuration **********
	DDRB	= 0b00000011;																// Set pin direction (1=OUTPUT, 0=INPUT)
	PORTB	= 0b00000000;																// Set pin state :
	DDRC	= 0b00000000;																// - if OUTPUT:	1= HIGH, 		0= LOW
	PORTC	= 0b00000000;																// - if INPUT:	1= Pullup on,	0= Pullup off
	DDRD	= 0b11111111;																//
	PORTD	= 0b00000000;																//
	//********************
	
	//********** Services Initialization **********
	initADC();
	//********************
	
	sei();																				// Enable Global Interrupts
	
////////////////////////////// END SETUP /////////////////////////////////////////
	while(1) {
////////////////////////////// MAIN LOOP /////////////////////////////////////////			
		if (!(PINB & (1 << PINB2))){
			startADC(ADC5);
		}
		//********** ISR flags checks **********
		if (ISR_ADC){
			uint16_t ADCValue = 0;
			ADCValue = handleConversion();
			PORTD = ADCValue;
			PORTB = ((ADCValue>>8)|0x04);
		}
		//********************
////////////////////////////// END MAIN LOOP /////////////////////////////////////
	}
	return 0;
}