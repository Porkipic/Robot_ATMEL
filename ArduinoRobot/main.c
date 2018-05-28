#define F_CPU					16000000UL						// Define CPU speed
#define PRESCALER				8								// Timer pre-scaler
#define TIMEUNIT				(PRESCALER/(F_CPU/1000000))*256	// Smallest time unit in us

////////////////////////////// INCLUDES ///////////////////////////////////////////
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
////////////////////////////// END INCLUDES ///////////////////////////////////////

////////////////////////////// DEFINITIONS ////////////////////////////////////////
#define BACKWARD				0
#define FORWARD					1
#define FALLING					0		// Used for Input Capture Edge Selection
#define RISING					1		// Used for Input Capture Edge Selection
#define AHI						11		// HIP4081 input pins
#define ALI						12		// HIP4081 input pins
#define BHI						15		// HIP4081 input pins
#define BLI						16		// HIP4081 input pins
////////////////////////////// END DEFINITIONS ////////////////////////////////////

////////////////////////////// ISR FLAGS //////////////////////////////////////////
volatile uint8_t ISR_PCINT0		= 0;	// Pin Change Interrupt Request 0
volatile uint8_t ISR_PCINT1		= 0;	// Pin Change Interrupt Request 1
volatile uint8_t ISR_PCINT2		= 0;	// Pin Change Interrupt Request 2
volatile uint8_t ISR_WDT		= 0;	// Watchdog Time-out Interrupt
volatile uint8_t ISR_TMR2CA		= 0;	// Timer/Counter 2 Compare Match A
volatile uint8_t ISR_TMR2CB		= 0;	// Timer/Counter 2 Compare Match B
volatile uint8_t ISR_TMR2OVF	= 0;	// Timer/Counter 2 Overflow
volatile uint8_t ISR_TMR1CAPT	= 0;	// Timer/Counter 1 Capture Event
volatile uint8_t ISR_TMR1CA		= 0;	// Timer/Counter 1 Compare Match A
volatile uint8_t ISR_TMR1CB		= 0;	// Timer/Counter 1 Compare Match B
volatile uint8_t ISR_TMR1OVF	= 0;	// Timer/Counter 1 Overflow
volatile uint8_t ISR_TMR0CA		= 0;	// Timer/Counter 0 Compare Match A
volatile uint8_t ISR_TMR0CB		= 0;	// Timer/Counter 0 Compare Match B
volatile uint8_t ISR_TMR0OVF	= 0;	// Timer/Counter 0 Overflow
volatile uint8_t ISR_SPI		= 0;	// SPI Serial Transfer Complete
volatile uint8_t ISR_USARTRX	= 0;	// USART Rx Complete
volatile uint8_t ISR_USARTUDRE	= 0;	// USART Data Register Empty
volatile uint8_t ISR_USARTTX	= 0;	// USART Tx Complete
volatile uint8_t ISR_ADC		= 0;	// ADC Conversion Complete
volatile uint8_t ISR_EEREADY	= 0;	// EEPROM Ready
volatile uint8_t ISR_ANALOGCOMP	= 0;	// Analog Comparator
volatile uint8_t ISR_TWI		= 0;	// 2-wire Serial Interface
volatile uint8_t ISR_SPM		= 0;	// Store Program Memory Ready
////////////////////////////// END ISR FLAGS //////////////////////////////////////

////////////////////////////// CONSTANTS DECLARATION //////////////////////////////
const uint8_t minSpeed 			= 100;	// Minimum speed value (motors stall under the value). To be determined and adjusted for different motors
const uint16_t maxSpeed 		= 300;	// Maximum speed value (motors cannot turn faster). To be determined and adjusted for different motors
const uint8_t PIDPeriod			= 100;	// Minimum time between PID computing (max 255 ms)
const uint16_t kp				= 500;	// P factor used for PID (increase = faster response, worst stability)
const uint16_t ki				= 25;	// I factor used for PID (increase = faster response, worst stability, eliminate steady-state error)
const uint16_t kd				= 0;	// D factor used for PID (increase = slower response, better stability)
const uint16_t PIDAttenuation	= 1000;	// Reduce the overall influence of the PID value on the motor speed
////////////////////////////// END CONSTANTS DECLARATION //////////////////////////

////////////////////////////// VARIABLES DECLARATION //////////////////////////////
volatile uint16_t result_ADC	= 0;	// ADC result holder (updated by ISR)

volatile uint32_t pulsesEncCHA	= 0;	// Pulse counter for encoder channel A (updated by ISR)
volatile uint32_t pulsesEncCHB	= 0;	// Pulse counter for encoder channel B (updated by ISR)

volatile uint32_t millisSeconds	= 0;	// Elapsed milliseconds since boot (updated by ISR)
volatile uint32_t microsSeconds	= 0;	// Elapsed microseconds since boot (updated by ISR)
volatile uint16_t micros1000s	= 0;	// Rolling counter for 1000s of microseconds (updated by ISR)
volatile uint16_t captTimestamp	= 0;	// 

uint8_t errorPin				= 0;	// Pin used to control the error LED
uint8_t errorPort				= 0;	// Port on which the LED is attached
uint8_t errorCode				= 0;	// Error code

uint8_t speedPIDSetpointRight	= 128; 	// Speed set point for motor A (min 0, max 255)
uint8_t speedPIDSetpointLeft	= 128; 	// Speed set point for motor B (min 0, max 255)
uint16_t previousTickEncRight	= 0;	// Previous tick count A
uint16_t previousTickEncLeft	= 0;	// Previous tick count B
uint16_t PIDIntegratorRight		= 0;	// Sum of PID error A for I computing
uint16_t PIDIntegratorLeft		= 0;	// Sum of PID error B for I computing
uint16_t previousPIDErrorRight	= 0;	// Previous error in tick count A for D computing
uint16_t previousPIDErrorLeft	= 0;	// Previous error in tick count B for D computing
uint16_t timestampEncRight		= 0;	// Timestamp A for duration computing
uint16_t timestampEncLeft		= 0;	// Timestamp B for duration computing
////////////////////////////// END VARIABLES DECLARATION //////////////////////////

////////////////////////////// FUNCTIONS DECLARATION //////////////////////////////
void motorCommand(uint8_t motorDirection, uint8_t motorSpeed);

void initADC();
void startADC(uint8_t channel);

void initPWM();
void setPWM(uint8_t pinPWM, uint8_t valuePWM);

void initErrorPin(uint8_t errorPin);
void setError(uint8_t errorPin);
void clearError();

void initTimer();
void startPulseLength(uint8_t state, uint16_t timeout);
uint16_t getPulseLength();
////////////////////////////// END FUNCTIONS DECLARATION //////////////////////////

int main (void){
////////////////////////////// SETUP //////////////////////////////////////////////
	//********** Pins configuration **********
	DDRB	= 0b11111111;				// Set pin direction (1=OUTPUT, 0=INPUT)
	DDRC	= 0b11111111;				// Set pin state :
	DDRD	= 0b01100000;				// if OUTPUT: 1= HIGH, 0= LOW		if INPUT: 1= Pullup on,	0= Pullup off
	PORTB	= 0b00000101;				// 
	PORTC	= 0b00000000;				//
	PORTD	= 0b00000000;				//
	//********************

	//********** Services Initialization **********
	initPWM();
	initADC();
	initErrorPin(12);
	initTimer();
	//********************
	sei();								// Enable Global Interrupt
////////////////////////////// END SETUP /////////////////////////////////////////
	
	while(1) {
////////////////////////////// MAIN LOOP /////////////////////////////////////////		
		//********** ISR flags checks **********
		if(ISR_PCINT0){
		}
		if(ISR_PCINT1){
		}
		if(ISR_PCINT2){
		}
		if(ISR_WDT){
		}
		if(ISR_TMR2CA){
		}
		if(ISR_TMR2CB){
		}
		if(ISR_TMR2OVF){
		}
		if(ISR_TMR1CAPT){
		}
		if(ISR_TMR1CA){
		}
		if(ISR_TMR1CB){
		}
		if(ISR_TMR1OVF){
		}
		if(ISR_TMR0CA){
		}
		if(ISR_TMR0CB){
		}
		if(ISR_TMR0OVF){
		}
		if(ISR_SPI){
		}
		if(ISR_USARTRX){
		}
		if(ISR_USARTUDRE){
		}
		if(ISR_USARTTX){
		}
		if(ISR_ADC){
			//TODO What to do with ADC result.
			ISR_ADC = 0;
		}
		if(ISR_EEREADY){
		}
		if(ISR_ANALOGCOMP){
		}
		if(ISR_TWI){
		}
		if(ISR_SPM){
		}
		//********************
		
		motorCommand(FORWARD, 255);
		_delay_ms(1000);
		motorCommand(FORWARD, 0);
		_delay_ms(1000);
		motorCommand(BACKWARD, 255);
		_delay_ms(1000);
		motorCommand(BACKWARD, 0);
		_delay_ms(1000);
////////////////////////////// END MAIN LOOP /////////////////////////////////////
	}
	return 0;
}

////////////////////////////// FUNCTIONS DEFINITIONS //////////////////////////////
void initADC(){
	ISR_ADC = 0;																		// Reset to allow new conversion after initialization
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);	// Enable ADC, Enable Interrupt, Set prescaler to 128
}
void startADC(uint8_t channel){
	if (ISR_ADC){																		// Prevent new conversion if previous is not cleared
		setError(1);																	// Set error in Error manager (1 = error in ADC)
	}else{
		ADMUX = (1 << REFS0) | (channel << MUX0);										// Set voltage reference (VCC), set ADC channel
		ADCSRA |= (1 << ADSC);															// Start conversion
	}
}

void initPWM(uint8_t timer){
	switch (timer){
		case 0:
			TCCR0A |= (1<<WGM01) | (1<<WGM00);						// Set mode to fast PWM
			TCCR0B |= (PRESCALER<<CS02);							// Set prescaler on clock
			OCR0A = 0;												// Reset Compare A value
			OCR0B = 0;												// Reset Compare B value
		break;
		case 1:
			TCCR1A |= (1<<WGM11) | (1<<WGM10);						// Set mode to fast PWM
			TCCR1B |= (PRESCALER<<CS12);							// Set prescaler on clock
			OCR1A = 0;												// Reset Compare A value
			OCR1B = 0;												// Reset Compare B value
		break;
		default:
			setError(3);											// If Timer number is invalid, generate error
		break;
	}
}
void setPWM(uint8_t pinPWM, uint8_t valuePWM){
	switch(pinPWM){
		case 11:													// Timer 0 (8 bits)
			if (valuePWM == 0){
				TCCR0A &= ~(1<<COM0B1);								// Deactivate PWM
				OCR0B = 0;
				PORTD &= ~(1<<PORTD5);								// Set PWM pin Low
			}else if (valuePWM != OCR0B){
				OCR0B = valuePWM;									// Set Compare B value. An interrupt will happen when the counter will reach this value
				TCCR0A |= (1<<COM0B1);								// Activate PWM
			}
		break;
		case 12:													// Timer 0 (8 bits)
			if (valuePWM == 0){
				TCCR0A &= ~(1<<COM0A1);								// Deactivate PWM
				OCR0A = 0;
				PORTD &= ~(1<<PORTD6);								// Set PWM pin Low
			}else if(valuePWM != OCR0A){
				OCR0A = valuePWM;									// Set Compare A value. An interrupt will happen when the counter will reach this value
				TCCR0A |= (1<<COM0A1);								// Activate PWM
			}
		break;
		case 15:													// Timer 1 (16 bits)
			if (valuePWM == 0){
				TCCR1A &= ~(1<<COM1A1);								// Deactivate PWM
				OCR1A = 0;
				PORTB &= ~(1<<PORTB1);								// Set PWM pin Low
			}else if(valuePWM != OCR1A){
				OCR1A = valuePWM;									// Set Compare A value. An interrupt will happen when the counter will reach this value
				TCCR1A |= (1<<COM1A1);								// Activate PWM
			}
		break;
		case 16:													// Timer 1 (16 bits)
			if (valuePWM == 0){
				TCCR1B &= ~(1<<COM1B1);								// Deactivate PWM
				OCR1B = 0;
				PORTB &= ~(1<<PORTB2);								// Set PWM pin Low
			}else if(valuePWM != OCR1B){
				OCR1B = valuePWM;									// Set Compare A value. An interrupt will happen when the counter will reach this value
				TCCR1B |= (1<<COM1B1);								// Activate PWM
			}
		break;
		default:													// If invalid pin number, generate error
			setError(3);
		break;
	}
}

void initErrorPin(uint8_t pin){													// Set the error pin on the right port based on the PIN number
	errorPin = pin;
	if (errorPin>=2 && errorPin<= 13){
		errorPort = 3;
		DDRD |= (1<<errorPin);
	}
	if (errorPin>=14 && errorPin<= 19){
		errorPort = 1;
		DDRB |= (1<<errorPin);
	}
	if (errorPin>=23 && errorPin<= 28){
		errorPort = 2;
		DDRC |= (1<<errorPin);
	}
}
void setError(uint8_t code){
	errorCode = code;
	switch(errorPort){
		case 0:
			initErrorPin(errorPin);
			setError(0);
		break;
		case 1:
			PORTC |= (1<<errorPin);
		break;
		case 2:
			PORTB |= (1<<errorPin);
		break;
		case 3:
			PORTD |= (1<<errorPin);
		break;
		default:
			initErrorPin(errorPin);
			setError(0);
		break;
	}
}
void clearError(){
	switch(errorPort){
		case 0:
			initErrorPin(errorPin);
			setError(0);
		break;
		case 1:
			PORTB &= (0<<errorPin);
		break;
		case 2:
			PORTC &= (0<<errorPin);
		break;
		case 3:
			PORTD &= (0<<errorPin);
		break;
		default:
			initErrorPin(errorPin);
			setError(0);
		break;
	}
}

void initTimer(){
	TCCR0A = (1<<WGM01);																// Timer0 CTC mode
	TCCR0B = (1<<CS00);																	// Timer0 no prescaling
	OCR0A = 7;																			// Timer0 OCR0A TOP value (see formula in datasheet for Normal Mode)
	TIMSK0 = (1<<TOIE0);																// Enable Timer 0 Timer Overflow Interrupt
	TCNT0 = 0;																			// Timer0 counter reset
	
	TCCR1B = (1<<WGM12) | (1<<CS10);													// Timer1 CTC mode OCRA1A TOP value, no prescaling
	TCNT1 = 0;																			// Timer1 counter reset
}
void startPulseLength(uint8_t state, uint16_t timeout){
	TCNT1 = 0;																			// Timer 1 counter reset
	OCR1A = timeout*7;																	// Set max measurable pulse length duration. TO BE CHECK WITH DATASHEET FORMULA
	TCCR1B = (state<<ICES1);															// Set which edge is detected
	TIMSK1 = (1<<ICIE1) | (1<<OCIE1A);													// Enable Input Capture Interrupt, enable Output Compare A interrupt
}
uint16_t getPulseLength(){
	ISR_TMR1CAPT = 0;
	return captTimestamp/2;																// Return the timestamp (/2 due to pre-scaler set to 8)  TODO recalculate the divider
}

void motorCommand(uint8_t motorDirection, uint8_t motorSpeed){
	switch (motorDirection){
		case FORWARD:
		setPWM(L298IN2, 0);														// Turn off PWM
		setPWM(L298IN1, motorSpeed);  											// PWM on driving pin at set speed (PID corrected)
		break;
		case BACKWARD:
		setPWM(L298IN1, 0);														// Turn off PWM
		setPWM(L298IN2, motorSpeed);  											// PWM on driving pin at set speed (PID corrected)
		break;
		default:
		// RAISE ERROR
		break;
	}
}
////////////////////////////// END FUNCTIONS DEFINITIONS //////////////////////////

////////////////////////////// ISRs //////////////////////////
ISR(INT0_vect){										// External Interrupt Request 0
	pulsesEncCHA ++;
}
ISR(INT1_vect){										// External Interrupt Request 1
	pulsesEncCHB ++;
}
ISR(PCINT0_vect){									// Pin Change Interrupt Request 0
	ISR_PCINT0 = 0;
}
ISR(PCINT1_vect){									// Pin Change Interrupt Request 1
	ISR_PCINT1 = 0;
}
ISR(PCINT2_vect){									// Pin Change Interrupt Request 2
	ISR_PCINT2 = 0;
}
ISR(WDT_vect){										// Watchdog Time-out Interrupt
	ISR_WDT = 0;
}
ISR(TIMER2_COMPA_vect){																	// Timer/Counter 2 Compare Match A
	ISR_TMR2CA = 0;
}
ISR(TIMER2_COMPB_vect){																	// Timer/Counter 2 Compare Match B
	ISR_TMR2CB = 0;
}
ISR(TIMER2_OVF_vect){																	// Timer/Counter 2 Overflow
	ISR_TMR2OVF = 0;
}
ISR(TIMER1_CAPT_vect){																	// Timer/Counter 1 Capture Event
	captTimestamp = ICR1;																// Return TIMER1_CAPT timestamp
	ISR_TMR1CAPT = 1;																	// Set ISR_TMR1CAPT extern variable. TO BE POLLED IN MAIN PROGRAM
	TIMSK1 = (0<<ICIE1);
}
ISR(TIMER1_COMPA_vect){								// Timer/Counter 1 Compare Match A
	ISR_TMR1CA = 0;
}
ISR(TIMER1_COMPB_vect){								// Timer/Counter 1 Compare Match B
	ISR_TMR1CB = 0;
}
ISR(TIMER1_OVF_vect){																	// Timer/Counter 1 Overflow
	captTimestamp = 0;																	// Return 0 if PulseLength>timeout
	ISR_TMR1CAPT = 1;																	// Set ISR_TMR1CAPT extern variable. TO BE POLLED IN MAIN PROGRAM
	TIMSK1 = (0<<ICIE1);
}
ISR(TIMER0_COMPA_vect){								// Timer/Counter 0 Compare Match A
	ISR_TMR0CA = 0;
}
ISR(TIMER0_COMPB_vect){								// Timer/Counter 0 Compare Match B
	ISR_TMR0CB = 0;
}
ISR(TIMER0_OVF_vect){																	// Timer/Counter 0 Overflow
	microsSeconds += TIMEUNIT;															// Increment the number of microseconds by the TIMEUNIT
	micros1000s += TIMEUNIT;															// Increment rolling counter by TIMEUNIT
	if (micros1000s>=1000){
		millisSeconds++;																// Increments mills every 1000s of microseconds
		micros1000s -= 1000;															// Adds the any extra microseconds to the rolling counter
	}
}
ISR(SPI_STC_vect){									// SPI Serial Transfer Complete
	ISR_SPI = 0;
}
ISR(USART_RX_vect){									// USART Rx Complete
	ISR_USARTRX = 0;
}
ISR(USART_UDRE_vect){								// USART Data Register Empty
	ISR_USARTUDRE = 0;
}
ISR(USART_TX_vect){									// USART TX Complete
	ISR_USARTTX = 0;
}
ISR(ADC_vect){										// ADC Conversion Complete
	result_ADC = ADC;
	ISR_ADC = 1;
}
ISR(EE_READY_vect){									// EEPROM Ready
	ISR_EEREADY = 0;
}
ISR(ANALOG_COMP_vect){								// Analog Comparator
	ISR_ANALOGCOMP = 0;
}
ISR(TWI_vect){										// 2-wire Serial Interface
	ISR_TWI = 0;
}
ISR(SPM_READY_vect){								// Store Program Memory Ready
	ISR_SPM = 0;
}
////////////////////////////// END ISRs //////////////////////////