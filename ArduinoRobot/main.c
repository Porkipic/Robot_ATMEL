////////////////////////////// DEFINITIONS ////////////////////////////////////////
#define F_CPU							16000000UL	// Define CPU speed
#define TIMEUNIT						10			// Time unit in us

#define OUTPUT							1		// Used for setPin() function
#define INPUT							0		// Used for setPin() function
#define HIGH							1		// Used for setPin() function
#define LOW								0		// Used for setPin() function
#define PULLUP_ENABLED					1		// Used for setPin() function
#define NO_PULLUP						0		// Used for setPin() function

#define SUPERFAST						1		// Used to set Max PWM frequency in init8BitsPWM() function (Max 62500 Hz)
#define FAST							8		// Used to set Max PWM frequency in init8BitsPWM() function (Max 7812 Hz)
#define MEDIUM							64		// Used to set Max PWM frequency in init8BitsPWM() function (Max 976 Hz)
#define SLOW							256		// Used to set Max PWM frequency in init8BitsPWM() function (Max 244 Hz)
#define SUPERSLOW						1024	// Used to set Max PWM frequency in init8BitsPWM() function (Max 61 Hz)

#define FALLING							0		// Used for Input Capture Edge Selection
#define RISING							1		// Used for Input Capture Edge Selection

#define TIMER0							0		// Used for PWM initialization
#define TIMER1							1		// Used for PWM initialization
#define TIMER2							2		// Used for PWM initialization

#define AHI								11		// HIP4081 input pins
#define ALI								12		// HIP4081 input pins
#define BHI								15		// HIP4081 input pins
#define BLI								16		// HIP4081 input pins
////////////////////////////// END DEFINITIONS ////////////////////////////////////

////////////////////////////// INCLUDES ///////////////////////////////////////////
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
////////////////////////////// END INCLUDES ///////////////////////////////////////

////////////////////////////// ISR FLAGS //////////////////////////////////////////
volatile uint8_t ISR_INT0				= 0;	// External Interrupt 0
volatile uint8_t ISR_INT1				= 0;	// External Interrupt 1
volatile uint8_t ISR_PCINT0				= 0;	// Pin Change Interrupt Request 0
volatile uint8_t ISR_PCINT1				= 0;	// Pin Change Interrupt Request 1
volatile uint8_t ISR_PCINT2				= 0;	// Pin Change Interrupt Request 2
volatile uint8_t ISR_WDT				= 0;	// Watchdog Time-out Interrupt
volatile uint8_t ISR_TMR2CA				= 0;	// Timer/Counter 2 Compare Match A
volatile uint8_t ISR_TMR2CB				= 0;	// Timer/Counter 2 Compare Match B
volatile uint8_t ISR_TMR2OVF			= 0;	// Timer/Counter 2 Overflow
volatile uint8_t ISR_TMR1CAPT			= 0;	// Timer/Counter 1 Capture Event
volatile uint8_t ISR_TMR1CA				= 0;	// Timer/Counter 1 Compare Match A
volatile uint8_t ISR_TMR1CB				= 0;	// Timer/Counter 1 Compare Match B
volatile uint8_t ISR_TMR1OVF			= 0;	// Timer/Counter 1 Overflow
volatile uint8_t ISR_TMR0CA				= 0;	// Timer/Counter 0 Compare Match A
volatile uint8_t ISR_TMR0CB				= 0;	// Timer/Counter 0 Compare Match B
volatile uint8_t ISR_TMR0OVF			= 0;	// Timer/Counter 0 Overflow
volatile uint8_t ISR_SPI				= 0;	// SPI Serial Transfer Complete
volatile uint8_t ISR_USARTRX			= 0;	// USART RX Complete
volatile uint8_t ISR_USARTUDRE			= 0;	// USART Data Register Empty
volatile uint8_t ISR_USARTTX			= 0;	// USART TX Complete
volatile uint8_t ISR_ADC				= 0;	// ADC Conversion Complete
volatile uint8_t ISR_EEREADY			= 0;	// EEPROM Ready
volatile uint8_t ISR_ANALOGCOMP			= 0;	// Analog Comparator
volatile uint8_t ISR_TWI				= 0;	// 2-wire Serial Interface
volatile uint8_t ISR_SPM				= 0;	// Store Program Memory Ready
////////////////////////////// END ISR FLAGS //////////////////////////////////////

////////////////////////////// CONSTANTS DECLARATION //////////////////////////////
const uint8_t minSpeed 					= 100;	// Minimum speed value (motors stall under the value). To be determined and adjusted for different motors
const uint16_t maxSpeed 				= 300;	// Maximum speed value (motors cannot turn faster). To be determined and adjusted for different motors
const uint8_t PIDPeriod					= 100;	// Minimum time between PID computing (max 255 ms)
const uint16_t kp						= 500;	// P factor used for PID (increase = faster response, worst stability)
const uint16_t ki						= 25;	// I factor used for PID (increase = faster response, worst stability, eliminate steady-state error)
const uint16_t kd						= 0;	// D factor used for PID (increase = slower response, better stability)
const uint16_t PIDAttenuation			= 1000;	// Reduce the overall influence of the PID value on the motor speed
////////////////////////////// END CONSTANTS DECLARATION //////////////////////////

////////////////////////////// VARIABLES DECLARATION //////////////////////////////
volatile uint16_t result_ADC			= 0;	// ADC result holder (updated by ISR)

volatile uint32_t pulsesEncCHA			= 0;	// Pulse counter for encoder channel A (updated by ISR)
volatile uint32_t pulsesEncCHB			= 0;	// Pulse counter for encoder channel B (updated by ISR)
volatile uint8_t lastInterruptChannel	= 0;	// Which encoder channel triggered the last external interrupt (updated by ISR)
uint16_t deltaTimeCHA					= 0;	// Time between encoder Channel A pulses
uint16_t deltaTimeCHB					= 0;	// Time between encoder Channel A pulses

volatile uint32_t millisSeconds			= 0;	// Elapsed milliseconds since boot (updated by ISR)
volatile uint16_t elapsed				= 0;
volatile uint32_t microsSeconds			= 0;	// Elapsed microseconds since boot (updated by ISR)
volatile uint16_t micros1000s			= 0;	// Rolling counter for 1000s of microseconds (updated by ISR)
volatile uint16_t inputCaptureTimeCHA	= 0;	// Input capture timestamp (updated by ISR)
volatile uint16_t inputCaptureTimeCHB	= 0;	// Input capture timestamp (updated by ISR)
volatile uint16_t prevCaptureTimeCHA	= 0;	// Previous Input capture timestamp (updated by ISR)
volatile uint16_t prevCaptureTimeCHB	= 0;	// Previous Input capture timestamp (updated by ISR)

uint16_t prescalerPWM					= 0;	// Input Capture time unit in nanoseconds
uint16_t prescalerTimer					= 0;	// Input Capture time unit in nanoseconds
uint16_t prescalerInputCapture			= 0;	// Input Capture time unit in nanoseconds
uint16_t inputCaptureTimeUnit			= 0;	// Time unit in nanoseconds

uint8_t errorPin						= 18;	// Pin used to control the error LED
uint8_t errorCode						= 0;

uint16_t temperatureHBridge				= 0;
uint16_t temperatureMotor				= 0;
uint16_t currentHBridge					= 0;
uint8_t channelADCinUse					= 0;
////////////////////////////// END VARIABLES DECLARATION //////////////////////////

////////////////////////////// FUNCTIONS DECLARATION //////////////////////////////
void setPin(uint8_t pin, uint8_t direction, uint8_t option);

void initADC();
void startADC(uint8_t channel);

void init8BitsPWM(uint8_t timer, uint16_t maxFrequency);
void setPWM(uint8_t pinPWM, uint8_t valuePWM);

void initErrorPin(uint8_t errorPin);
void setError(uint8_t code);
void clearError();

void initTimer();
void initInputCapture();
////////////////////////////// END FUNCTIONS DECLARATION //////////////////////////

int main (void){
////////////////////////////// SETUP //////////////////////////////////////////////
	//********** Pins configuration **********
	DDRB	= 0b00000000;						// Set pin direction (1=OUTPUT, 0=INPUT)
	DDRC	= 0b00000000;						//
	DDRD	= 0b11111111;						//
	
	PORTB	= 0b00000000;						// Set pin state :
	PORTC	= 0b00000000;						// if OUTPUT: 1= HIGH, 0= LOWs
	PORTD	= 0b00000000;						// if INPUT: 1= Pull-up on, 0= Pull-up off
	//********************

	//********** Services Initialization **********
	initADC();
	initErrorPin(errorPin);
	init8BitsPWM(TIMER0, SUPERSLOW);		// Initialize PWM Timer with frequency limiter (Hz)
	initTimer();
	sei();										// Enable Global Interrupt
	//********************
////////////////////////////// END SETUP /////////////////////////////////////////
	
	while(1) {
////////////////////////////// MAIN LOOP /////////////////////////////////////////		
		//********** ISR flags checks **********
		if(ISR_TMR1CAPT){
			switch (lastInterruptChannel){
				case 0:
					if (prevCaptureTimeCHA>inputCaptureTimeCHA){						// Prevents error when counter overflows
						deltaTimeCHA = (65536-prevCaptureTimeCHA)+inputCaptureTimeCHA;	// 65536 = MAX counter value
					}else{
						deltaTimeCHA = inputCaptureTimeCHA-prevCaptureTimeCHA;
					}
				break;
				case 1:
					if (prevCaptureTimeCHB>inputCaptureTimeCHB){
						deltaTimeCHB = (65536-prevCaptureTimeCHB)+inputCaptureTimeCHB;
					}else{
						deltaTimeCHB = inputCaptureTimeCHB-prevCaptureTimeCHB;
					}
				break;
			}
		}
		if(ISR_ADC){
			switch (channelADCinUse){
				case 0:
					temperatureHBridge = result_ADC;
				break;
				case 1:
					temperatureMotor = result_ADC;
				break;
				case 2:
					currentHBridge = result_ADC;
				break;
				default:
					setError(4);
				break;
			}
			ISR_ADC = 0;
		}
		//********************
		
		if (!ISR_ADC){
			startADC(0);
		}
		setPWM(11, result_ADC>>2);
		if (elapsed>1000){
			PORTB ^= (1<<PORTB4);
			elapsed -=1000;
		}
////////////////////////////// END MAIN LOOP /////////////////////////////////////
	}
	return 0;
}

////////////////////////////// FUNCTIONS DEFINITIONS //////////////////////////////
void setPin(uint8_t pin, uint8_t direction, uint8_t option){
	uint8_t bitOrder = 0;
	if (pin>=2 && pin<= 13){						// Determine BIT order and PORT based on Pin number (Pin 6 is PORTD, BIT 4)
		bitOrder = pin-2;
		DDRD |= (direction<<bitOrder);				// Set Pin as 1=output; 0=input
		PORTD |= (option<<bitOrder);				// Pin High/Low (output) or Pull-up enabled/disabled (input)
	}
	if (pin>=14 && pin<= 19){						// Determine BIT order and PORT based on Pin number (Pin 19 is PORTB, BIT 5)
		bitOrder = pin-14;
		DDRB |= (direction<<bitOrder);				// Set Pin as 1=output; 0=input
		PORTD |= (option<<bitOrder);				// Pin High/Low (output) or Pull-up enabled/disabled (input)
	}
	if (pin>=23 && pin<= 28){						// Determine BIT order and PORT based on Pin number (Pin 25 is PORTC, Bit 2)
		bitOrder = pin-23;
		DDRC |= (direction<<bitOrder);				// Set Pin as 1=output; 0=input
		PORTD |= (option<<bitOrder);				// Pin High/Low (output) or Pull-up enabled/disabled (input)
	}
}

void initADC(){
	ISR_ADC = 0;									// Reset to allow new conversion after initialization
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);	// Enable ADC, Enable Interrupt, Set prescaler to 128
}
void startADC(uint8_t channel){
	if (channel<8){
		if (ISR_ADC){									// Prevent new conversion if previous is not cleared
			setError(1);								// Set error in Error manager (1 = error in ADC)
		}else{
			ADMUX = (1 << REFS0) | (channel << MUX0);	// Set voltage reference (VCC), set ADC channel
			ADCSRA |= (1 << ADSC);						// Start conversion
			channelADCinUse = channel;
		}
	}else{
		setError(5);
	}
}

void initErrorPin(uint8_t pin){						// Set the Error pin on the right port based on the Pin number
	errorPin = pin;
	setPin(pin, OUTPUT, LOW);
}
void setError(uint8_t code){
	errorCode = code;								// Set Error code to the code used when the function has been called
	setPin(errorPin, OUTPUT,HIGH);
}
void clearError(){
	setPin(errorPin, OUTPUT,LOW);
}

void init8BitsPWM(uint8_t timer, uint16_t prescaler){
	prescalerPWM = prescaler;
	switch(timer){
		case 0:
			TCCR0A |= (1<<WGM01) | (1<<WGM00);						// Set mode to fast PWM
			switch (prescaler){
				case SUPERFAST:
					TCCR0B |= (0<<CS02) | (0<<CS01) | (1<<CS00);	// Set prescaler on clock (no prescaler)
				break;
				case FAST:
					TCCR0B |= (0<<CS02) | (1<<CS01) | (0<<CS00);	// Set prescaler on clock (CLKio/8)
				break;
				case MEDIUM:
					TCCR0B |= (0<<CS02) | (1<<CS01) | (1<<CS00);	// Set prescaler on clock (CLKio/64)
				break;
				case SLOW:
					TCCR0B |= (1<<CS02) | (0<<CS01) | (0<<CS00);	// Set prescaler on clock (CLKio/256)
				break;
				case SUPERSLOW:
					TCCR0B |= (1<<CS02) | (0<<CS01) | (1<<CS00);	// Set prescaler on clock (CLKio/1024)
				break;
				default:
					TCCR0B |= (1<<CS02) | (0<<CS01) | (1<<CS00);	// Default to SUPERSLOW PWM frequency
				break;
			}
			OCR0A = 0;												// Reset Compare A value
			OCR0B = 0;												// Reset Compare B value
		break;
		case 1:
			TCCR1A |= (1<<WGM11) | (1<<WGM10);						// Set mode to fast PWM
			switch(prescaler){
				case SUPERFAST:
					TCCR1B |= (0<<CS12) | (0<<CS11) | (1<<CS10);	// Set prescaler on clock (no prescaler)
				break;
				case FAST:
					TCCR1B |= (0<<CS12) | (1<<CS11) | (0<<CS10);	// Set prescaler on clock (CLKio/8)
				break;
				case MEDIUM:
					TCCR1B |= (0<<CS12) | (1<<CS11) | (1<<CS10);	// Set prescaler on clock (CLKio/64)
				break;
				case SLOW:
					TCCR1B |= (1<<CS12) | (0<<CS11) | (0<<CS10);	// Set prescaler on clock (CLKio/256)
				break;
				case SUPERSLOW:
					TCCR1B |= (1<<CS12) | (0<<CS11) | (1<<CS10);	// Set prescaler on clock (CLKio/1024)
				default:
					TCCR1B |= (1<<CS12) | (0<<CS11) | (1<<CS10);	// Default to SUPERSLOW PWM frequency
				break;
			}
			OCR1A = 0;												// Reset Compare A value
			OCR1B = 0;												// Reset Compare B value
		default:
			setError(5);
		break;
		
	}
}
void setPWM(uint8_t pinPWM, uint8_t valuePWM){
	switch(pinPWM){
		case 11:									// Timer 0 (8 bits)
			if (valuePWM == 0){
				TCCR0A &= ~(1<<COM0B1);				// Deactivate PWM
				OCR0B = 0;
				PORTD &= ~(1<<PORTD5);				// Set PWM pin Low
			}else if (valuePWM != OCR0B){
				OCR0B = valuePWM;					// Set Compare B value. An interrupt will happen when the counter will reach this value
				TCCR0A |= (1<<COM0B1);				// Activate PWM
			}
		break;
		case 12:									// Timer 0 (8 bits)
			if (valuePWM == 0){
				TCCR0A &= ~(1<<COM0A1);				// Deactivate PWM
				OCR0A = 0;
				PORTD &= ~(1<<PORTD6);				// Set PWM pin Low
			}else if(valuePWM != OCR0A){
				OCR0A = valuePWM;					// Set Compare A value. An interrupt will happen when the counter will reach this value
				TCCR0A |= (1<<COM0A1);				// Activate PWM
			}
		break;
		case 15:									// Timer 1 (16 bits)
			if (valuePWM == 0){
				TCCR1A &= ~(1<<COM1A1);				// Deactivate PWM
				OCR1A = 0;
				PORTB &= ~(1<<PORTB1);				// Set PWM pin Low
			}else if(valuePWM != OCR1A){
				OCR1A = valuePWM;					// Set Compare A value. An interrupt will happen when the counter will reach this value
				TCCR1A |= (1<<COM1A1);				// Activate PWM
			}
		break;
		case 16:									// Timer 1 (16 bits)
			if (valuePWM == 0){
				TCCR1B &= ~(1<<COM1B1);				// Deactivate PWM
				OCR1B = 0;
				PORTB &= ~(1<<PORTB2);				// Set PWM pin Low
			}else if(valuePWM != OCR1B){
				OCR1B = valuePWM;					// Set Compare A value. An interrupt will happen when the counter will reach this value
				TCCR1A |= (1<<COM1B1);				// Activate PWM
			}
		break;
		default:									// If invalid pin number, generate error and disable all PWM.
			TCCR0A &= ~(1<<COM0B1);					// Deactivate PWM on pin 11
			setPin(11, OUTPUT, LOW);
			TCCR0A &= ~(1<<COM0A1);					// Deactivate PWM on pin 12
			setPin(12, OUTPUT, LOW);
			TCCR1A &= ~(1<<COM1A1);					// Deactivate PWM on pin 15
			setPin(15, OUTPUT, LOW);
			TCCR1A &= ~(1<<COM1B1);					// Deactivate PWM on pin 16
			setPin(16, OUTPUT, LOW);
			setError(3);
		break;
	}
}

void initTimer(uint16_t prescaler){
	prescalerTimer = prescaler;
	TCCR2A = (1<<WGM21);						// Timer1 CTC mode OCR2A as MAX value (WGMx)						
	TCCR2B = (1<<CS21);							// 8 prescaler (CTC)
	uint8_t CTCTopValue = (F_CPU/(2*prescaler*(1000000/(TIMEUNIT*2))))-1;	// (F_CPU/2*prescaler*Frequency)-1
	if (CTCTopValue>255){						// 8 bits timer, TOP value cannot exceed 255
		setError(6);
	}else{
		OCR2A = CTCTopValue;					// Set OCR2A to calculated TOP value			
		TIMSK2 = (1<<OCIE2A);					// Enable interrupt when counter reaches ICR1
		TCNT2 = 0;								// Timer1 counter reset
	}
}
void initInputCapture(uint16_t prescaler){
	prescalerInputCapture = prescaler;
	inputCaptureTimeUnit = 
	TCCR1B = (1<<ICNC1) | (1<<ICES1) | (1<<CS10);	// Input capture noise canceler enabled, detect on rising edge, no prescaling
	TIMSK1 = (1<<ICIE1);
}

uint8_t calculateFrequency(uint16_t deltaTime){
	uint8_t result = 0;
	result = 1/deltaTime;
	return result;
}
////////////////////////////// END FUNCTIONS DEFINITIONS //////////////////////////

////////////////////////////// ISRs //////////////////////////
ISR(INT0_vect){									// External Interrupt Request 0
	pulsesEncCHA ++;
	lastInterruptChannel = 0;
}
ISR(INT1_vect){									// External Interrupt Request 1
	pulsesEncCHB ++;
	lastInterruptChannel = 1;
}
ISR(PCINT0_vect){								// Pin Change Interrupt Request 0
	ISR_PCINT0 = 0;
}
ISR(PCINT1_vect){								// Pin Change Interrupt Request 1
	ISR_PCINT1 = 0;
}
ISR(PCINT2_vect){								// Pin Change Interrupt Request 2
	ISR_PCINT2 = 0;
}
ISR(WDT_vect){									// Watchdog Time-out Interrupt
	ISR_WDT = 0;
}
ISR(TIMER2_COMPA_vect){							// Timer/Counter 2 Compare Match A
	microsSeconds += TIMEUNIT;					// Increment the number of microseconds by the TIMEUNIT
	micros1000s += TIMEUNIT;					// Increment rolling counter by TIMEUNIT
	if (micros1000s>=1000){
		millisSeconds++;						// Increments milliseconds counter every 1000s of microseconds
		micros1000s -= 1000;					// Adds the any extra microseconds to the rolling counter
		elapsed ++;
	}
}
ISR(TIMER2_COMPB_vect){							// Timer/Counter 2 Compare Match B
	ISR_TMR2CB = 0;
}
ISR(TIMER2_OVF_vect){							// Timer/Counter 2 Overflow
	ISR_TMR2OVF = 0;
}
ISR(TIMER1_CAPT_vect){							// Timer/Counter 1 Capture Event
	ISR_TMR1CAPT = 1;
	switch (lastInterruptChannel){						//Check which encoder channel sent triggered the Input Capture
		case 0:
			prevCaptureTimeCHA = inputCaptureTimeCHA;	// Move current timestamp to the previous one
			inputCaptureTimeCHA = ICR1H;				// Update current timestamp
		break;
		case 1:
			prevCaptureTimeCHB = inputCaptureTimeCHB;	// Move current timestamp to the previous one
			inputCaptureTimeCHB = ICR1H;				// Update current timestamp
		break;
	}
}
ISR(TIMER1_COMPA_vect){							// Timer/Counter 1 Compare Match A
	ISR_TMR1CA = 0;
}
ISR(TIMER1_COMPB_vect){							// Timer/Counter 1 Compare Match B
	ISR_TMR1CB = 0;
}
ISR(TIMER1_OVF_vect){							// Timer/Counter 1 Overflow
	ISR_TMR1OVF = 0;
}
ISR(TIMER0_COMPA_vect){							// Timer/Counter 0 Compare Match A
	ISR_TMR0CA = 0;
}
ISR(TIMER0_COMPB_vect){							// Timer/Counter 0 Compare Match B
	ISR_TMR0CB = 0;
}
ISR(TIMER0_OVF_vect){							// Timer/Counter 0 Overflow
	ISR_TMR0OVF = 0;
}
ISR(SPI_STC_vect){								// SPI Serial Transfer Complete
	ISR_SPI = 0;
}
ISR(USART_RX_vect){								// USART Rx Complete
	ISR_USARTRX = 0;
}
ISR(USART_UDRE_vect){							// USART Data Register Empty
	ISR_USARTUDRE = 0;
}
ISR(USART_TX_vect){								// USART TX Complete
	ISR_USARTTX = 0;
}
ISR(ADC_vect){									// ADC Conversion Complete
	result_ADC = ADC;
	ISR_ADC = 1;
}
ISR(EE_READY_vect){								// EEPROM Ready
	ISR_EEREADY = 0;
}
ISR(ANALOG_COMP_vect){							// Analog Comparator
	ISR_ANALOGCOMP = 0;
}
ISR(TWI_vect){									// 2-wire Serial Interface
	ISR_TWI = 0;
}
ISR(SPM_READY_vect){							// Store Program Memory Ready
	ISR_SPM = 0;
}
////////////////////////////// END ISRs //////////////////////////