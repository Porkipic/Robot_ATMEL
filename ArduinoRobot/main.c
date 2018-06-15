////////////////////////////// DEFINITIONS ////////////////////////////////////////
#define F_CPU							16000000UL	// Define CPU speed
#define TIMEUNIT						10			// Time unit in us
#define USARTBUFFER						8			// USART buffer size
#define BAUDRATE						9600		// USART baudrate
#define UBRR							((F_CPU/100)/(16*(BAUDRATE/100)))-1

#define OUTPUT							1		// Used for setPin() function
#define INPUT							0		// Used for setPin() function
#define HIGH							1		// Used for setPin() function
#define LOW								0		// Used for setPin() function
#define PULLUP							1		// Used for setPin() function
#define NO_PULLUP						0		// Used for setPin() function

#define READ							1		// Used for I2C direction bit
#define WRITE							1		// Used for I2C direction bit

#define SUPERFAST						1		// Used to set Max PWM frequency in init8BitsPWM() function (Max 62500 Hz)
#define FAST							8		// Used to set Max PWM frequency in init8BitsPWM() function (Max 7812 Hz)
#define MEDIUM							64		// Used to set Max PWM frequency in init8BitsPWM() function (Max 976 Hz)
#define SLOW							256		// Used to set Max PWM frequency in init8BitsPWM() function (Max 244 Hz)
#define SUPERSLOW						1024	// Used to set Max PWM frequency in init8BitsPWM() function (Max 61 Hz)

#define FALLING							1		// Used for Input Capture Edge Selection
#define RISING							0		// Used for Input Capture Edge Selection

#define TIMER0							0		// Used for PWM initialization
#define TIMER1							1		// Used for PWM initialization
#define TIMER2							2		// Used for PWM initialization

#define USART							0		// Used for PWM initialization
#define TIMER1							1		// Used for PWM initialization
#define TIMER2							2		// Used for PWM initialization

//********** Error Messages **********
#define INVALID_ADC_CHANNEL				10
#define PREVIOUS_ADC_NOT_READ			11
#define ADC_PREVIOUS_NOT_FINISHED		12

#define INVALID_TIMER					20
#define INVALID_PWM_PIN					21
#define INVALID_TIMER_TOP_VALUE			22

#define I2C_INVALID_SLAVE_ADDRESS		30
#define I2C_SLA_NOK_RESPONSE			31
#define I2C_DATA_NOK_RESPONSE			32
#define I2C_UNKNOWN_ERROR				33

#define USART_TX_BUSY					40
#define USART_RX_BUFFER_FULL			41
#define USART_TX_BUFFER_FULL			42
//********************

#define AHI								11		// HIP4081 input pins
#define ALI								12		// HIP4081 input pins
#define BHI								15		// HIP4081 input pins
#define BLI								16		// HIP4081 input pins
////////////////////////////// END DEFINITIONS ////////////////////////////////////

////////////////////////////// INCLUDES ///////////////////////////////////////////
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
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

volatile uint8_t RXData[USARTBUFFER];			// Array holding USART received data
volatile uint8_t RXBufferOutPos			= 0;	// Position for the data going OUT the buffer
volatile uint8_t RXBufferAmount			= 0;	// Number of elements in the buffer
volatile uint8_t TXData[USARTBUFFER];			// Array holding USART received data
volatile uint8_t TXBufferOutPos			= 0;	// Position for the data going OUT the buffer
volatile uint8_t TXBufferAmount			= 0;	// Number of elements in the buffer

volatile uint8_t I2CData				= 0;	// Holds I2C data (either transmit or receive)
volatile uint8_t I2CRemoteAddress		= 0;	// I2C address of the slave to contact
uint8_t I2CDataSize						= 0;	// Size of the packet to transmit

volatile uint32_t pulsesEncCHA			= 0;	// Pulse counter for encoder channel A (updated by ISR)
volatile uint32_t pulsesEncCHB			= 0;	// Pulse counter for encoder channel B (updated by ISR)
volatile uint8_t lastInterruptChannel	= 0;	// Which encoder channel triggered the last external interrupt (updated by ISR)
uint16_t deltaTimeCHA					= 0;	// Time between encoder Channel A pulses
uint16_t deltaTimeCHB					= 0;	// Time between encoder Channel A pulses
uint16_t HZCHA							= 0;	// Encoder channel A frequency
uint16_t HZCHB							= 0;	// Encoder channel A frequency

volatile uint32_t millisSeconds			= 0;	// Elapsed milliseconds since boot (updated by ISR)
volatile uint32_t microsSeconds			= 0;	// Elapsed microseconds since boot (updated by ISR)
volatile uint16_t micros1000s			= 0;	// Rolling counter for 1000s of microseconds (updated by ISR)
volatile uint16_t inCaptTimeCHA			= 0;	// Input capture timestamp for encoder channel A (updated by ISR)
volatile uint16_t inCaptTimeCHB			= 0;	// Input capture timestamp for encoder channel B (updated by ISR)
volatile uint16_t prevCaptTimeCHA		= 0;	// Previous Input capture timestamp for encoder channel A (updated by ISR)
volatile uint16_t prevCaptTimeCHB		= 0;	// Previous Input capture timestamp for encoder channel B (updated by ISR)

uint16_t prescalerPWM					= 0;	// Input Capture time unit in nanoseconds
uint16_t prescalerTimer					= 0;	// Input Capture time unit in nanoseconds
uint16_t prescalerInputCapture			= 0;	// Input Capture time unit in nanoseconds
uint16_t inCaptTimeUnit					= 0;	// Time unit in microseconds

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

void initPWM(uint8_t timer, uint16_t maxFrequency);
void setPWM(uint8_t pinPWM, uint8_t valuePWM);

void initErrorPin(uint8_t errorPin);
void setError(uint8_t code);
void clearError();

void initTimer();
void initInputCapture( uint16_t prescaler, uint8_t edge);
uint16_t calculateFrequency(uint16_t deltaTime);

void initUSART(uint16_t baudrate);
void transmitUSART(uint8_t data);

void initI2C();
void I2CTransmit(uint8_t slaveAddress, uint8_t READorWRITE, uint8_t data);
////////////////////////////// END FUNCTIONS DECLARATION //////////////////////////

int main (void){
////////////////////////////// SETUP //////////////////////////////////////////////
	//********** Pins configuration **********
	DDRB	= 0b00000110;						// Set pin direction (1=OUTPUT, 0=INPUT)
	DDRC	= 0b11111111;						//
	DDRD	= 0b11111111;						//
	
	PORTB	= 0b00000001;						// Set pin state :
	PORTC	= 0b00000000;						// if OUTPUT: 1= HIGH, 0= LOWs
	PORTD	= 0b00000000;						// if INPUT: 1= Pull-up on, 0= Pull-up off
	//********************

	//********** Services Initialization **********
	initErrorPin(errorPin);						// Mandatory services
	initTimer();								//
	
	initADC();
	initPWM(TIMER0, SUPERSLOW);					// Initialize PWM Timer with frequency limiter (Hz)
	initInputCapture(SUPERSLOW, RISING);
	//initI2C();
	initUSART(9600);

	sei();										// Enable Global Interrupt
	//********************
////////////////////////////// END SETUP /////////////////////////////////////////
	
	while(1) {
////////////////////////////// MAIN LOOP /////////////////////////////////////////		
		//********** ISR flags checks **********
		if(ISR_TMR1CAPT){
			switch (lastInterruptChannel){
				case 0:
					if (prevCaptTimeCHA>inCaptTimeCHA){							// Prevents error when delteTime counter overflows
						deltaTimeCHA = (65536-prevCaptTimeCHA)+inCaptTimeCHA;	// 65536 = MAX counter value
					}else{
						deltaTimeCHA = inCaptTimeCHA-prevCaptTimeCHA;
					}
					HZCHA = calculateFrequency(deltaTimeCHA);
				break;
				case 1:
					if (prevCaptTimeCHB>inCaptTimeCHB){
						deltaTimeCHB = (65536-prevCaptTimeCHB)+inCaptTimeCHB;
					}else{
						deltaTimeCHB = inCaptTimeCHB-prevCaptTimeCHB;
					}
					HZCHB = calculateFrequency(deltaTimeCHB);
				break;
			}
			ISR_TMR1CAPT = 0;
		}
		if(ISR_ADC){
			result_ADC = ADC;							// Transfer ADC result in holder variable
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
					setError(INVALID_ADC_CHANNEL);
				break;
			}
			ISR_ADC = 0;
		}
		if(ISR_USARTRX){
			if(RXBufferAmount<USARTBUFFER){
				if(RXBufferOutPos < USARTBUFFER){
					RXBufferOutPos ++;
				}else{
					RXBufferOutPos = 1;
				}
				// Do things with received data: RXData[RXBufferOutPos-1];
				//PORTC = RXData[RXBufferOutPos-1];
				transmitUSART(1234);
				RXBufferAmount --;
			}else{
				setError(USART_RX_BUFFER_FULL);
			}
		}
		if(ISR_USARTTX){
			if(TXBufferAmount>0){
				transmitUSART(TXData[TXBufferOutPos]);
				if(RXBufferOutPos < USARTBUFFER){
					RXBufferOutPos ++;
					}else{
					RXBufferOutPos = 1;
				}
			}else{
				setError(USART_RX_BUFFER_FULL);
			}
		}
		//********************
		
		startADC(0);
		//setPWM(12, result_ADC>>2);
		//I2CTransmit(128, READ, 255);
		//transmitUSART(result_ADC>>2);
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
		PORTB |= (option<<bitOrder);				// Pin High/Low (output) or Pull-up enabled/disabled (input)
	}
	if (pin>=23 && pin<= 28){						// Determine BIT order and PORT based on Pin number (Pin 25 is PORTC, Bit 2)
		bitOrder = pin-23;
		DDRC |= (direction<<bitOrder);				// Set Pin as 1=output; 0=input
		PORTC |= (option<<bitOrder);				// Pin High/Low (output) or Pull-up enabled/disabled (input)
	}
}

void initADC(){
	ISR_ADC = 0;									// Reset to allow new conversion after initialization
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);	// Enable ADC, Enable Interrupt, Set prescaler to 128
}
void startADC(uint8_t channel){
	if(ISR_ADC){
		setError(ADC_PREVIOUS_NOT_FINISHED);
	}else{
		if (channel<8){
			setPin(23+channel, INPUT, NO_PULLUP);
			ADMUX = (1 << REFS0) | (channel << MUX0);	// Set voltage reference (VCC), set ADC channel
			ADCSRA |= (1 << ADSC);						// Start conversion
			channelADCinUse = channel;
		}else{
			setError(INVALID_ADC_CHANNEL);
		}
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

void initPWM(uint8_t timer, uint16_t prescaler){
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
			setError(INVALID_TIMER);
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
			setError(INVALID_PWM_PIN);
		break;
	}
}

void initTimer(uint16_t prescaler){
	prescalerTimer = prescaler;
	TCCR2A = (1<<WGM21);								// Timer1 CTC mode OCR2A as MAX value (WGMx)
	switch (prescaler){
		case SUPERFAST:
			TCCR2B = (0<<CS22) | (0<<CS21) | (1<<CS20);	// Set prescaler on clock (no prescaler)
		break;
		case FAST:
			TCCR2B = (0<<CS22) | (1<<CS21) | (0<<CS20);	// Set prescaler on clock (CLKio/8)
		break;
		case 32:
			TCCR2B = (0<<CS22) | (1<<CS21) | (1<<CS20);	// Set prescaler on clock (CLKio/32)
		break;
		case MEDIUM:
			TCCR2B = (1<<CS22) | (0<<CS21) | (0<<CS20);	// Set prescaler on clock (CLKio/64)
		break;
		case 128:
			TCCR2B = (1<<CS22) | (0<<CS21) | (1<<CS20);	// Set prescaler on clock (CLKio/128)
		break;
		case SLOW:
			TCCR2B = (1<<CS22) | (1<<CS21) | (0<<CS20);	// Set prescaler on clock (CLKio/256)
		break;
		case SUPERSLOW:
			TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);	// Set prescaler on clock (CLKio/1024)
		break;
		default:
			TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);	// Default to SUPERSLOW
		break;
	}
	TCCR2B = (1<<CS21);									// 8 prescaler (CTC)
	uint8_t CTCTopValue = (F_CPU/(2*prescalerTimer*(1000000/(TIMEUNIT*2))))-2;	// (F_CPU/2*prescaler*Frequency)-2. -2 instaed of -1 to compensate for couter roll-over (back to 0).
	if (CTCTopValue>255){								// 8 bits timer, TOP value cannot exceed 255
		setError(INVALID_TIMER_TOP_VALUE);
	}else{
		OCR2A = CTCTopValue;							// Set OCR2A to calculated TOP value			
		TIMSK2 = (1<<OCIE2A);							// Enable interrupt when counter reaches ICR1
		TCNT2 = 0;										// Timer1 counter reset
	}
}
void initInputCapture(uint16_t prescaler, uint8_t edge){
	prescalerInputCapture = prescaler;
	inCaptTimeUnit = 1000000/(F_CPU/prescalerInputCapture);
	TCCR1B = (1<<ICNC1) | (edge<<ICES1);				// Input capture noise canceler enabled, detect on selected signal edge
	switch (prescaler){
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
		TCCR1B |= (1<<CS12) | (0<<CS11) | (1<<CS10);	// Set prescaler on clock (CLKio/256)
		break;
		case SUPERSLOW:
		TCCR1B |= (01<CS12) | (0<<CS11) | (1<<CS10);	// Set prescaler on clock (CLKio/1024)
		break;
		default:
		TCCR1B |= (0<<CS12) | (0<<CS11) | (1<<CS10);	// Default to SUPERSLOW
		break;
	}
	TIMSK1 = (1<<ICIE1);
}
uint16_t calculateFrequency(uint16_t deltaTime){
	uint16_t result = 0;
	result = 1000000/(deltaTime*inCaptTimeUnit);
	return result;
}

void initI2C(){
	TWBR = 18;									// Determines I2C frequency
	TWCR |= (1<<TWEN) | (1<<TWIE);				// Enables I2C and I2C interrupts
	TWSR |= (1<<TWPS0);							// I2C prescaler
	TWAR = 2;									// Slave address
	TWDR = 0;									// Reset I2C Data register
}
void I2CTransmit(uint8_t slaveAddress, uint8_t READorWRITE, uint8_t data){
	if(slaveAddress>128){
		setError(I2C_INVALID_SLAVE_ADDRESS);
	}else{
		I2CRemoteAddress = (slaveAddress<<1) & (READorWRITE | 0b11111110);
		TWDR = I2CRemoteAddress;
		TWCR |= (1<<TWSTA) | (0<<TWSTO) | (1<<TWINT);
	}
}

void initUSART(uint16_t baudrate){
	UBRR0 = UBRR;
	UCSR0B = (1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}
void transmitUSART(uint8_t data){
	if(TXBufferAmount>=USARTBUFFER){
		setError(USART_TX_BUFFER_FULL);
	}else{
		if((TXBufferOutPos+TXBufferAmount) < USARTBUFFER){
			TXData[TXBufferOutPos+TXBufferAmount] = UDR0;	// Transfer register value to RX Buffer
		}else{
			TXData[(TXBufferOutPos+TXBufferAmount)-USARTBUFFER] = UDR0;		// Transfer register value to RX Buffer
		}
		TXBufferAmount ++;
	}
	if(ISR_USARTTX){
		setError(USART_TX_BUSY);
	}else{
		ISR_USARTTX = 1;
		UDR0 = data;
		TXBufferAmount --;
		if(TXBufferOutPos < USARTBUFFER){
			TXBufferOutPos ++;
		}else{
			TXBufferOutPos = 1;
		}
	}
}
uint8_t digitsInInt(uint32_t number){
	uint8_t result = 0;
	if(number<10){
		result = 1;
	} else if(number<100){
		result = 2;
	} else if(number<1000){
		result = 3;
	} else if(number<10000){
		result = 4;
	} else if(number<100000){
		result = 5;
	} else if(number<1000000){
		result = 6;
	} else if(number<10000000){
		result = 7;
	} else if(number<100000000){
		result = 8;
	} else if(number<1000000000){
		result = 9;
	} 
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
	switch (lastInterruptChannel){				// Check which encoder channel sent triggered the Input Capture
		case 0:
			prevCaptTimeCHA = inCaptTimeCHA;	// Move current timestamp to the previous one
			inCaptTimeCHA = ICR1;				// Update current timestamp
		break;
		case 1:
			prevCaptTimeCHB = inCaptTimeCHB;	// Move current timestamp to the previous one
			inCaptTimeCHB = ICR1;				// Update current timestamp
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
	ISR_TMR1OVF = 1;
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
	ISR_USARTRX = 1;							//
}
ISR(USART_UDRE_vect){							// USART Data Register Empty
	ISR_USARTUDRE = 1;							//
}
ISR(USART_TX_vect){								// USART TX Complete
	ISR_USARTTX = 0;							//
}
ISR(ADC_vect){									// ADC Conversion Complete
	ISR_ADC = 1;								// 
}
ISR(EE_READY_vect){								// EEPROM Ready
	ISR_EEREADY = 0;
}
ISR(ANALOG_COMP_vect){							// Analog Comparator
	ISR_ANALOGCOMP = 0;
}
ISR(TWI_vect){									// 2-wire Serial Interface
	ISR_TWI = 0;
	switch(TWSR & 0b11111100){					// Masking prescaler bits in TWSR
		case 8:									// A START condition has been transmitted
			TWDR = I2CRemoteAddress;
			TWCR |= (0<<TWSTA) | (0<<TWSTO) | (1<<TWINT);
		break;
		case 10:								// A repeated START condition has been transmitted
		break;
		case 18:								// SLA+W has been transmitted; ACK has been received
			TWDR = I2CData;
			TWCR |= (0<<TWSTA) | (0<<TWSTO) | (1<<TWINT);
		break;
		case 20:								// SLA+W has been transmitted; NOT ACK has been received
			setError(I2C_SLA_NOK_RESPONSE);
		break;
		case 28:								// Data byte has been transmitted; ACK has been received
			TWCR |= (0<<TWSTA) | (1<<TWSTO) | (1<<TWINT);
		break;
		case 30:								// Data byte has been transmitted; NOT ACK has been received
			setError(I2C_DATA_NOK_RESPONSE);
		break;
		case 38:								// Arbitration lost in SLA+W or data bytes
		break;
		default:
			setError(I2C_UNKNOWN_ERROR);
		break;		
	}
}
ISR(SPM_READY_vect){							// Store Program Memory Ready
	ISR_SPM = 0;
}
////////////////////////////// END ISRs //////////////////////////