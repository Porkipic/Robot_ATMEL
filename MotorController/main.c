////////////////////////////// DEFINITIONS ////////////////////////////////////////
#define F_CPU							16000000UL	// Define CPU speed

#define OUTPUT							1			// Used for setPin()
#define INPUT							0			// Used for setPin()
#define HIGH							1			// Used for setPin()
#define LOW								0			// Used for setPin()
#define PULLUP							1			// Used for setPin()
#define NO_PULLUP						0			// Used for setPin()

#define CURRENT_ID						0			// Used for initialization(). IDs 0 to 5 reserved for ADC (channel set based on ID)
#define TEMP_HB_ID						1			// Used for initialization(). IDs 0 to 5 reserved for ADC (channel set based on ID)
#define TEMP_MOTOR_ID					2			// Used for initialization(). IDs 0 to 5 reserved for ADC (channel set based on ID)
#define PID_ID							7			// Used for initialization()
#define ENCODER_ID						8			// Used for initialization()
#define HB_DRIVER_ID					9			// Used for initialization()
#define FAN_ID							10			// Used for initialization()
#define ERROR_ID						11			// Used for initialization()
#define TIMER_ID						12			// Used for initialization()
#define DRIVETRAIN_ID					13			// Used for initialization()
#define START_ADC_ID					252			// Used for startADC()
#define ADC_PRESCALER_ID				253			// Used for calculateADCPrescaler()
#define INITIALIZATION_ID				254			// Used for initialization()
#define ALL								255			// Used for initialization(). ID 255 reserved for initialization of ALL structures/services

#define ERROR_INITIALIZATION			0			// initialzation()			Error during global initalization()
#define ERROR_ADC_PRESCALER				1			// calculateADCPrescaler()	No ADC prescaler within sampling speed (50kHz to 200kHz)
#define ERROR_ADC_BUSY					2			// startADC()				ADC started beore previous result has been read
#define ERROR_INVALID_ADC_CHANNEL		3			// startADC()				ADC channel is invalid (channel can be betzeen 0 and 5)

#define RESET							1			// RESET pin
#define CAN_INTERRUPT					6			// MCP2515 CAN Interrupt
#define CHIP_SELECT						15			// SPI Chip Select
#define SS								16			// SPI SS
#define MOSI							17			// SPI MOSI
#define MISO							18			// SPI MISO
#define SCK								19			// SPI SCK
////////////////////////////// END DEFINITIONS ////////////////////////////////////

////////////////////////////// INCLUDES ///////////////////////////////////////////
#include <avr/io.h>
#include <avr/interrupt.h>
////////////////////////////// END INCLUDES ///////////////////////////////////////

////////////////////////////// ISR FLAGS //////////////////////////////////////////
volatile uint8_t ISR_INT0				= 0;		// External Interrupt 0
volatile uint8_t ISR_INT1				= 0;		// External Interrupt 1
volatile uint8_t ISR_PCINT0				= 0;		// Pin Change Interrupt Request 0
volatile uint8_t ISR_PCINT1				= 0;		// Pin Change Interrupt Request 1
volatile uint8_t ISR_PCINT2				= 0;		// Pin Change Interrupt Request 2
volatile uint8_t ISR_WDT				= 0;		// Watchdog Time-out Interrupt
volatile uint8_t ISR_TMR2CA				= 0;		// Timer/Counter 2 Compare Match A
volatile uint8_t ISR_TMR2CB				= 0;		// Timer/Counter 2 Compare Match B
volatile uint8_t ISR_TMR2OVF			= 0;		// Timer/Counter 2 Overflow
volatile uint8_t ISR_TMR1CAPT			= 0;		// Timer/Counter 1 Capture Event
volatile uint8_t ISR_TMR1CA				= 0;		// Timer/Counter 1 Compare Match A
volatile uint8_t ISR_TMR1CB				= 0;		// Timer/Counter 1 Compare Match B
volatile uint8_t ISR_TMR1OVF			= 0;		// Timer/Counter 1 Overflow
volatile uint8_t ISR_TMR0CA				= 0;		// Timer/Counter 0 Compare Match A
volatile uint8_t ISR_TMR0CB				= 0;		// Timer/Counter 0 Compare Match B
volatile uint8_t ISR_TMR0OVF			= 0;		// Timer/Counter 0 Overflow
volatile uint8_t ISR_SPI				= 0;		// SPI Serial Transfer Complete
volatile uint8_t ISR_USARTRX			= 0;		// USART RX Complete
volatile uint8_t ISR_USARTUDRE			= 1;		// USART Data Register Empty
volatile uint8_t ISR_USARTTX			= 1;		// USART TX Complete
volatile uint8_t ISR_ADC				= 0;		// ADC Conversion Complete
volatile uint8_t ISR_EEREADY			= 0;		// EEPROM Ready
volatile uint8_t ISR_ANALOGCOMP			= 0;		// Analog Comparator
volatile uint8_t ISR_TWI				= 0;		// 2-wire Serial Interface
volatile uint8_t ISR_SPM				= 0;		// Store Program Memory Ready
////////////////////////////// END ISR FLAGS //////////////////////////////////////

////////////////////////////// STRUCTURES /////////////////////////////////////////
struct Timer{
	uint8_t id;
	uint8_t timeunit;
	uint8_t prescaler;
	uint8_t hours;
	uint8_t minutes;
	uint16_t seconds;
	uint16_t milliseconds;
	uint16_t microseconds;
};
struct Timer timer;
struct Timer *timerPTR = &timer;

struct ADConverter{
	uint8_t id;
	uint8_t prescaler;
	uint8_t channel;
	uint16_t result;
	uint8_t pin;
};
struct ADConverter current;
struct ADConverter *currentPTR = &current;
struct ADConverter tempHBridge;
struct ADConverter *tempHBridgePTR = &tempHBridge;
struct ADConverter tempMotor;
struct ADConverter *tempMotorPTR = &tempMotor;

struct Timestamp {
	uint8_t id;
	uint16_t milliseconds;
	uint16_t microseconds;	
};
struct Timestamp timestamp;
struct Timestamp *timestampPTR = &timestamp;

struct Drivetrain{
	uint8_t id;
	uint8_t wheelCircumference;						// Wheel circumference in cm
	uint8_t gearRatio;								// Gear ratio
	uint8_t LinearRatio;
};
struct Drivetrain drivetrain;
struct Drivetrain *drivetrainPTR = &drivetrain;

struct PID{
	uint8_t id;
	uint8_t P;
	uint8_t I;
	uint8_t D;
	uint8_t attenuator;
};
struct PID pid;
struct PID *pidPTR = &pid;

struct Encoder{
	uint8_t id;
	uint8_t ppm;
	uint16_t requestedFrequency;
	uint16_t measuredFrequency;
	uint8_t requestedDirection;
	uint8_t measuredDirection;
	uint8_t CHAPin;
	uint8_t CHBPin;
	volatile uint16_t CHAPulseCounter;
	volatile uint16_t CHBPulseCounter;
	struct Timestamp CHAPreviousTime;
	struct Timestamp CHBPreviousTime;
};
struct Encoder encoder;
struct Encoder *encoderPTR = &encoder;

struct HBRidgeDriver{
	uint8_t id;
	uint8_t AHIPin;
	uint8_t BHIPin;
	uint8_t ALIPin;
	uint8_t BLIPin;
	uint8_t DISPin;
};
struct HBRidgeDriver hbDriver;
struct HBRidgeDriver *hbDriverPTR = &hbDriver;

struct Fan{
	uint8_t id;
	uint8_t tempON;
	uint8_t tempOFF;
	uint8_t pin;
};
struct Fan fan;
struct Fan *fanPTR = &fan;

struct Error{
	uint8_t id;
	uint8_t code;
	uint8_t sourceID;
	uint8_t pin;
};
struct Error errorSignal;
struct Error *errorSignalPTR = &errorSignal;
////////////////////////////// END STRUCTURES /////////////////////////////////////

////////////////////////////// CONSTANTS DECLARATION //////////////////////////////

////////////////////////////// END CONSTANTS DECLARATION //////////////////////////

////////////////////////////// VARIABLES DECLARATION //////////////////////////////
struct ADConverter *channelInUsePTR = NULL;
////////////////////////////// END VARIABLES DECLARATION //////////////////////////

////////////////////////////// FUNCTIONS DECLARATION //////////////////////////////
void initialization(uint8_t id);
void timerINIT(struct Timer *pointer, uint8_t id);
void ADConverterINIT(struct ADConverter *pointer, uint8_t channel);
void drivetrainINIT (struct Drivetrain *pointer, uint8_t id);
void pidINIT (struct PID *pointer, uint8_t id);
void encoderINIT (struct Encoder *pointer, uint8_t id);
void hbDriverINIT (struct HBRidgeDriver *pointer, uint8_t id);
void fanINIT (struct Fan *pointer, uint8_t id);
void errorINIT (struct Error *pointer, uint8_t id);

void setPin(uint8_t pin, uint8_t direction, uint8_t option);

void setError(struct Error *pointer, uint8_t code, uint8_t sourceID);
void clearError(struct Error *pointer);

void calculateFrequency(struct Encoder *pointer, struct Timer *timer);
uint8_t speedFromFrequency(struct Encoder *encoder, struct Drivetrain *drivetrain);
uint8_t calculateADCPrescaler(struct ADConverter *ADConverter);
////////////////////////////// END FUNCTIONS DECLARATION //////////////////////////

int main(void){

////////////////////////////// SETUP //////////////////////////////////////////////
	
	//********** Structures Initialization **********
	initialization(ALL);
	//********************
	
	//********** Pins configuration **********
	setPin(RESET, INPUT, PULLUP);
	setPin(CHIP_SELECT, OUTPUT, LOW);
	setPin(SS, INPUT, NO_PULLUP);
	setPin(MOSI, OUTPUT, LOW);
	setPin(MISO, INPUT, NO_PULLUP);
	setPin(SCK, OUTPUT, LOW);
	setPin(CAN_INTERRUPT, INPUT, NO_PULLUP);
	//********************

	//********** Services Initialization **********
	//initTimer();									// Mandatory service
	//********************
	
	//********** Global interrupts **********
	sei();											// Enable Global Interrupts
	//********************
////////////////////////////// END SETUP //////////////////////////////////////////
	
	while(1){
		startADC(currentPTR);
		calculateFrequency(encoderPTR, timerPTR);
		//********** ISR flags polling **********
		pollADC(channelInUsePTR);
		//********************
		
	}
}

////////////////////////////// FUNCTIONS DEFINITIONS //////////////////////////////
void initialization(uint8_t id){
	switch (id){
		case CURRENT_ID:
			ADConverterINIT(currentPTR, CURRENT_ID);
			break;
		case TEMP_HB_ID:
			ADConverterINIT(tempHBridgePTR, TEMP_HB_ID);
			break;
		case TEMP_MOTOR_ID:
			ADConverterINIT(tempMotorPTR, TEMP_MOTOR_ID);
			break;
		case TIMER_ID:
			timerINIT(timerPTR, TIMER_ID);
			break;
		case DRIVETRAIN_ID:
			drivetrainINIT(drivetrainPTR, DRIVETRAIN_ID);
			break;
		case PID_ID:
			pidINIT(pidPTR, PID_ID);
			break;
		case ENCODER_ID:
			encoderINIT(encoderPTR, ENCODER_ID);
			break;
		case HB_DRIVER_ID:
			hbDriverINIT(hbDriverPTR, HB_DRIVER_ID);
			break;
		case FAN_ID:
			fanINIT(fanPTR, FAN_ID);
			break;
		case ERROR_ID:
			errorINIT(errorSignalPTR, ERROR_ID);
			break;
		case ALL:
			ADConverterINIT(currentPTR, CURRENT_ID);
			ADConverterINIT(tempHBridgePTR, TEMP_HB_ID);
			ADConverterINIT(tempMotorPTR, TEMP_MOTOR_ID);
			timerINIT(timerPTR, TIMER_ID);
			drivetrainINIT(drivetrainPTR, DRIVETRAIN_ID);
			pidINIT(pidPTR, PID_ID);
			encoderINIT(encoderPTR, ENCODER_ID);
			hbDriverINIT(hbDriverPTR, HB_DRIVER_ID);
			fanINIT(fanPTR, FAN_ID);
			errorINIT(errorSignalPTR, ERROR_ID);
			break;
		default:
			errorINIT(errorSignalPTR, ERROR_ID);
			setError(errorSignalPTR,ERROR_INITIALIZATION, INITIALIZATION_ID);
			break;
	}	
}
void timerINIT(struct Timer *pointer, uint8_t id){
	pointer->id = id;
	pointer->hours = 0;
	pointer->microseconds = 0;
	pointer->milliseconds = 0;
	pointer->minutes = 0;
	pointer->prescaler = 1;
	pointer->seconds = 0;
	pointer->timeunit = F_CPU/timerPTR->prescaler;
}
void ADConverterINIT(struct ADConverter *pointer, uint8_t id){
	pointer->id = id;
	pointer->channel = id;
	pointer->prescaler = 0;
	pointer->result = 0;
	pointer->pin = id+23;
	setPin(pointer->pin, INPUT, NO_PULLUP);
	ISR_ADC = 0;											// Reset to allow new conversion after initialization
	ADCSRA |= (1 << ADEN);									// Enable ADC
	ADCSRA |= (0 << ADSC);									// No start conversion
	ADCSRA |= (0 << ADATE);									// No auto-trigger
	ADCSRA |= (1 << ADIF);									// Clear ADC interrupt
	ADCSRA |= (1 << ADIE);									// Enable ADC interrupts
	ADCSRA |= (calculateADCPrescaler(pointer) << ADPS0);			// Set ADC prescaler (ADC sampling speed between 50kHz and 200kHz)
}
void drivetrainINIT (struct Drivetrain *pointer, uint8_t id){
	pointer->id = id;
	pointer->wheelCircumference = 20;
	pointer->gearRatio = 1;
	pointer->LinearRatio = drivetrainPTR->wheelCircumference/drivetrainPTR->gearRatio;
}
void pidINIT (struct PID *pointer, uint8_t id){
	pointer->id = id;
	pointer->P = 1;
	pointer->I = 0;
	pointer->D = 0;
	pointer->attenuator = 1;
}
void encoderINIT (struct Encoder *pointer, uint8_t id){
	pointer->id = id;
	pointer->measuredFrequency = 0;
	pointer->requestedFrequency = 0;
	pointer->CHAPin = 4;
	pointer->CHBPin = 5;
	setPin(pointer->CHAPin, INPUT, PULLUP);
	setPin(pointer->CHBPin, INPUT, PULLUP);
}
void hbDriverINIT (struct HBRidgeDriver *pointer, uint8_t id){
	pointer->id = id;
	pointer->AHIPin = 11;
	pointer->ALIPin = 2;
	pointer->BHIPin = 12;
	pointer->BLIPin = 3;
	pointer->DISPin = 13;
	setPin(pointer->AHIPin, OUTPUT, LOW);
	setPin(pointer->ALIPin, OUTPUT, LOW);
	setPin(pointer->BHIPin, OUTPUT, LOW);
	setPin(pointer->BLIPin, OUTPUT, LOW);
	setPin(pointer->DISPin, OUTPUT, LOW);
}
void fanINIT (struct Fan *pointer, uint8_t id){
	pointer->id = id;
	pointer->pin = 14;
	pointer->tempOFF = 30;
	pointer->tempON = 50;
	setPin(pointer->pin, OUTPUT, LOW);
}
void errorINIT (struct Error *pointer, uint8_t id){
	pointer->id = id;
	pointer->code = 0;
	pointer->pin = 26;
	pointer->sourceID = 0;
	setPin(pointer->pin, OUTPUT, LOW);
}

void setPin(uint8_t pin, uint8_t direction, uint8_t option){
	if (pin>=2 && pin<= 13){						// Determine BIT order and PORT based on Pin number (Pin 6 is PORTD, BIT 4)
		DDRD |= (direction<<(pin-2));				// Set Pin as 1=output; 0=input
		PORTD |= (option<<(pin-2));					// Pin High/Low (output) or Pull-up enabled/disabled (input)
	}
	if (pin>=14 && pin<= 19){						// Determine BIT order and PORT based on Pin number (Pin 19 is PORTB, BIT 5)
		DDRB |= (direction<<(pin-14));				// Set Pin as 1=output; 0=input
		PORTB |= (option<<(pin-14));				// Pin High/Low (output) or Pull-up enabled/disabled (input)
	}
	if (pin>=23 && pin<= 28){						// Determine BIT order and PORT based on Pin number (Pin 25 is PORTC, BIT 2)
		DDRC |= (direction<<(pin-23));				// Set Pin as 1=output; 0=input
		PORTC |= (option<<(pin-23));				// Pin High/Low (output) or Pull-up enabled/disabled (input)
	}
}

void setError(struct Error *pointer, uint8_t code, uint8_t sourceID){
	setPin(pointer->pin, OUTPUT, HIGH);		// Turn Error pin ON
	pointer->code = code;					// Set error code
	pointer->sourceID = sourceID;			// Set error source ID
}
void clearError(struct Error *pointer){
	setPin(pointer->pin, OUTPUT, LOW);
	pointer->code = 0;
	pointer->sourceID = 0;
}

void calculateFrequency(struct Encoder *encoder, struct Timer *timer){
	struct Timestamp deltaTime;																								// Initialize deltaTime temporary variable
	uint16_t CHAFrequency = 0;																								// Initialize Channel A frequency variable
	uint16_t CHBFrequency = 0;																								// Initialize Channel B frequency variable
	
	deltaTime.milliseconds = ((timer->milliseconds)-(encoder->CHAPreviousTime.milliseconds))/encoder->CHAPulseCounter;		// Set time per pulse (milliseconds)
	encoder->CHAPreviousTime.milliseconds = timer->milliseconds;															// Update previous time with current time (milliseconds)
	deltaTime.microseconds = ((timer->microseconds)-(encoder->CHAPreviousTime.microseconds))/encoder->CHAPulseCounter;		// Set time per pule (microseconds)
	encoder->CHAPreviousTime.milliseconds = timer->milliseconds;															// Update previous time with current time (microseconds)
	CHAFrequency= 1000000/((deltaTime.milliseconds*1000)+(deltaTime.microseconds));											// Calculate Channel A frequency
	
	deltaTime.milliseconds = ((timer->milliseconds)-(encoder->CHBPreviousTime.milliseconds))/encoder->CHBPulseCounter;		// Set time per pulse (milliseconds)
	encoder->CHBPreviousTime.milliseconds = timer->milliseconds;															// Update previous time with current time (milliseconds)
	deltaTime.microseconds = ((timer->microseconds)-(encoder->CHBPreviousTime.microseconds))/encoder->CHBPulseCounter;		// Set time per pule (microseconds)
	encoder->CHBPreviousTime.milliseconds = timer->milliseconds;															// Update previous time with current time (microseconds)
	CHBFrequency= 1000000/((deltaTime.milliseconds*1000)+(deltaTime.microseconds));											// Calculate Channel B frequency
	
	encoder->measuredFrequency = (CHAFrequency>>2)+(CHBFrequency>>2);															// Average CHA and CHB frequencies (Bitwise division by 2)
}
uint8_t speedFromFrequency(struct Encoder *encoder, struct Drivetrain *drivetrain){	// CHECK THE MATHS
	uint8_t speed = 0;
	speed = ((((encoder->ppm)/2)*(drivetrain->LinearRatio))*36)/100;
	return speed;
}
uint8_t calculateADCPrescaler(struct ADConverter *ADConverter){
	uint8_t frequencyDivider = 2;
	uint8_t samplingFrequency = 0;
	uint8_t prescaler = 0;
	while (samplingFrequency<50 || samplingFrequency>200){
		samplingFrequency = (F_CPU/1000)/frequencyDivider;
		frequencyDivider &= frequencyDivider<<2;
	}
	ADConverter->prescaler = frequencyDivider;
	switch(frequencyDivider){
		case 2:
			prescaler = 0b001;
			break;
		case 4:
			prescaler = 0b010;
			break;
		case 8:
			prescaler = 0b011;
			break;
		case 16:
			prescaler = 0b100;
			break;
		case 32:
			prescaler = 0b101;
			break;
		case 64:
			prescaler = 0b110;
			break;
		case 128:
			prescaler = 0b111;
			break;
		default:
			setError(errorSignalPTR, ERROR_ADC_PRESCALER, ADC_PRESCALER_ID);
	}
	return prescaler;
}
void startADC(struct ADConverter *ADConverter){
	if(ISR_ADC){
		setError(errorSignalPTR, ERROR_ADC_BUSY, START_ADC_ID);
	}else{
		if (ADConverter->channel<5){
			channelInUsePTR = *ADConverter;
			ADMUX |= (1<<REFS1) | (0<<REFS0);						// Set voltage reference to VCC
			ADMUX |= (1<<ADLAR);									// Result left-justified (8 MSB on ADCH and 2 LSB on ADCL)
			ADMUX |= (ADConverter->channel << MUX0);				// Set ADC channel as requested
			ADCSRA |= (1 << ADSC);									// Start conversion
		}else{
			setError(errorSignalPTR, ERROR_INVALID_ADC_CHANNEL, START_ADC_ID);
		}
	}
}
void pollADC(struct ADConverter *ADConverter){
	if(ISR_ADC){
		ADConverter->result = ADC>>6;
		*ADConverter = NULL;
		ISR_ADC = 0;
	}
}
////////////////////////////// END FUNCTIONS DEFINITIONS //////////////////////////

////////////////////////////// ISRs ///////////////////////////////////////////////
ISR(INT0_vect){										// External Interrupt Request 0
	encoderPTR->CHAPulseCounter++;
}
ISR(INT1_vect){										// External Interrupt Request 1
	encoderPTR->CHBPulseCounter++;
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
ISR(TIMER2_COMPA_vect){								// Timer/Counter 2 Compare Match A
	ISR_TMR2CA = 0;
}
ISR(TIMER2_COMPB_vect){								// Timer/Counter 2 Compare Match B
	ISR_TMR2CB = 0;
}
ISR(TIMER2_OVF_vect){								// Timer/Counter 2 Overflow
	ISR_TMR2OVF = 0;
}
ISR(TIMER1_CAPT_vect){								// Timer/Counter 1 Capture Event
	ISR_TMR1CAPT = 0;
}
ISR(TIMER1_COMPA_vect){								// Timer/Counter 1 Compare Match A
	ISR_TMR1CA = 0;
}
ISR(TIMER1_COMPB_vect){								// Timer/Counter 1 Compare Match B
	ISR_TMR1CB = 0;
}
ISR(TIMER1_OVF_vect){								// Timer/Counter 1 Overflow
	ISR_TMR1OVF = 0;
}
ISR(TIMER0_COMPA_vect){								// Timer/Counter 0 Compare Match A
	ISR_TMR0CA = 0;
}
ISR(TIMER0_COMPB_vect){								// Timer/Counter 0 Compare Match B
	ISR_TMR0CB = 0;
}
ISR(TIMER0_OVF_vect){								// Timer/Counter 0 Overflow
	ISR_TMR0OVF = 0;
}
ISR(SPI_STC_vect){									// SPI Serial Transfer Complete
	ISR_SPI = 0;
}
ISR(USART_RX_vect){									// USART RX Complete
	ISR_USARTRX = 0;
}
ISR(USART_UDRE_vect){								// USART Data Register Empty
	ISR_USARTUDRE = 0;
}
ISR(USART_TX_vect){									// USART TX Complete
	ISR_USARTTX = 0;
}
ISR(ADC_vect){										// ADC Conversion Complete
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
////////////////////////////// END ISRs ///////////////////////////////////////////