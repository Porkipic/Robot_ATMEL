////////////////////////////// DEFINITIONS ////////////////////////////////////////
#define F_CPU							16000000UL	// Define CPU speed

#define ADC_PRESCALER					0b111		// Define ADC prescaler value (50kHz < sampling frequency < 200kHz)

#define USART_BAUDRATE					19200		// USART baudrate
#define UBRR							((F_CPU/100)/(16*(USART_BAUDRATE/100)))-1
#define USART_BUFFER_SIZE				8

#define MSB_FIRST						0			// Used for SPI data order
#define LSB_FIRST						1			// Used for SPI data order
#define MASTER							1			// Used for SPI mode selection
#define SLAVE							0			// Used for SPI mode selection
#define SPI_Fosc2						2			// Used for SPI speed selection
#define SPI_Fosc4						4			// Used for SPI speed selection
#define SPI_Fosc8						8			// Used for SPI speed selection
#define SPI_Fosc16						16			// Used for SPI speed selection
#define SPI_Fosc32						32			// Used for SPI speed selection
#define SPI_Fosc64						64			// Used for SPI speed selection
#define SPI_Fosc128						128			// Used for SPI speed selection
#define SS								16			// SPI SS
#define MOSI							17			// SPI MOSI
#define MISO							18			// SPI MISO
#define SCK								19			// SPI SCK

#define OUTPUT							1			// Used for setPin()
#define INPUT							0			// Used for setPin()
#define HIGH							1			// Used for setPin()
#define LOW								0			// Used for setPin()
#define PULLUP							1			// Used for setPin()
#define NO_PULLUP						0			// Used for setPin()

//********** Structures IDs definitions **********
#define CURRENT_structID				0			// Used for initialization(). IDs 0 to 5 reserved for ADC (channel set based on ID)
#define TEMP_HBA_structID				1			// Used for initialization(). IDs 0 to 5 reserved for ADC (channel set based on ID)
#define TEMP_MOTOR_structID				2			// Used for initialization(). IDs 0 to 5 reserved for ADC (channel set based on ID)
#define TEMP_HBB_structID				3			// Used for initialization(). IDs 0 to 5 reserved for ADC (channel set based on ID)
#define USART_structID					6			// Used for initialization()
#define PID_structID					7			// Used for initialization()
#define ENCODER_structID				8			// Used for initialization()
#define HB_DRIVER_structID				9			// Used for initialization()
#define FAN_structID					10			// Used for initialization()
#define ERROR_structID					11			// Used for initialization()
#define TIMER_structID					12			// Used for initialization()
#define DRIVETRAIN_structID				13			// Used for initialization()
#define RX_RING_BUFFER_structID			14			// Used for initialization()
#define TX_RING_BUFFER_structID			15			// Used for initialization()
#define SPI_structID					16			// Used for initialization()
//********************

//********** Functions ID definitions **********
#define ALL								255			// Used for initialization(). ID 255 reserved for initialization of ALL structures/services
#define INITIALIZATION_funcID			254			// Used for initialization()
#define ADC_PRESCALER_funcID			253			// Used for calculateADCPrescaler()
#define START_ADC_funcID				252			// Used for adcStart()
#define USART_SEND_funcID				251			// Used for addTXRingBuffer()
#define USART_CYCLE_TX_funcID			250			// Used for cycleTXRingBuffer()
#define USART_CYCLE_RX_funcID			249			// Used for usartCycleRX()
#define USART_RECEIVE_funcID			248			// Used for usartReceive()
#define SPI_INIT_funcID					247			// Used for spiINIT()
//********************

//********** Error messages **********
#define ERROR_INITIALIZATION			0			// initialzation()			Error during initalization() for ALL structures
#define ERROR_ADC_BUSY					2			// adcStart()				ADC started before previous result has been read
#define ERROR_INVALID_ADC_CHANNEL		3			// adcStart()				ADC channel is invalid (channel can be betzeen 0 and 5)
#define ERROR_TX_BUFFER_FULL			4			// addTXRingBuffer()				USART TX buffer is full
#define ERROR_TX_BUSY					5			// cycleTXRingBuffer()			USART TX buffer is alreday in use
#define ERROR_RX_BUFFER_FULL			6			// cycleTXRingBuffer()			USART RX buffer is full
#define ERROR_RX_BUFFER_EMPTY			7			// usartReceive()			USART RX buffer is empty
#define ERROR_SPI_INVALID_MODE			8			// spiINIT()				SPI mode in invalide (See Master/Slave definitions)
#define ERROR_SPI_DATA_MODE				9			// spiINIT()				SPI data mode invalid (0, 1, 2, 3 available)
#define ERROR_SPI_SPEED					10			// spiINIT()				SPI speed invalid (See "SPI_Fosc" definitions)
#define ERROR_SPI_INVALID_DATA_ORDER	11			// spiINIT()				SPI data order invalid (See MSB/LSB definitions)
//********************

#define RESET							1			// RESET pin
#define CAN_INTERRUPT					6			// MCP2515 CAN Interrupt
#define CHIP_SELECT						15			// SPI Chip Select
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
	uint8_t channel;
	uint16_t result;
	uint8_t pin;
};
struct ADConverter current;
struct ADConverter *currentPTR = &current;
struct ADConverter tempHBridgeA;
struct ADConverter *tempHBridgeAPTR = &tempHBridgeA;
struct ADConverter tempHBridgeB;
struct ADConverter *tempHBridgeBPTR = &tempHBridgeB;
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

struct RingBuffer{
	uint8_t id;
	uint8_t data[USART_BUFFER_SIZE];				// Array holding USART data
	uint8_t inputPosition;							// Position for the data going IN the buffer
	uint8_t outputPosition;							// Position for the data going OUT the buffer
	volatile uint8_t dataAmount;					// Number of elements in the buffer	
};
struct RingBuffer rxBuffer;
struct RingBuffer txBuffer;

struct USART{
	uint8_t id;
	uint16_t baudrate;
	struct RingBuffer *rxBufferPTR;
	struct RingBuffer *txBufferPTR;
};
struct USART usart;
struct USART *usartPTR = &usart;

struct SPI{
	uint8_t id;
	uint8_t dataOrder;
	uint8_t spiMode;
	uint8_t dataMode;
	uint8_t speed;
	struct RingBuffer *rxBufferPTR;
	struct RingBuffer *txBufferPTR;
};
struct SPI spi;
struct SPI *spiPTR = &spi;
////////////////////////////// END STRUCTURES /////////////////////////////////////

////////////////////////////// CONSTANTS DECLARATION //////////////////////////////

////////////////////////////// END CONSTANTS DECLARATION //////////////////////////

////////////////////////////// VARIABLES DECLARATION //////////////////////////////
struct ADConverter *channelInUsePTR = 0;
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
void usartINIT(struct USART *pointer, uint8_t id);
void spiINIT(struct SPI *pointer, uint8_t dataOrder, uint8_t spiMode, uint8_t dataMode, uint8_t speed, uint8_t id);

void setPin(uint8_t pin, uint8_t direction, uint8_t option);

void setError(struct Error *pointer, uint8_t code, uint8_t sourceID);
void clearError(struct Error *pointer);

void calculateFrequency(struct Encoder *pointer, struct Timer *timer);
uint8_t speedFromFrequency(struct Encoder *encoder, struct Drivetrain *drivetrain);

void adcStart(struct ADConverter *adconverter);
void adcPoll(struct ADConverter *adconverter);

void addTXRingBuffer(struct USART *usart, uint8_t data);
void cycleTXRingBuffer(struct USART *usart);
uint8_t retrieveRXRingBuffer(struct USART *usart);
void cycleTXRingBuffer(struct USART *usart);
void cycleRXRingBuffer(struct USART *usart)
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
	//initTimer();									// Initialize time keeping timer
	//
	//********************
	
	//********** Global interrupts **********
	sei();											// Enable Global Interrupts
	//********************
////////////////////////////// END SETUP //////////////////////////////////////////
	
	while(1){
		adcStart(currentPTR);
		addTXRingBuffer(usartPTR->txBufferPTR, (channelInUsePTR->result)>>8);
		//calculateFrequency(encoderPTR, timerPTR);
		
		//********** ISR flags polling **********
		if(ISR_SPI){
			
		}
		if(ISR_USARTRX){
			cycleTXRingBuffer(usartPTR->txBufferPTR);	// Transmit data in TX buffer
		}
		if(ISR_USARTRX){
			cycleRXRingBuffer(usartPTR->rxBufferPTR);	// Store data in RX buffer
		}
		if(ISR_ADC){
			adcPoll(channelInUsePTR);					// Check if a new AD conversion is available
		}
		//********************
		
	}
}

////////////////////////////// FUNCTIONS DEFINITIONS //////////////////////////////
void initialization(uint8_t id){
	switch (id){
		case CURRENT_structID:
			ADConverterINIT(currentPTR, CURRENT_structID);
			break;
		case TEMP_HBA_structID:
			ADConverterINIT(tempHBridgeAPTR, TEMP_HBA_structID);
			break;
		case TEMP_HBB_structID:
			ADConverterINIT(tempHBridgeBPTR, TEMP_HBB_structID);
			break;
		case TEMP_MOTOR_structID:
			ADConverterINIT(tempMotorPTR, TEMP_MOTOR_structID);
			break;
		case TIMER_structID:
			timerINIT(timerPTR, TIMER_structID);
			break;
		case DRIVETRAIN_structID:
			drivetrainINIT(drivetrainPTR, DRIVETRAIN_structID);
			break;
		case PID_structID:
			pidINIT(pidPTR, PID_structID);
			break;
		case ENCODER_structID:
			encoderINIT(encoderPTR, ENCODER_structID);
			break;
		case HB_DRIVER_structID:
			hbDriverINIT(hbDriverPTR, HB_DRIVER_structID);
			break;
		case FAN_structID:
			fanINIT(fanPTR, FAN_structID);
			break;
		case ERROR_structID:
			errorINIT(errorSignalPTR, ERROR_structID);
			break;
		case USART_structID:
			usartINIT(usartPTR, USART_structID);
			break;
		case SPI_structID:
			spiINIT(spiPTR, MSB_FIRST, MASTER, 0, 128, SPI_structID);
			break;
		case ALL:
			ADConverterINIT(currentPTR, CURRENT_structID);
			ADConverterINIT(tempHBridgeAPTR, TEMP_HBA_structID);
			ADConverterINIT(tempHBridgeBPTR, TEMP_HBB_structID);
			ADConverterINIT(tempMotorPTR, TEMP_MOTOR_structID);
			timerINIT(timerPTR, TIMER_structID);
			drivetrainINIT(drivetrainPTR, DRIVETRAIN_structID);
			pidINIT(pidPTR, PID_structID);
			encoderINIT(encoderPTR, ENCODER_structID);
			hbDriverINIT(hbDriverPTR, HB_DRIVER_structID);
			fanINIT(fanPTR, FAN_structID);
			errorINIT(errorSignalPTR, ERROR_structID);
			usartINIT(usartPTR, USART_structID);
			spiINIT(spiPTR, MSB_FIRST, MASTER, 0, 128, SPI_structID);
			break;
		default:
			errorINIT(errorSignalPTR, ERROR_structID);
			setError(errorSignalPTR,ERROR_INITIALIZATION, INITIALIZATION_funcID);
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
	pointer->result = 0;
	pointer->pin = id+23;
	ISR_ADC = 0;											// Reset to allow new conversion after initialization
	ADCSRA |= (1 << ADEN);									// Enable ADC
	ADCSRA |= (0 << ADSC);									// No start conversion
	ADCSRA |= (0 << ADATE);									// No auto-trigger
	ADCSRA |= (1 << ADIF);									// Clear ADC interrupt
	ADCSRA |= (1 << ADIE);									// Enable ADC interrupts
	ADCSRA |= (ADC_PRESCALER << ADPS0);						// Set ADC prescaler (ADC sampling speed between 50kHz and 200kHz)
	setPin(pointer->pin, INPUT, NO_PULLUP);
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
void usartINIT(struct USART *pointer, uint8_t id){
	pointer->id = id;
	pointer->baudrate = USART_BAUDRATE;
	pointer->rxBufferPTR = &rxBuffer;
	pointer->txBufferPTR = &txBuffer;
	ringBufferINIT(pointer->rxBufferPTR, RX_RING_BUFFER_structID);
	ringBufferINIT(pointer->txBufferPTR, TX_RING_BUFFER_structID);
	UBRR0 = UBRR;									// Set baud rate according to compiler calculation
	UCSR0B = (1<<RXCIE0) | (1<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02);					// Enable USART interrupts
	UCSR0C = (0<<UMSEL01) | (0<<UMSEL01);
	UCSR0C |= (0<<UPM01) | (0<<UPM00);				// Asynchronous
	UCSR0C |= (1<<USBS0);							// No Parity
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);			// 8 Data bits
	setPin(2, INPUT, NO_PULLUP);					// Set RX pin as input
	setPin(3, OUTPUT, LOW);							// Set TX pin as output
}
void ringBufferINIT(struct RingBuffer *pointer, uint8_t id){
	pointer->id = RX_RING_BUFFER_structID;
	pointer->dataAmount = 0;
	pointer->inputPosition = 0;
	pointer->outputPosition = 0;
}
void spiINIT(struct SPI *pointer, uint8_t dataOrder, uint8_t spiMode, uint8_t dataMode, uint8_t speed, uint8_t id){
	pointer->id = id;
	switch(dataOrder){
		case 0:
			SPCR &= (dataOrder<<DORD);				// Data Order
			pointer->dataOrder = dataOrder;
			break;
		case 1:
			SPCR |= (dataOrder<<DORD);				// Data Order
			pointer->dataOrder = dataOrder;
			break;
		default:
			setError(errorSignalPTR, ERROR_SPI_INVALID_DATA_ORDER, SPI_INIT_funcID);
			break;
	}
	switch(spiMode){
		case 0:
			SPCR &= (spiMode<<MSTR);				// SPI Mode Slave
			pointer->spiMode = spiMode;
			setPin(MISO, OUTPUT, LOW);				// Set Output Low
			break;
		case 1:
			SPCR |= (spiMode<<MSTR);				// SPI Mode Master
			pointer->spiMode = spiMode;
			setPin(MOSI, OUTPUT, LOW);				// Set Output Low
			setPin(SCK, OUTPUT, LOW);				// Set Clock Low
			setPin(SS, INPUT, NO_PULLUP);			// Set SS as tri-state input. If pulled LOW, will disable Master Mode.
			break;
		default:
			setError(errorSignalPTR, ERROR_SPI_INVALID_MODE, SPI_INIT_funcID);
			break;
	}
	switch (dataMode){								// Set clock Polarity and Phase
		case 0:										//
			SPCR &= (0<<CPOL) & (0<<CPHA);			//
			pointer->dataMode = 0;					//
			break;									//
		case 1:										//
			SPCR &= (0<<CPOL) | (1<<CPHA);			//
			pointer->dataMode = 1;					//
			break;									//
		case 2:										//
			SPCR |= (1<<CPOL) & (0<<CPHA);			//
			pointer->dataMode = 2;					//
			break;									//
		case 3:										//
			SPCR |= (1<<CPOL) | (1<<CPHA);			//
			pointer->dataMode = 3;					//
			break;									//
		default:									//
			setError(errorSignalPTR, ERROR_SPI_DATA_MODE, SPI_INIT_funcID);
			break;									//
	}
	switch (speed){									// Set clock Polarity and Phase
		case 2:										//
			SPCR &= (0<<SPR1) & (0<<SPR0);			// F_CPU/2
			SPSR |= (1<<SPI2X);						// SPI double speed
			pointer->speed = speed;					//
		break;										//
		case 4:										//
			SPCR &= (0<<SPR1)  & (0<<SPR0);			// F_CPU/4
			SPSR &= (0<<SPI2X);						// No SPI double speed
			pointer->speed = speed;					//
		break;										//
		case 8:										//
			SPCR &= (0<<SPR1) | (1<<SPR0);			// F_CPU/8
			SPSR |= (1<<SPI2X);						// SPI double speed
			pointer->speed = speed;					//
		break;										//
		case 16:									//
			SPCR &= (0<<SPR1) | (1<<SPR0);			// F_CPU/16
			SPSR &= (0<<SPI2X);						// No SPI double speed
			pointer->speed = speed;					//
		break;										//
		case 32:									//
			SPCR |= (1<<SPR1) & (0<<SPR0);			// F_CPU/32
			SPSR |= (1<<SPI2X);						// SPI double speed
			pointer->speed = speed;					//
		break;										//
		case 64:									//
			SPCR |= (1<<SPR1) & (0<<SPR0);			// F_CPU/64
			SPSR &= (0<<SPI2X);						// No SPI double speed
			pointer->speed = speed;					//
		break;										//
		case 128:									//
			SPCR |= (1<<SPR1)  | (1<<SPR0);			// F_CPU/128
			SPSR &= (0<<SPI2X);						// No SPI double speed
			pointer->speed = speed;					//
		break;										//
		default:									//
			setError(errorSignalPTR, ERROR_SPI_SPEED, SPI_INIT_funcID);
		break;										//
	}
	SPCR |= (1<<SPIE);								// Enable SPI interrupts
	SPCR |= (1<<SPE);								// Enable SPI
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

void adcStart(struct ADConverter *ADConverter){
	if(ISR_ADC){
		setError(errorSignalPTR, ERROR_ADC_BUSY, START_ADC_funcID);
	}else{
		if (ADConverter->channel<5){
			channelInUsePTR = ADConverter;							// Set channel in use to address of ADConverter
			ADMUX |= (1<<REFS1) | (0<<REFS0);						// Set voltage reference to VCC
			ADMUX |= (1<<ADLAR);									// Result left-justified (8 MSB on ADCH and 2 LSB on ADCL)
			ADMUX |= (ADConverter->channel << MUX0);				// Set ADC channel as requested
			ADCSRA |= (1 << ADSC);									// Start conversion
		}else{
			setError(errorSignalPTR, ERROR_INVALID_ADC_CHANNEL, START_ADC_funcID);
		}
	}
}
void adcPoll(struct ADConverter *ADConverter){
	if(ISR_ADC){
		ADConverter->result = ADC;
		ISR_ADC = 0;
	}
}

void addTXRingBuffer(struct RingBuffer *pointer, uint8_t data){
	if(pointer->dataAmount<USART_BUFFER_SIZE){				// Check if TXBuffer is full
		pointer->data[pointer->inputPosition] = data;		// Put Data in buffer
		pointer->dataAmount ++;								// Increment elements count in TX buffer
		if(pointer->inputPosition<(USART_BUFFER_SIZE-1)){	// Check if end of buffer
			pointer->inputPosition ++;						// Append at the end of buffer
		}else{												//
			pointer->inputPosition = 0;						// Back to beginning of buffer
		}
	}else{
		setError(errorSignalPTR, ERROR_TX_BUFFER_FULL, USART_SEND_funcID);	// Set error if buffer is full
	}
}
void cycleTXRingBuffer(struct RingBuffer *pointer){
	if(ISR_USARTTX && pointer->dataAmount>0){				// Check if previous TX is finished
		ISR_USARTTX = 0;									// Flag pending TX
		UDR0 = pointer->data[pointer->outputPosition];		// Put TX data in UDR register
		if(pointer->outputPosition<(USART_BUFFER_SIZE-1)){	// Check if end of buffer
			pointer->outputPosition ++;						// Append at the end of buffer
		}else{												//
			pointer->outputPosition = 0;					// Back to start of buffer
		}
	}else{
		setError(errorSignalPTR, ERROR_TX_BUSY, USART_CYCLE_TX_funcID);
	}
}
uint8_t retrieveRXRingBuffer(struct USART *usart){
	uint8_t result = 0;															// Initialize result
	if(usart->rxBufferPTR->dataAmount>0){										// Check if unread data are in the RX buffer
		result = usart->rxBufferPTR->data[usart->rxBufferPTR->outputPosition];	// Get Data from the buffer
		if(usart->rxBufferPTR->outputPosition<(USART_BUFFER_SIZE-1)){			// Check if end of buffer
			usart->rxBufferPTR->outputPosition ++;								// Increment RX output position
		}else{																	//
			usart->rxBufferPTR->outputPosition = 0;								// Back to start of buffer
		}
		usart->rxBufferPTR->outputPosition--;									// Decrement number of elements in the buffer
	}else{
		setError(errorSignalPTR, ERROR_RX_BUFFER_EMPTY, USART_RECEIVE_funcID);
	}
	return result;
}
void cycleRXRingBuffer(struct RingBuffer *pointer){
	if(pointer->dataAmount<USART_BUFFER_SIZE){				// Check if RX buffer is not full
		pointer->data[pointer->inputPosition] = UDR0;		// Add data to RX buffer
		if(pointer->inputPosition<(USART_BUFFER_SIZE-1)){	// Check if end of buffer
			pointer->inputPosition ++;						// Increment RX input position
		}else{												//
			pointer->inputPosition  = 0;					// Back to beginning of buffer
		}
		ISR_USARTRX = 0;									// Flag new data can be received
	}else{
		setError(errorSignalPTR, ERROR_RX_BUFFER_FULL, USART_CYCLE_RX_funcID);
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
	ISR_SPI = 1;
}
ISR(USART_RX_vect){									// USART RX Complete
	ISR_USARTRX = 1;								// Flag complete incoming RX
	usartPTR->rxBufferPTR->dataAmount++;			// Increase element count in RX buffer
}
ISR(USART_UDRE_vect){								// USART Data Register Empty
	ISR_USARTUDRE = 1;
}
ISR(USART_TX_vect){									// USART TX Complete
	ISR_USARTTX = 1;								// Flag complete TX
	usartPTR->txBufferPTR->dataAmount++;			// Increase element count in RX buffer
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