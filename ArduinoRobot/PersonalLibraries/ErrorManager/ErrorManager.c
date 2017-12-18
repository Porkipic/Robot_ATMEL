#include "ErrorManager.h"

uint8_t port = 0;
uint8_t pin = 18;														// To be changed by pin 19. Damaged board has pin 19 always on.
uint8_t error = 0;

void initErrorPin(uint8_t errorPin){
	pin = errorPin;
	if (errorPin>=2 && errorPin<= 13){
		port = 1;
		pin = errorPin-2;
		DDRD |= (1<<pin);
	}
	if (errorPin>=14 && errorPin<= 19){
		port = 2;
		pin = errorPin-14;
		DDRB |= (1<<pin);
	}
	if (errorPin>=23 && errorPin<= 28){
		port = 3;
		pin = errorPin-23;
		DDRC |= (1<<pin);
	}
}

void setError(uint8_t errorCode){
		error = errorCode;
		switch(port){
			case 0:
				initErrorPin(pin);
				setError(0);
				break;
			case 1:
				PORTD |= (1<<pin);
				break;
			case 2:
				PORTB |= (1<<pin);
				break;
			case 3:
				PORTC |= (1<<pin);
				break;
			default:
				initErrorPin(pin);
				setError(0);
				break;
		}
}

void clearError(){
	switch(port){
		case 0:
			initErrorPin(pin);
			setError(0);
			break;
		case 1:
			PORTD &= (0<<pin);
			break;
		case 2:
			PORTB &= (0<<pin);
			break;
		case 3:
			PORTC &= (0<<pin);
			break;
		default:
			initErrorPin(pin);
			setError(0);
			break;
	}
}