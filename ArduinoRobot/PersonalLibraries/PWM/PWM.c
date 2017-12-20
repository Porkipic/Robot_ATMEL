#include "PWM.h"

void initPWM(){
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);		// Output set low on Compare A interrupt, output set low on compare B, set mode to fast PWM
	TCCR0B |= (1<<CS02);												// Set prescaler on clock/256s
	OCR0A = 0;															// Reset Compare A value
	OCR0B = 0;															// Reset Compare B value
}
void setPWM(uint8_t pinPWM, uint8_t valuePWM){
	switch(pinPWM){
		case 12:
			OCR0A = valuePWM;											// Set Compare A value. An interrupt will happen with the counter will reach this value
			break;
		case 11:
			OCR0B = valuePWM;											// Set Compare A value. An interrupt will happen with the counter will reach this value
			break;
		default:														// If invalid pin number, generate error
			setError(3);
			break;
	}
}
