#include "ADC.h"

////////////////////////////// ADC //////////////////////////////////////////////
//********** Initialize ADC **********
void initADC(){
	ISR_ADC = 0;
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
}
//********************

//********** Start ADC conversion **********
void startADC(uint8_t channel){
	if (ISR_ADC){
		setError(1);
	}else{
		ADMUX = (1 << REFS0) | (channel << MUX0);
		ADCSRA |= (1 << ADSC);
	}
}
//********************

//********** Handle ADC **********
uint16_t getADCValue(){
	uint16_t result = ADC;
	ISR_ADC = 0;
	return result;
}
//********************

//********** ADC ISR **********
ISR(ADC_vect){
	ISR_ADC = 1;
}
//********************
////////////////////////////// END ADC //////////////////////////////////////////