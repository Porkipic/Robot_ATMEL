#include "ADC.h"

////////////////////////////// ADC //////////////////////////////////////////////
//********** Initialize ADC **********
void initADC(){
	ISR_ADC = 0;																		// Reset to allow nez conversion after initialization
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);	// Enable ADC, Enable Interrupt, Set prescaler to 128
}
//********************

//********** Start ADC conversion **********
void startADC(uint8_t channel){
	if (ISR_ADC){																		// Prevent new conversion if previous is not cleared
		setError(1);																	// Set error in Error manager (1 = error in ADC)
	}else{
		ADMUX = (1 << REFS0) | (channel << MUX0);										// Set voltage reference (VCC), set ADC channel
		ADCSRA |= (1 << ADSC);															// Start conversion
	}
}
//********************

//********** Handle ADC **********
uint16_t getADCValue(){
	uint16_t result = ADC;																// Set ADC value to result
	ISR_ADC = 0;																		// Reset ISR_ADC to allow new conversion
	return result;
}
//********************

//********** ADC ISR **********
ISR(ADC_vect){
	ISR_ADC = 1;																		// Set ISR_ADC. Variable to be polled in Main code
}
//********************
////////////////////////////// END ADC //////////////////////////////////////////