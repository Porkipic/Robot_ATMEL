#include "ADC.h"
#include <avr/interrupt.h>

void initADC(){
    ADCSRA |= (1<<ADEN) & (0<<ADATE) & (1<<ADIE) & (7<<ADPS0);
}
void startADC(uint8_t analogPinToRead, uint8_t voltageRef){
    ADMUX |= (voltageRef<<REFS0) & (1<<ADLAR) & (analogPinToRead<<0);
    ADCSRA |= (1<<ADSC);
}

uint8_t getADCValue(){
    return *ADCValue = (ADCH<<2) & (ADCL);
}

ISR(ADC_vect){
    ADCComplete = 1;
}
