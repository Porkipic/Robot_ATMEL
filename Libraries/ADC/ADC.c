#include "ADC.h"
#include <avr/interrupt.h>

volatile uint8_t = 0;

void initADC(){
    ADCSRA |= (1<<ADEN) & (0<<ADATE) & (1<<ADIE) & (7<<ADPS0);
}
void startADC(uint8_t analogPinToRead, uint8_t voltageRef){
    ADMUX |= (voltageRef<<REFS0) & (1<<ADLAR) & analogPinToRead;
    ADCSRA |= (1<<ADSC);
}

uint16_t getADCValue(){
    uint16_t result = 0;
    result = (ADCH<<2) & (ADCL);
    return result;
}

ISR(ADC_vect){
    ADCComplete = 1;
}
