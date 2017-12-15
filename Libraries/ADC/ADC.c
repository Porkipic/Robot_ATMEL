#include "ADC.h"
#include <avr/interrupt.h>

void initADC(){
    ADCSRA |= (1<<ADEN) & (0<<ADATE) & (1<<ADIE) & (7<<ADPS0);
}
void readADCbits(uint8_t analogPinToRead, uint8_t voltageRef){
    ADMUX |= (voltageRef<<REFS0) & (1<<ADLAR) & (analogPinToRead<<0);
    ADCSRA |= (1<<ADSC);
}

void getADCValue(uint13_t &ADCValue){
    return *ADCValue = (ADCH<<2) & (ADCL);
}

ISR(ADC_vect){
    ADCComplete = 1;
}
