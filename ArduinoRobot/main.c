#include <avr/io.h>
#include"util/delay.h"

int main (void){ 
	DDRD = 0xFF;
	while (1){ 
		PORTD ^=  0xFF;	//toggle port B
		_delay_ms(1000);	//wait 1 second 
	} 
} 