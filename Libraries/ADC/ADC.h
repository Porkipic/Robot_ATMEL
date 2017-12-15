#ifndef ADC
#define ADC

#define AREF 0;
#define AVCC 1;
#define INTERNAL 3;

void initADC();
uint16_t readADCbits(uint8_t analogPinToRead, uint8_t voltageRef);

#endif
