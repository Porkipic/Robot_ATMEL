#ifndef ADC
#define ADC

void initADC();
uint16_t readADCbits(uint8_t analogPinToRead, uint8_t voltageRef = 0);

#endif
