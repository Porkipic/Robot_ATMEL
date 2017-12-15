#ifndef ADC
#define ADC

#define AREF 0;
#define AVCC 1;
#define INTERNAL 3;

#define ADC0 0;
#define ADC1 1;
#define ADC2 2;
#define ADC3 3;
#define ADC4 4;
#define ADC4 5;

void initADC();
void startADC(uint8_t analogPinToRead, uint8_t voltageRef);
uint16_t getADCValue();

#endif
