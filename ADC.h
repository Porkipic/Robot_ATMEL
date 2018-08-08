#ifndef ADC_H
#define ADC_H

typedef struct ADConverter ADC;

void ADConverterINIT(struct ADConverter *pointer, uint8_t channel);
void adcStart(struct ADConverter *adconverter);
void adcPoll(struct ADConverter *adconverter);

#endif