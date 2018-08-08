#include "ADC.h"

ADC{
	uint8_t id;
	uint8_t channel;
	uint16_t result;
	uint8_t pin;
};
ADC current;
ADC *currentPTR = &current;
ADC tempHBridgeA;
ADC *tempHBridgeAPTR = &tempHBridgeA;
ADC tempHBridgeB;
ADC *tempHBridgeBPTR = &tempHBridgeB;
ADC tempMotor;
ADC *tempMotorPTR = &tempMotor;