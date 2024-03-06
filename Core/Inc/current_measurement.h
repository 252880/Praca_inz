#ifndef CURRENT_MEASUREMENT_H
#define CURRENT_MEASUREMENT_H

#include "stm32f4xx_ll_adc.h"

typedef struct
{
	ADC_TypeDef* adc;
	uint32_t adc_channel;
	double gainAV;
}currentMeasurementConfigStruct_t;

typedef struct
{
	currentMeasurementConfigStruct_t* config;
	uint32_t adc_measurement;
	//int32_t current_mA;
	float current_mA;
}currentMeasurement_t;

void currentMeasurement_Init(currentMeasurement_t* currMeasure, currentMeasurementConfigStruct_t* config);
void currentMeasurement_MeasueCurrent(currentMeasurement_t* currMeasure);
void currentMeasurement_ConvAdcToCurrent(currentMeasurement_t* currMeasure);
int32_t currentMeasurement_GetCurrent(currentMeasurement_t* currMeasure);
#endif
