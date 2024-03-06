#include "current_measurement.h"

void currentMeasurement_Init(currentMeasurement_t* currMeasure, currentMeasurementConfigStruct_t* config)
{
	currMeasure->config = config;
	currMeasure->adc_measurement = 0;
	currMeasure->current_mA = 0;
}
void currentMeasurement_MeasueCurrent(currentMeasurement_t* currMeasure)
{
	LL_ADC_REG_SetSequencerRanks(currMeasure->config->adc, LL_ADC_REG_RANK_1, currMeasure->config->adc_channel);
	LL_ADC_REG_StartConversionSWStart(currMeasure->config->adc);
	while(LL_ADC_REG_GetFlagEndOfConversion(currMeasure->config->adc) != LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
	LL_ADC_ClearFlag_OVR(currMeasure->config->adc);
	currMeasure->adc_measurement = LL_ADC_REG_ReadConversionData32(currMeasure->config->adc);
	currentMeasurement_ConvAdcToCurrent(currMeasure);
	return;
}

void currentMeasurement_ConvAdcToCurrent(currentMeasurement_t* currMeasure)
{
	float voltage =  ( 3.3f * (float)(currMeasure->adc_measurement) / 4095.0f ) * 2.0f; // multiplied due to voltage divider 1:1 on adc input;
	float current = (voltage - 2.5f) * currMeasure->config->gainAV;
	currMeasure->current_mA = current;

	//float voltage =  ( 3.3 * (float)(currMeasure->adc_measurement) / 4095 ) * 2*1.035;
	//float current = (voltage - 2.5)/0.185;   //0.066 0.1
	//currMeasure->current_mA = current;

}

int32_t currentMeasurement_GetCurrent(currentMeasurement_t* currMeasure)
{
	return currMeasure->current_mA;
}
