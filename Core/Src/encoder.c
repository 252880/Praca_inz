#include "encoder.h"

void encoder_Init(encoder_t* encoder, TIM_TypeDef* timer, uint32_t CPR)
{
	encoder->timer = timer;
	encoder->CPR = CPR;
	encoder->RPM = 0;
	return;
}
void encoder_MeasureRPM(encoder_t* encoder, uint32_t period_ms)
{
	int32_t sign = 1;
	uint32_t counter = LL_TIM_GetCounter(encoder->timer);
	LL_TIM_SetCounter(encoder->timer, 0);

	if(__LL_TIM_IS_TIM_COUNTING_DOWN(encoder->timer) )
	{
		counter = ((1 << 16) - 1) - counter + 1;
		sign = -1;
	}
	encoder->RPM = sign * (int32_t)counter * (60 * 1000) / (4 * (int32_t)encoder->CPR * (int32_t)period_ms );

	return;
}

int32_t encoder_GetRPM(encoder_t* encoder)
{
	return encoder->RPM;
}
