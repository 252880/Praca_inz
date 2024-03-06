#ifndef ENCODER_H
#define ENCODER_H

#define __LL_TIM_IS_TIM_COUNTING_DOWN(__HANDLE__) (((__HANDLE__)->CR1 &(TIM_CR1_DIR)) == (TIM_CR1_DIR))

#include "stm32f4xx_ll_tim.h"

typedef struct
{
	TIM_TypeDef* timer;
	uint32_t CPR; //Cycles per revolution
	int32_t RPM;
}encoder_t;

void encoder_Init(encoder_t* encoder, TIM_TypeDef* timer, uint32_t CPR);
void encoder_MeasureRPM(encoder_t* encoder, uint32_t period_ms);
int32_t encoder_GetRPM(encoder_t* encoder);

#endif
