#ifndef PPM_H
#define PPM_H

#define PPM_MAX_NUM_OF_CHANNELS 10

#include "stm32f4xx_ll_tim.h"
#include "structs.h"



#define D_PPM_CONTROL_TYPE_MANUAL 0
#define D_PPM_CONTROL_TYPE_AUTO 1

#define RC_TYPE_PPM 0
#define RC_TYPE_IBUS 1
#define RC_TYPE_SBUS 2




typedef struct
{
	GPIO_t gpio;
	uint8_t trigger_state;
	uint8_t no_of_channels;
	uint16_t max_t_us;
	uint16_t min_t_us;
	uint16_t max_vel;
	uint8_t rc_type;
}ppmConfigStruct_t;

typedef struct
{
	ppmConfigStruct_t* config;
	double channel_values[PPM_MAX_NUM_OF_CHANNELS + 1];
	uint32_t timers[PPM_MAX_NUM_OF_CHANNELS + 2];
	uint8_t pulse_no;
	uint8_t motor_power;
	uint8_t control_type;
	inputFormat_t control_signal;
}ppm_t;

void ppm_Init(ppm_t* ppm, ppmConfigStruct_t* config);
void ppm_ComputeChannelValue(ppm_t* ppm, uint8_t channel);
double ppm_GetChannelValue(ppm_t* ppm, uint8_t no_of_channel);
void ppm_Interrupt(ppm_t* ppm);
void ppm_UpdateControlSignal(ppm_t* ppm);
inputFormat_t ppm_GetControlSignal(ppm_t* ppm);
uint8_t ppm_GetMotorPowerState(ppm_t* ppm);
uint8_t ppm_GetControlType(ppm_t* ppm);
#endif
