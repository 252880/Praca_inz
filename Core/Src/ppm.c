#include "ppm.h"
#include "ibus.h"

//timer configured to tick every 1 microsecond - used by all ppm instances as time reference
TIM_TypeDef* timer_us = TIM5;

void ppm_Init(ppm_t* ppm, ppmConfigStruct_t* config)
{
	ppm->config = config;
	ppm->pulse_no = 0;
	ppm->motor_power = 0;
	ppm->control_type = D_PPM_CONTROL_TYPE_MANUAL;
	return;
}
void ppm_ComputeChannelValue(ppm_t* ppm, uint8_t channel)
{
	if((ppm->timers[channel] - ppm->timers[channel - 1] < ppm->config->min_t_us))
		ppm->channel_values[channel - 1] = 0.0f;
	else
		ppm->channel_values[channel - 1] = (ppm->timers[channel] - ppm->timers[channel - 1] - ppm->config->min_t_us) / (double)(ppm->config->max_t_us - ppm->config->min_t_us);
	if(ppm->channel_values[channel - 1] >= 1.2f)
	{
		ppm->timers[0] = 0;//ppm->timers[channel];
		LL_TIM_SetCounter(timer_us, 0);
		ppm->pulse_no = 1;
		ppm_UpdateControlSignal(ppm);
	}
	else
		ppm->pulse_no++;
}

void ppm_Interrupt(ppm_t* ppm)
{
	if(LL_GPIO_IsInputPinSet(ppm->config->gpio.port, ppm->config->gpio.pin) == ppm->config->trigger_state)
	{
		ppm->timers[ppm->pulse_no] = LL_TIM_GetCounter(timer_us);
		if(ppm->pulse_no > 0)
		{
			ppm_ComputeChannelValue(ppm, ppm->pulse_no);
		}
		else
			ppm->pulse_no++;
	}
}

uint8_t ppm_GetMotorPowerState(ppm_t* ppm)
{
	return ppm->motor_power;
}

void ppm_UpdateControlSignal(ppm_t* ppm)
{

	ppm->control_signal.lin_vel = 2*(ppm_GetChannelValue(ppm, 2) - 0.5) * ppm->config->max_vel;
	ppm->control_signal.rot_vel = 2*(ppm_GetChannelValue(ppm, 1) - 0.5) * ppm->config->max_vel;

	double control_type = ppm_GetChannelValue(ppm, 5);
	if(control_type >= 0.5)
		ppm->control_type = D_PPM_CONTROL_TYPE_AUTO;
	else
		ppm->control_type = D_PPM_CONTROL_TYPE_MANUAL;

	double  motor_power = ppm_GetChannelValue(ppm, 6);
	if( motor_power >= 0) //0.5
		ppm->motor_power = 1;
	else
		ppm->motor_power = 0;

	if( ppm->config->no_of_channels > 6)
	{
		ppm->control_signal.max_vel_proc = 100 * ppm_GetChannelValue(ppm, 7);
		if(ppm->control_signal.max_vel_proc > 100)
			ppm->control_signal.max_vel_proc = 100;
	}
}
inputFormat_t ppm_GetControlSignal(ppm_t* ppm)
{
	return ppm->control_signal;
}

uint8_t ppm_GetControlType(ppm_t* ppm)
{
	return ppm->control_type;
}

double ppm_GetChannelValue(ppm_t* ppm, uint8_t no_of_channel)
{
	if(ppm->config->rc_type >=0 && ppm->config->rc_type <=2){
	if(ppm->config->rc_type == RC_TYPE_IBUS)
		return  ((ibus_data[no_of_channel - 1]- 1000)/(double)1000);

	if(ppm->config->rc_type == RC_TYPE_PPM)
	return ppm->channel_values[no_of_channel - 1];

	if(ppm->config->rc_type == RC_TYPE_SBUS)
		return ((CH[no_of_channel - 1]- 1000)/(double)1000);}
	else
		return 0;

}

