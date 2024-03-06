#include "motor.h"
#include "stm32f4xx_ll_tim.h"
#include "hoverserial.h"gvv

void motor_Init(motor_t* motor,	motorConfigStuct_t* config, PWM_t pwm1, PWM_t pwm2, GPIO_t out)
{
	motor->config = config;
	motor->pwm1 = pwm1;
	motor->pwm2 = pwm2;
	motor->out = out;
	return;
}
void motor_Actuate(motor_t* motor, double control_signal)
{
	switch(motor->config->actuation_type)
	{
		case D_MOTOR_CONTROL_TYPE_LPWM_RPWM:
			motor_Actuate_LPWM_RPWM(motor, control_signal);
			break;
		case D_MOTOR_CONTROL_TYPE_PWM_DIR:
			motor_Actuate_PWM_DIR(motor, control_signal);
			break;
		case D_MOTOR_CONTROL_TYPE_STEP_DIR:
			motor_Actuate_STEP_DIR(motor, control_signal);
			break;

	}
	return;
}

void motor_Actuate_PWM_DIR(motor_t* motor, double control_signal)
{
	if(control_signal < 0)
	{
		LL_GPIO_ResetOutputPin(motor->out.port, motor->out.pin);
		control_signal = -control_signal;
	}
	else
		LL_GPIO_SetOutputPin(motor->out.port, motor->out.pin);

	uint32_t remapped_signal = motor_RemapControlSignal(motor, control_signal);


	switch(motor->pwm1.channel)
	{
		case LL_TIM_CHANNEL_CH1:
			LL_TIM_OC_SetCompareCH1(motor->pwm1.timer, remapped_signal);
			break;
		case LL_TIM_CHANNEL_CH2:
			LL_TIM_OC_SetCompareCH2(motor->pwm1.timer, remapped_signal);
			break;
		case LL_TIM_CHANNEL_CH3:
			LL_TIM_OC_SetCompareCH3(motor->pwm1.timer, remapped_signal);
			break;
		case LL_TIM_CHANNEL_CH4:
			LL_TIM_OC_SetCompareCH4(motor->pwm1.timer, remapped_signal);
			break;
	}
	return;
}

void motor_Actuate_LPWM_RPWM(motor_t* motor, double control_signal)
{


	uint32_t pwm1_coeff, pwm2_coeff;
	if(control_signal <= 0)
	{
		pwm1_coeff = 0;
		pwm2_coeff = 1;
		control_signal = -control_signal;
	}
	else
	{
		pwm1_coeff = 1;
		pwm2_coeff = 0;
	}


	uint32_t remapped_signal = motor_RemapControlSignal(motor, control_signal);
	switch(motor->pwm1.channel)
	{
		case LL_TIM_CHANNEL_CH1:
			LL_TIM_OC_SetCompareCH1(motor->pwm1.timer, remapped_signal * pwm1_coeff);
			break;
		case LL_TIM_CHANNEL_CH2:
			LL_TIM_OC_SetCompareCH2(motor->pwm1.timer, remapped_signal * pwm1_coeff);
			break;
		case LL_TIM_CHANNEL_CH3:
			LL_TIM_OC_SetCompareCH3(motor->pwm1.timer, remapped_signal * pwm1_coeff);
			break;
		case LL_TIM_CHANNEL_CH4:
			LL_TIM_OC_SetCompareCH4(motor->pwm1.timer, remapped_signal * pwm1_coeff);
			break;
	}
	switch(motor->pwm2.channel)
	{
		case LL_TIM_CHANNEL_CH1:
			LL_TIM_OC_SetCompareCH1(motor->pwm2.timer, remapped_signal * pwm2_coeff);
			break;
		case LL_TIM_CHANNEL_CH2:
			LL_TIM_OC_SetCompareCH2(motor->pwm2.timer, remapped_signal * pwm2_coeff);
			break;
		case LL_TIM_CHANNEL_CH3:
			LL_TIM_OC_SetCompareCH3(motor->pwm2.timer, remapped_signal * pwm2_coeff);
			break;
		case LL_TIM_CHANNEL_CH4:
			LL_TIM_OC_SetCompareCH4(motor->pwm2.timer, remapped_signal * pwm2_coeff);
			break;
	}
	return;
}

void motor_Actuate_STEP_DIR(motor_t* motor, double control_signal)
{
	if(control_signal < 0)
		{
			LL_GPIO_ResetOutputPin(motor->out.port, motor->out.pin);
			control_signal = -control_signal;
		}
		else
			LL_GPIO_SetOutputPin(motor->out.port, motor->out.pin);

		uint32_t remapped_signal = motor_RemapControlSignal(motor, control_signal);
		remapped_signal = remapped_signal / 10;

		uint32_t freq = (remapped_signal * (STEPPER_MOTOR_MAX_FREQ_HZ - STEPPER_MOTOR_MIN_FREQ_HZ))/STEPPER_MOTOR_MAX_SPEED;
		uint32_t counter = 2 * HAL_RCC_GetPCLK1Freq() / (LL_TIM_GetPrescaler(motor->pwm1.timer) * freq);
		//uint32_t counter = HAL_RCC_GetPCLK1Freq() / (LL_TIM_GetPrescaler(motor->pwm1.timer) * freq);

motor->counter_1 = counter;

		LL_TIM_SetCounter(motor->pwm1.timer,0);
		LL_TIM_SetAutoReload(motor->pwm1.timer,counter - 1);

		switch(motor->pwm1.channel)
		{
			case LL_TIM_CHANNEL_CH1:
				LL_TIM_OC_SetCompareCH1(motor->pwm1.timer, (counter/2) - 1);
				break;
			case LL_TIM_CHANNEL_CH2:
				LL_TIM_OC_SetCompareCH2(motor->pwm1.timer, (counter/2) - 1);
				break;
			case LL_TIM_CHANNEL_CH3:
				LL_TIM_OC_SetCompareCH3(motor->pwm1.timer, (counter/2) - 1);
				break;
			case LL_TIM_CHANNEL_CH4:
				LL_TIM_OC_SetCompareCH4(motor->pwm1.timer, (counter/2) - 1);
				break;
		}

	return;
}

void motor_Actuate_UART( double control_signal,double control_signal_1)
{

//	Receive(&byte);




	uint32_t remapped_signal = control_signal * 400;
	uint32_t remapped_signal_1 =  control_signal_1 * 400;


		Send(remapped_signal , remapped_signal_1 );

}

uint32_t motor_RemapControlSignal(motor_t* motor, double control_signal)
{
	return motor->config->max_pwm_output * control_signal;
}
