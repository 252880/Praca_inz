#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "pid.h"

typedef struct
{
	GPIO_TypeDef* port;
	uint32_t pin;
}GPIO_t;

typedef struct
{
	TIM_TypeDef* timer;
	uint32_t channel;
}PWM_t;

typedef struct{
	int16_t lin_vel;
	int16_t rot_vel;
	uint16_t max_vel_proc;
}inputFormat_t;

typedef struct{
	uint8_t motor_type;
	uint8_t actuation_type;
	uint8_t rc_type;
	pidCoefficients_t inner_loop_pid_L;
	pidCoefficients_t outer_loop_pid_L;
	pidCoefficients_t inner_loop_pid_R;
	pidCoefficients_t outer_loop_pid_R;
	uint32_t pid_output_bound;
	uint32_t enc_cpr_L;
	uint32_t enc_cpr_R;
	uint8_t	ppm_no_of_channels;
	uint16_t ppm_max_t_us;
	uint16_t ppm_min_t_us;
	uint16_t max_vel;
	int16_t safety_level0_minRPM;
	int16_t safety_level0_maxRPM;
	int16_t safety_level1_minRPM;
	int16_t safety_level1_maxRPM;
	double curr_measure_gainAV;
}driverConfig_t;

#endif
