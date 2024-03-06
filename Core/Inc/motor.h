#ifndef MOTOR_HH
#define MOTOR_HH

#include <stdint.h>
#include "structs.h"



#define D_MOTOR_TYPE_STEPPER 0
#define D_MOTOR_TYPE_BLDC 1
#define D_MOTOR_TYPE_DC 2

#define D_MOTOR_CONTROL_TYPE_PWM_DIR 		0x01
#define D_MOTOR_CONTROL_TYPE_LPWM_RPWM		0x02
#define D_MOTOR_CONTROL_TYPE_STEP_DIR		0x03
#define  D_MOTOR_CONTROL_TYPE_UART			0x04


#define STEPPER_MOTOR_MAX_FREQ_HZ	(MICRO_STEP * 1000)
#define STEPPER_MOTOR_MIN_FREQ_HZ	1
#define STEPPER_MOTOR_MAX_SPEED		100
#define MICRO_STEP			32


typedef struct
{
	uint8_t motor_type;
	uint8_t actuation_type;
	uint32_t max_pwm_output;
} motorConfigStuct_t;

typedef struct
{
	motorConfigStuct_t* config;
	PWM_t pwm1;
	PWM_t pwm2;
	GPIO_t out;
	uint32_t counter_1;
}motor_t;

void motor_Init(motor_t* motor,	motorConfigStuct_t* config, PWM_t pwm1, PWM_t pwm2, GPIO_t out);
void motor_Actuate(motor_t* motor, double control_signal);

void motor_Actuate_PWM_DIR(motor_t* motor,  double control_signal);
void motor_Actuate_STEP_DIR(motor_t* motor, double control_signal);
void motor_Actuate_LPWM_RPWM(motor_t* motor, double control_signal);
void motor_Actuate_UART( double control_signal, double control_signal_1);

uint32_t motor_RemapControlSignal(motor_t* motor, double control_signal);

#endif
