#ifndef PID_H
#define PID_H
#include <stdint.h>

typedef struct{
	uint32_t p;
	uint32_t i;
	uint32_t d;
}pidCoefficients_t;

typedef struct
{
	pidCoefficients_t inner_loop;
	pidCoefficients_t outer_loop;
	uint32_t output_bound;
}pidConfigStruct_t;

typedef struct
{
	int32_t last_e;
	int32_t sum;
}pidRuntimeVariables_t;

typedef struct
{
	pidConfigStruct_t* config;
	pidRuntimeVariables_t inner_loop;
	pidRuntimeVariables_t outer_loop;
	int32_t setpoint;
}pid_controller_t;

void pid_Init(pid_controller_t* pid, pidConfigStruct_t* config);
void pid_ChangeSetpoint(pid_controller_t* pid, int32_t setpoint);
double pid_ComputeOutput(pid_controller_t* pid, int32_t ang_vel, int32_t m_curr);
void pid_Reset(pid_controller_t* pid);
#endif
