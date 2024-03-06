#include "pid.h"


void pid_Init(pid_controller_t* pid, pidConfigStruct_t* config)
{
	pid->config = config;
	return;
}

void pid_ChangeSetpoint(pid_controller_t* pid, int32_t setpoint)
{
	pid->setpoint = setpoint;
	return;
}

void pid_Reset(pid_controller_t* pid)
{
	pid->setpoint = 0;
	pid->inner_loop.last_e = 0;
	pid->inner_loop.sum = 0;
	pid->outer_loop.last_e = 0;
	pid->outer_loop.sum = 0;
}

double pid_ComputeOutput(pid_controller_t* pid, int32_t ang_vel, int32_t m_curr)
{
	if(pid->setpoint <= 5 && pid->setpoint >= -5)
	{
		pid_Reset(pid);
		return 0.0f;
	}
	int32_t e_outer = pid->setpoint - ang_vel;

	int32_t p_outer = e_outer * pid->config->outer_loop.p;
	pid->outer_loop.sum += e_outer;
	int32_t i_outer = pid->outer_loop.sum * pid->config->outer_loop.i;
	int32_t d_outer = (e_outer - pid->outer_loop.last_e) * pid->config->outer_loop.i;
	pid->outer_loop.last_e = e_outer;

	int32_t output_outer = p_outer + i_outer + d_outer;

	int32_t output_inner = output_outer;

//	int32_t e_inner = output_outer - m_curr;

//	int32_t p_inner = e_inner * pid->config->inner_loop.p;
//	pid->inner_loop.sum += e_inner;
	//int32_t i_inner = pid->inner_loop.sum * pid->config->inner_loop.i;
	//int32_t d_inner = (e_inner - pid->inner_loop.last_e) * pid->config->inner_loop.i;
	//pid->inner_loop.last_e = e_inner;

	//int32_t output_inner = p_inner + i_inner + d_inner;


	if(output_inner > (int32_t)pid->config->output_bound)
		output_inner = (int32_t)pid->config->output_bound;

	if(output_inner < -(int32_t)pid->config->output_bound)
		output_inner = -(int32_t)pid->config->output_bound;

	return (double)output_inner / (double)pid->config->output_bound;
}
