#include "message_handler.h"
#include "driver_setup.h"
#include "structs.h"

void messageHandler_RegisterMessage(messageHandler_t* handler, uint8_t id, uint8_t* payload, uint32_t payload_length)
{
	handler->message_id = id;
	handler->payload = payload;
	handler->payload_length = payload_length;
}

extern uint8_t driver_standby_flag;
extern uint8_t driver_configured;
extern inputFormat_t usb_input;
void messageHandler_ApplyMessage(messageHandler_t* handler)
{
	if(handler->message_id == messageControlSignal_ID)
	{
		messageControlSignal_t* message = (messageControlSignal_t*)(handler->payload);
		usb_input.lin_vel = message->lin_vel;
		usb_input.rot_vel = message->rot_vel;
	}
	else if(handler->message_id == messageDriverState_ID)
	{
		if(driver_configured == 0)
			return;
		messageDriverState_t* message = (messageDriverState_t*)(handler->payload);

		if(message->state == 1)
			driver_standby_flag = 0;
		else if(message->state == 0)
			driver_standby_flag = 1;
	}
	else if(handler->message_id == messageDriverConfiguration_ID)
	{
		messageDriverConfiguration_t* message = (messageDriverConfiguration_t*)(handler->payload);
		driverConfig_t config;
		config.motor_type = message->motor_type;
		config.actuation_type = message->actuation_type;
		config.curr_measure_gainAV = message->curr_measure_gainAV;
		config.enc_cpr_L = message->enc_cpr_L;
		config.enc_cpr_R = message->enc_cpr_R;

		config.inner_loop_pid_L.p = message->inner_loop_pid_L_P;
		config.inner_loop_pid_L.i = message->inner_loop_pid_L_I;
		config.inner_loop_pid_L.d = message->inner_loop_pid_L_D;

		config.outer_loop_pid_L.p = message->outer_loop_pid_L_P;
		config.outer_loop_pid_L.i = message->outer_loop_pid_L_I;
		config.outer_loop_pid_L.d = message->outer_loop_pid_L_D;

		config.inner_loop_pid_R.p = message->inner_loop_pid_R_P;
		config.inner_loop_pid_R.i = message->inner_loop_pid_R_I;
		config.inner_loop_pid_R.d = message->inner_loop_pid_R_D;

		config.outer_loop_pid_R.p = message->outer_loop_pid_R_P;
		config.outer_loop_pid_R.i = message->outer_loop_pid_R_I;
		config.outer_loop_pid_R.d = message->outer_loop_pid_R_D;

		config.max_vel = message->max_vel;
		config.pid_output_bound = message->pid_output_bound;
		config.ppm_max_t_us = message->ppm_max_t_us;
		config.ppm_min_t_us = message->ppm_min_t_us;
		config.ppm_no_of_channels = message->ppm_no_of_channels;

		config.safety_level0_maxRPM = message->safety_level0_maxRPM;
		config.safety_level0_minRPM = message->safety_level0_minRPM;
		config.safety_level1_maxRPM = message->safety_level1_maxRPM;
		config.safety_level1_minRPM = message->safety_level1_minRPM;

		driverSetup(&config);
		driver_configured = 1;
	}

}
