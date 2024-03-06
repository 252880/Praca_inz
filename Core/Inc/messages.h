#ifndef INC_MESSAGES_H_
#define INC_MESSAGES_H_

#pragma pack(push,1)

enum message_ID
{
		messageDriverState_ID = 0,
		messageDriverConfiguration_ID = 1,
		messageControlSignal_ID = 2,
};

typedef struct
{
	uint8_t state;
}messageDriverState_t;

typedef struct
{
	uint8_t motor_type;
	uint8_t actuation_type;

	uint32_t inner_loop_pid_L_P;
	uint32_t inner_loop_pid_L_I;
	uint32_t inner_loop_pid_L_D;

	uint32_t outer_loop_pid_L_P;
	uint32_t outer_loop_pid_L_I;
	uint32_t outer_loop_pid_L_D;

	uint32_t inner_loop_pid_R_P;
	uint32_t inner_loop_pid_R_I;
	uint32_t inner_loop_pid_R_D;

	uint32_t outer_loop_pid_R_P;
	uint32_t outer_loop_pid_R_I;
	uint32_t outer_loop_pid_R_D;

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
}messageDriverConfiguration_t;

typedef struct
{
	int16_t lin_vel;
	int16_t rot_vel;
}messageControlSignal_t;

#pragma pack(pop)

#endif


